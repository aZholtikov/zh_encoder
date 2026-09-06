#include "zh_encoder.h"

#define TAG "zh_encoder"

#define ZH_LOGI(msg, ...) ESP_LOGI(TAG, msg, ##__VA_ARGS__)
#define ZH_LOGE(msg, err, ...) ESP_LOGE(TAG, "[%s:%d:%s] " msg, __FILE__, __LINE__, esp_err_to_name(err), ##__VA_ARGS__)

#define ZH_ERROR_CHECK(cond, err, cleanup, msg, ...) \
    if (!(cond))                                     \
    {                                                \
        ZH_LOGE(msg, err, ##__VA_ARGS__);            \
        cleanup;                                     \
        return err;                                  \
    }

#define ZH_ERROR_CHECK_CONT(cond, cleanup, msg, ...) \
    if (!(cond))                                     \
    {                                                \
        ZH_LOGE(msg, ESP_FAIL, ##__VA_ARGS__);       \
        cleanup;                                     \
        continue;                                    \
    }

#define ZH_ENCODER_DIRECTION_CW 1
#define ZH_ENCODER_DIRECTION_CCW -1

/**
 * @brief Internal encoder handle structure.
 *
 * Stores all runtime state for a single encoder instance,
 * including GPIO configuration, PCNT handles, and position tracking.
 */
struct _zh_encoder_handle_t
{
    bool s_gpio_status;                          /*!< Current encoder button status */
    uint8_t encoder_number;                      /*!< Unique encoder number */
    gpio_num_t s_gpio_number;                    /*!< Encoder button GPIO number */
    uint16_t s_gpio_debounce_time;               /*!< Encoder button debounce time in microseconds */
    uint64_t s_gpio_prev_time;                   /*!< Timestamp of the previous button interrupt */
    float encoder_step;                          /*!< Encoder step size */
    float encoder_position;                      /*!< Current encoder position */
    float encoder_min_value;                     /*!< Encoder minimum value */
    float encoder_max_value;                     /*!< Encoder maximum value */
    pcnt_unit_handle_t pcnt_unit_handle;         /*!< PCNT unit handle for quadrature decoding */
    pcnt_channel_handle_t pcnt_channel_a_handle; /*!< PCNT channel A handle (A phase) */
    pcnt_channel_handle_t pcnt_channel_b_handle; /*!< PCNT channel B handle (B phase) */
};

/**
 * @brief Queue message structure for encoder ISR processing.
 *
 * Sent from ISR context to the processing task when encoder position changes.
 */
typedef struct
{
    int watch_point_value;       /*!< PCNT watch point value (CW or CCW direction) */
    zh_encoder_handle_t *handle; /*!< Pointer to the encoder handle */
} zh_encoder_queue_t;

TaskHandle_t zh_encoder = NULL;                               /*!< Handle to the shared encoder ISR processing task */
static QueueHandle_t _queue_handle = NULL;                    /*!< Queue for passing encoder events from ISR to processing task */
static portMUX_TYPE _spinlock = portMUX_INITIALIZER_UNLOCKED; /*!< Spinlock for protecting encoder position access */

static zh_encoder_stats_t _stats = {0}; /*!< Global error statistics */

static zh_vector_t *_vector = NULL; /*!< Vector storing encoder numbers */

/**
 * @brief Validate encoder initialization configuration.
 *
 * Checks all configuration parameters for validity and initializes
 * the encoder handle with configuration values.
 *
 * @param config Pointer to encoder configuration structure
 * @param handle Pointer to encoder handle to initialize
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if any configuration parameter is invalid
 */
static esp_err_t _zh_encoder_validate_config(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle);

/**
 * @brief Initialize PCNT peripheral for quadrature decoding.
 *
 * Configures PCNT unit with two channels (A and B phases),
 * sets up glitch filter, edge/level actions for CW/CCW counting,
 * registers watch points and event callbacks.
 *
 * @param config Pointer to encoder configuration structure
 * @param handle Pointer to encoder handle to populate with PCNT handles
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if GPIO numbers are invalid or identical
 * @return ESP_FAIL if PCNT peripheral initialization fails
 */
static esp_err_t _zh_encoder_pcnt_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle);

/**
 * @brief Initialize GPIO pin for encoder button (if configured).
 *
 * Configures the button GPIO as input with interrupt on any edge.
 * Installs ISR service if not already installed and registers the button ISR handler.
 *
 * @param config Pointer to encoder configuration structure
 * @param handle Pointer to encoder handle to populate with GPIO settings
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if GPIO numbers conflict with encoder pins
 * @return ESP_FAIL if GPIO or ISR service initialization fails
 */
static esp_err_t _zh_encoder_gpio_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle);

/**
 * @brief Initialize shared resources (queue) for the first encoder.
 *
 * Creates the FreeRTOS queue used for passing encoder events
 * from ISR context to the processing task. Only executed once
 * when the first encoder is initialized.
 *
 * @param config Pointer to encoder configuration structure
 *
 * @return ESP_OK on success
 * @return ESP_FAIL if queue creation fails
 */
static esp_err_t _zh_encoder_resources_init(const zh_encoder_init_config_t *config);

/**
 * @brief Initialize the shared ISR processing task for the first encoder.
 *
 * Creates the FreeRTOS task that processes encoder events from the queue.
 * Only executed once when the first encoder is initialized.
 *
 * @param config Pointer to encoder configuration structure
 *
 * @return ESP_OK on success
 * @return ESP_FAIL if task creation fails
 */
static esp_err_t _zh_encoder_task_init(const zh_encoder_init_config_t *config);

/**
 * @brief PCNT event callback invoked when watch point is reached.
 *
 * Clears the PCNT counter and sends the watch point value
 * to the processing task queue. Executed in ISR context.
 *
 * @param unit PCNT unit handle that triggered the event
 * @param edata Pointer to PCNT watch event data
 * @param user_ctx Pointer to encoder handle
 */
static bool _zh_encoder_isr_handler(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);

/**
 * @brief FreeRTOS task for processing encoder events from the queue.
 *
 * Receives queue messages from the ISR handler, updates encoder position
 * based on CW/CCW direction, and posts esp_event notifications
 * when position changes. Runs indefinitely until the queue is deleted.
 *
 * @param pvParameter Task parameter (unused)
 */
static void _zh_encoder_isr_processing_task(void *pvParameter);

/**
 * @brief GPIO ISR handler for encoder button press/release.
 *
 * Implements debouncing by checking the time since the last interrupt.
 * Posts a button event via esp_event_isr_post when button state changes.
 * Executed in ISR context.
 *
 * @param arg Pointer to encoder handle
 */
static void _zh_encoder_button_isr_handler(void *arg);

ESP_EVENT_DEFINE_BASE(ZH_ENCODER);

esp_err_t zh_encoder_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t **handle) // -V2008
{
    ZH_LOGI("Encoder initialization started.");
    ZH_ERROR_CHECK(config != NULL && handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder initialization failed. Invalid argument.");
    ZH_ERROR_CHECK(*handle == NULL, ESP_ERR_INVALID_STATE, NULL, "Encoder initialization failed. Encoder is already initialized.");
    *handle = heap_caps_calloc(1, sizeof(zh_encoder_handle_t), MALLOC_CAP_8BIT);
    ZH_ERROR_CHECK(_zh_encoder_validate_config(config, *handle) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. Initial configuration check failed.");
    if (_vector == NULL)
    {
        ZH_ERROR_CHECK(zh_vector_init(&_vector, sizeof(uint8_t)) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. Failed to create vector.");
    }
    ZH_ERROR_CHECK(zh_vector_push_back(&_vector, &config->encoder_number) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. Failed to add vector data.");
    ZH_ERROR_CHECK(_zh_encoder_resources_init(config) == ESP_OK, ESP_FAIL, zh_vector_delete_back(&_vector); heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. Resources initialization failed.");
    // clang-format off
    ZH_ERROR_CHECK(_zh_encoder_task_init(config) == ESP_OK, ESP_FAIL,
                   zh_vector_delete_back(&_vector); heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. Processing task initialization failed.");
    ZH_ERROR_CHECK(_zh_encoder_pcnt_init(config, *handle) == ESP_OK, ESP_FAIL,
                   zh_vector_delete_back(&_vector); heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. PCNT initialization failed.");
    ZH_ERROR_CHECK(_zh_encoder_gpio_init(config, *handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_stop((*handle)->pcnt_unit_handle) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "PCNT unit stop fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_disable((*handle)->pcnt_unit_handle) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "PCNT unit disable fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel((*handle)->pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel((*handle)->pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit((*handle)->pcnt_unit_handle) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "PCNT delete unit fail.")};
                   zh_vector_delete_back(&_vector); heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. GPIO initialization failed.");
    // clang-format on
    if (_stats.min_stack_size == 0)
    {
        _stats.min_stack_size = config->stack_size;
    }
    ZH_LOGI("Encoder initialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_deinit(zh_encoder_handle_t **handle) // -V2008
{
    ZH_LOGI("Encoder deinitialization started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder deinitialization failed. Invalid argument.");
    ZH_ERROR_CHECK(pcnt_unit_stop((*handle)->pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. PCNT unit stop fail.");
    ZH_ERROR_CHECK(pcnt_unit_disable((*handle)->pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. PCNT unit disable fail.");
    ZH_ERROR_CHECK(pcnt_unit_remove_watch_point((*handle)->pcnt_unit_handle, ZH_ENCODER_DIRECTION_CW) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. PCNT unit remove watch point fail.");
    ZH_ERROR_CHECK(pcnt_unit_remove_watch_point((*handle)->pcnt_unit_handle, ZH_ENCODER_DIRECTION_CCW) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. PCNT unit remove watch point fail.");
    ZH_ERROR_CHECK(pcnt_del_channel((*handle)->pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. PCNT delete channel fail.");
    ZH_ERROR_CHECK(pcnt_del_channel((*handle)->pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. PCNT delete channel fail.");
    ZH_ERROR_CHECK(pcnt_del_unit((*handle)->pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. PCNT delete unit fail.");
    if ((*handle)->s_gpio_number != GPIO_NUM_MAX)
    {
        ZH_ERROR_CHECK(gpio_isr_handler_remove((*handle)->s_gpio_number) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. Remove GPIO isr handler failed.");
        ZH_ERROR_CHECK(gpio_reset_pin((*handle)->s_gpio_number) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. Reset GPIO failed.");
    }
    int32_t index = 0;
    ZH_ERROR_CHECK(zh_vector_find_item(&_vector, &(*handle)->encoder_number, &index) == ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_ARG, NULL, "Encoder deinitialization failed. Failed to find vector item.");
    ZH_ERROR_CHECK(zh_vector_delete_item(&_vector, (uint16_t)index) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. Vector delete item failed.");
    uint16_t vector_size = 0;
    ZH_ERROR_CHECK(zh_vector_get_size(&_vector, &vector_size) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. Failed to get vector size.");
    if (vector_size == 0)
    {
        vQueueDelete(_queue_handle);
        _queue_handle = NULL;
        vTaskDelete(zh_encoder);
        zh_encoder = NULL;
        ZH_ERROR_CHECK(zh_vector_free(&_vector) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. Free vector failed.");
    }
    heap_caps_free(*handle);
    *handle = NULL;
    ZH_LOGI("Encoder deinitialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_reinit(zh_encoder_handle_t **handle, float min, float max, float step) // -V2008
{
    ZH_LOGI("Encoder reinitialization started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder reinitialization failed. Invalid argument.");
    ZH_ERROR_CHECK(max > min, ESP_ERR_INVALID_ARG, NULL, "Encoder reinitialization failed. Invalid encoder min/max value.");
    ZH_ERROR_CHECK(step > 0, ESP_ERR_INVALID_ARG, NULL, "Encoder reinitialization failed. Invalid encoder step.");
    taskENTER_CRITICAL(&_spinlock);
    (*handle)->encoder_min_value = min;
    (*handle)->encoder_max_value = max;
    (*handle)->encoder_step = step;
    (*handle)->encoder_position = ((*handle)->encoder_min_value + (*handle)->encoder_max_value) / 2;
    taskEXIT_CRITICAL(&_spinlock);
    ZH_LOGI("Encoder reinitialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_set(zh_encoder_handle_t **handle, float position)
{
    ZH_LOGI("Encoder set position started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder set position failed. Invalid argument.");
    ZH_ERROR_CHECK(position <= (*handle)->encoder_max_value && position >= (*handle)->encoder_min_value, ESP_ERR_INVALID_ARG, NULL, "Encoder set position failed. Invalid argument.");
    taskENTER_CRITICAL(&_spinlock);
    (*handle)->encoder_position = position;
    taskEXIT_CRITICAL(&_spinlock);
    ZH_LOGI("Encoder set position completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_get(zh_encoder_handle_t **handle, float *position)
{
    ZH_LOGI("Encoder get position started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL && position != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder get position failed. Invalid argument.");
    *position = (*handle)->encoder_position;
    ZH_LOGI("Encoder get position completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_reset(zh_encoder_handle_t **handle)
{
    ZH_LOGI("Encoder reset started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder reset failed. Invalid argument.");
    taskENTER_CRITICAL(&_spinlock);
    (*handle)->encoder_position = ((*handle)->encoder_min_value + (*handle)->encoder_max_value) / 2;
    taskEXIT_CRITICAL(&_spinlock);
    ZH_LOGI("Encoder reset completed successfully.");
    return ESP_OK;
}

const zh_encoder_stats_t *zh_encoder_get_stats(void)
{
    return &_stats;
}

void zh_encoder_reset_stats(void)
{
    ZH_LOGI("Error statistic reset started.");
    _stats.event_post_error = 0;
    _stats.queue_overflow_error = 0;
    _stats.min_stack_size = 0;
    ZH_LOGI("Error statistic reset successfully.");
}

static esp_err_t _zh_encoder_validate_config(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle) // -V2008
{
    ZH_ERROR_CHECK(config->task_priority >= 1 && config->stack_size >= configMINIMAL_STACK_SIZE, ESP_ERR_INVALID_ARG, NULL, "Invalid task settings.");
    ZH_ERROR_CHECK(config->queue_size >= 1, ESP_ERR_INVALID_ARG, NULL, "Invalid queue size.");
    ZH_ERROR_CHECK(config->encoder_max_value > config->encoder_min_value, ESP_ERR_INVALID_ARG, NULL, "Invalid encoder min/max value.");
    ZH_ERROR_CHECK(config->encoder_step > 0, ESP_ERR_INVALID_ARG, NULL, "Invalid encoder step.");
    ZH_ERROR_CHECK(config->encoder_number > 0, ESP_ERR_INVALID_ARG, NULL, "Invalid encoder number.");
    if (_vector != NULL)
    {
        int32_t index = 0;
        ZH_ERROR_CHECK(zh_vector_find_item(&_vector, &config->encoder_number, &index) == ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_ARG, NULL, "Encoder number already present.");
    }
    handle->encoder_number = config->encoder_number;
    handle->encoder_min_value = config->encoder_min_value;
    handle->encoder_max_value = config->encoder_max_value;
    handle->encoder_step = config->encoder_step;
    handle->encoder_position = (handle->encoder_min_value + handle->encoder_max_value) / 2;
    return ESP_OK;
}

static esp_err_t _zh_encoder_pcnt_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle) // -V2008
{
    ZH_ERROR_CHECK(config->a_gpio_number < GPIO_NUM_MAX && config->b_gpio_number < GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, NULL, "Invalid GPIO number.")
    ZH_ERROR_CHECK(config->a_gpio_number != config->b_gpio_number, ESP_ERR_INVALID_ARG, NULL, "Encoder A and B GPIO is same.")
    pcnt_unit_config_t pcnt_unit_config = {
        .high_limit = 10,
        .low_limit = -10,
    };
    pcnt_unit_handle_t pcnt_unit_handle = NULL;
    ZH_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_config, &pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT initialization failed.");
    pcnt_glitch_filter_config_t pcnt_glitch_filter_config = {
        .max_glitch_ns = 1000,
    };
    ZH_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_handle, &pcnt_glitch_filter_config) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    pcnt_chan_config_t pcnt_chan_a_config = {
        .edge_gpio_num = config->a_gpio_number,
        .level_gpio_num = config->b_gpio_number,
    };
    pcnt_channel_handle_t pcnt_channel_a_handle = NULL;
    ZH_ERROR_CHECK(pcnt_new_channel(pcnt_unit_handle, &pcnt_chan_a_config, &pcnt_channel_a_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    pcnt_chan_config_t pcnt_chan_b_config = {
        .edge_gpio_num = config->b_gpio_number,
        .level_gpio_num = config->a_gpio_number,
    };
    pcnt_channel_handle_t pcnt_channel_b_handle = NULL;
    ZH_ERROR_CHECK(pcnt_new_channel(pcnt_unit_handle, &pcnt_chan_b_config, &pcnt_channel_b_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_channel_a_handle, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel_a_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_HOLD) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_channel_b_handle, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel_b_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_HOLD) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_handle, ZH_ENCODER_DIRECTION_CW) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_handle, ZH_ENCODER_DIRECTION_CCW) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, ZH_ENCODER_DIRECTION_CW) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    pcnt_event_callbacks_t cbs = {
        .on_reach = _zh_encoder_isr_handler,
    };
    ZH_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit_handle, &cbs, handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, ZH_ENCODER_DIRECTION_CW) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, ZH_ENCODER_DIRECTION_CCW) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, ZH_ENCODER_DIRECTION_CW) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, ZH_ENCODER_DIRECTION_CCW) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_disable(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT unit disable fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, ZH_ENCODER_DIRECTION_CW) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, ZH_ENCODER_DIRECTION_CCW) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_start(pcnt_unit_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_disable(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT unit disable fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, ZH_ENCODER_DIRECTION_CW) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, ZH_ENCODER_DIRECTION_CCW) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    if (config->pullup == false)
    {
        ZH_ERROR_CHECK(gpio_pullup_dis(config->a_gpio_number) == ESP_OK, ESP_FAIL, NULL, "GPIO pullup disable fail.");
        ZH_ERROR_CHECK(gpio_pullup_dis(config->b_gpio_number) == ESP_OK, ESP_FAIL, NULL, "GPIO pullup disable fail.");
    }
    handle->pcnt_unit_handle = pcnt_unit_handle;
    handle->pcnt_channel_a_handle = pcnt_channel_a_handle;
    handle->pcnt_channel_b_handle = pcnt_channel_b_handle;
    return ESP_OK;
}

static esp_err_t _zh_encoder_gpio_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle) // -V2008
{
    ZH_ERROR_CHECK(config->s_gpio_number <= GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, NULL, "Invalid GPIO number.")
    ZH_ERROR_CHECK(config->a_gpio_number != config->s_gpio_number && config->b_gpio_number != config->s_gpio_number, ESP_ERR_INVALID_ARG, NULL, "Encoder GPIO and button GPIO is same.")
    if (config->s_gpio_number != GPIO_NUM_MAX)
    {
        gpio_config_t pin_config = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = (config->pullup == true) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pin_bit_mask = (1ULL << config->s_gpio_number),
            .intr_type = GPIO_INTR_ANYEDGE};
        ZH_ERROR_CHECK(gpio_config(&pin_config) == ESP_OK, ESP_FAIL, NULL, "GPIO initialization failed.");
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
        ZH_ERROR_CHECK(err == ESP_OK || err == ESP_ERR_INVALID_STATE, ESP_FAIL,
                       {ZH_ERROR_CHECK(gpio_reset_pin(config->s_gpio_number) == ESP_OK, ESP_FAIL, NULL, "Reset GPIO failed.")}, "Failed install isr service.");
        ZH_ERROR_CHECK(gpio_isr_handler_add(config->s_gpio_number, _zh_encoder_button_isr_handler, handle) == ESP_OK, ESP_FAIL,
                       {ZH_ERROR_CHECK(gpio_reset_pin(config->s_gpio_number) == ESP_OK, ESP_FAIL, NULL, "Reset GPIO failed.")}, "Interrupt initialization failed.");
        handle->s_gpio_number = config->s_gpio_number;
        handle->s_gpio_debounce_time = config->s_gpio_debounce_time;
        handle->s_gpio_status = gpio_get_level(config->s_gpio_number);
    }
    return ESP_OK;
}

static esp_err_t _zh_encoder_resources_init(const zh_encoder_init_config_t *config)
{
    uint16_t vector_size = 0;
    zh_vector_get_size(&_vector, &vector_size);
    if (vector_size == 1)
    {
        _queue_handle = xQueueCreate(config->queue_size, sizeof(zh_encoder_queue_t));
        ZH_ERROR_CHECK(_queue_handle != NULL, ESP_FAIL, NULL, "Failed to create queue.");
    }
    return ESP_OK;
}

static esp_err_t _zh_encoder_task_init(const zh_encoder_init_config_t *config)
{
    uint16_t vector_size = 0;
    zh_vector_get_size(&_vector, &vector_size);
    if (vector_size == 1)
    {
        ZH_ERROR_CHECK(xTaskCreatePinnedToCore(&_zh_encoder_isr_processing_task, "zh_encoder_isr_processing", config->stack_size, NULL, config->task_priority, &zh_encoder, tskNO_AFFINITY) == pdPASS,
                       ESP_FAIL, NULL, "Failed to create isr processing task.");
    }
    return ESP_OK;
}

static bool IRAM_ATTR _zh_encoder_isr_handler(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    zh_encoder_handle_t *encoder_handle = (zh_encoder_handle_t *)user_ctx;
    zh_encoder_queue_t encoder_queue = {0};
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (pcnt_unit_clear_count(unit) != ESP_OK)
    {
        return false;
    }
    encoder_queue.watch_point_value = edata->watch_point_value;
    encoder_queue.handle = encoder_handle;
    if (xQueueSendFromISR(_queue_handle, &encoder_queue, &xHigherPriorityTaskWoken) != pdTRUE)
    {
        ++_stats.queue_overflow_error;
    }
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        return true;
    };
    return false;
}

static void IRAM_ATTR _zh_encoder_isr_processing_task(void *pvParameter)
{
    (void)pvParameter;
    zh_encoder_queue_t encoder_queue = {0};
    while (xQueueReceive(_queue_handle, &encoder_queue, portMAX_DELAY) == pdTRUE)
    {
        bool is_value_changed = false;
        zh_encoder_handle_t *encoder_handle = encoder_queue.handle;
        switch (encoder_queue.watch_point_value)
        {
        case ZH_ENCODER_DIRECTION_CW:
            if (encoder_handle->encoder_position < encoder_handle->encoder_max_value)
            {
                encoder_handle->encoder_position = encoder_handle->encoder_position + encoder_handle->encoder_step;
                if (encoder_handle->encoder_position > encoder_handle->encoder_max_value)
                {
                    encoder_handle->encoder_position = encoder_handle->encoder_max_value;
                }
                is_value_changed = true;
            }
            break;
        case ZH_ENCODER_DIRECTION_CCW:
            if (encoder_handle->encoder_position > encoder_handle->encoder_min_value)
            {
                encoder_handle->encoder_position = encoder_handle->encoder_position - encoder_handle->encoder_step;
                if (encoder_handle->encoder_position < encoder_handle->encoder_min_value)
                {
                    encoder_handle->encoder_position = encoder_handle->encoder_min_value;
                }
                is_value_changed = true;
            }
            break;
        default:
            break;
        }
        if (is_value_changed == true)
        {
            is_value_changed = false;
            zh_encoder_event_on_isr_t encoder_data = {0};
            encoder_data.encoder_number = encoder_handle->encoder_number;
            if (encoder_handle->encoder_position < 0 && encoder_handle->encoder_position > -1e-12)
            {
                encoder_handle->encoder_position = 0;
            }
            encoder_data.encoder_position = encoder_handle->encoder_position;
            ZH_ERROR_CHECK_CONT(esp_event_post(ZH_ENCODER, ZH_ENCODER_EVENT, &encoder_data, sizeof(zh_encoder_event_on_isr_t), 1000 / portTICK_PERIOD_MS) == ESP_OK, ++_stats.event_post_error, "Encoder isr processing failed. Failed to post interrupt event.");
        }
        _stats.min_stack_size = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    }
    vTaskDelete(NULL);
}

static void IRAM_ATTR _zh_encoder_button_isr_handler(void *arg)
{
    zh_encoder_handle_t *encoder_handle = (zh_encoder_handle_t *)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint64_t _current_us = esp_timer_get_time();
    if (_current_us - encoder_handle->s_gpio_prev_time >= encoder_handle->s_gpio_debounce_time)
    {
        bool s_gpio_status = gpio_get_level(encoder_handle->s_gpio_number);
        if (encoder_handle->s_gpio_status != s_gpio_status)
        {
            encoder_handle->s_gpio_status = s_gpio_status;
            zh_encoder_button_event_on_isr_t encoder_data = {0};
            encoder_data.encoder_number = encoder_handle->encoder_number;
            encoder_data.button_status = encoder_handle->s_gpio_status;
            if (esp_event_isr_post(ZH_ENCODER, ZH_BUTTON_EVENT, &encoder_data, sizeof(zh_encoder_button_event_on_isr_t), &xHigherPriorityTaskWoken) != ESP_OK)
            {
                ++_stats.event_post_error;
            }
        }
    }
    encoder_handle->s_gpio_prev_time = _current_us;
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    };
}