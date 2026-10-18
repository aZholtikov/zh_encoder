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
    }

#define ZH_ENCODER_PCNT_HIGH_LIMIT 100
#define ZH_ENCODER_PCNT_LOW_LIMIT -100

/**
 * @brief Internal encoder handle structure.
 *
 * Stores the runtime state of a single encoder: button GPIO state and the software
 * debounce timer, value range and step, PCNT count accumulator, and PCNT unit/channel handles.
 *
 * @note Fields accessed from both ISR and task context are protected by a spinlock
 *       in the public API functions.
 */
struct _zh_encoder_handle_t
{
    bool s_gpio_status;                          /*!< Current encoder button status */
    uint8_t encoder_number;                      /*!< Unique encoder number */
    gpio_num_t s_gpio_number;                    /*!< Encoder button GPIO number */
    float encoder_step;                          /*!< Encoder step per detent */
    float encoder_position;                      /*!< Current encoder position */
    float encoder_min_value;                     /*!< Encoder minimum value */
    float encoder_max_value;                     /*!< Encoder maximum value */
    int32_t count_accumulator;                   /*!< Leftover PCNT counts not yet converted into a full step */
    uint8_t pcnt_counts_per_step;                /*!< PCNT counts per encoder step (detent) */
    pcnt_unit_handle_t pcnt_unit_handle;         /*!< PCNT unit handle */
    pcnt_channel_handle_t pcnt_channel_a_handle; /*!< PCNT channel A handle */
    pcnt_channel_handle_t pcnt_channel_b_handle; /*!< PCNT channel B handle */
    TimerHandle_t button_timer;                  /*!< One-shot debounce timer for the button */
};

/**
 * @brief Queue message structure for encoder ISR processing.
 *
 * Passed from the PCNT watch-point ISR to the shared processing task through
 * the queue. Carries the raw PCNT count together with the source encoder handle.
 *
 * @note The handle pointer is read without holding the spinlock. It is safe because
 *       the handle is freed in zh_encoder_deinit() only after the processing task
 *       has been deleted (the last encoder removal).
 */
typedef struct
{
    int32_t count;               /*!< PCNT count read before clearing (signed) */
    zh_encoder_handle_t *handle; /*!< Pointer to the encoder handle */
} zh_encoder_queue_t;

TaskHandle_t zh_encoder = NULL;
static QueueHandle_t _queue_handle = NULL;
static portMUX_TYPE _spinlock = portMUX_INITIALIZER_UNLOCKED;

static zh_encoder_stats_t _stats = {0};
static zh_vector_t *_vector = NULL;

/**
 * @brief Validate the initialization configuration and copy parameters into the handle.
 *
 * Checks task, queue, range, step, encoder number and watch-point settings. When a
 * vector of registered encoders exists, verifies that the encoder number is not yet used.
 * On success, populates the handle fields and sets the position to the midpoint of the range.
 *
 * @param config Pointer to the encoder configuration structure
 * @param handle Pointer to the encoder handle to populate
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if any configuration parameter is invalid or the encoder number is already present
 */
static esp_err_t _zh_encoder_validate_config(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle);

/**
 * @brief Initialize the PCNT unit, channels, watch points and event callbacks.
 *
 * Configures encoder A/B GPIOs as inputs, creates a PCNT unit with a glitch filter,
 * adds two channels with quadrature edge/level actions, registers high and low watch
 * points, then enables, clears and starts the unit. Any failure frees the resources
 * created so far before returning.
 *
 * @param config Pointer to the encoder configuration structure
 * @param handle Pointer to the encoder handle to store PCNT handles in
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if A/B GPIO numbers are invalid or identical
 * @return ESP_FAIL if any GPIO or PCNT operation fails
 */
static esp_err_t _zh_encoder_pcnt_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle);

/**
 * @brief Initialize the push-button GPIO and register its interrupt handler.
 *
 * Does nothing when the button is disabled (s_gpio_number equals GPIO_NUM_MAX). Installs
 * the shared GPIO ISR service when needed, configures the button pin for any-edge
 * interrupt, creates the one-shot software debounce timer (microseconds rounded up to
 * milliseconds, minimum 1 ms) and registers the button interrupt handler.
 *
 * @param config Pointer to the encoder configuration structure
 * @param handle Pointer to the encoder handle to store button state in
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if the button GPIO number is invalid or equals the A/B GPIO number
 * @return ESP_FAIL if GPIO configuration, ISR service installation or handler registration fails
 */
static esp_err_t _zh_encoder_gpio_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle);

/**
 * @brief Create the shared processing queue for the first registered encoder.
 *
 * The queue is allocated once, when the encoder vector contains exactly one entry.
 *
 * @param config Pointer to the encoder configuration structure
 *
 * @return ESP_OK on success
 * @return ESP_FAIL if queue creation fails
 */
static esp_err_t _zh_encoder_resources_init(const zh_encoder_init_config_t *config);

/**
 * @brief Create the shared ISR processing task for the first registered encoder.
 *
 * The task is created once, when the encoder vector contains exactly one entry,
 * pinned to no specific core with the configured priority and stack size.
 *
 * @param config Pointer to the encoder configuration structure
 *
 * @return ESP_OK on success
 * @return ESP_FAIL if task creation fails
 */
static esp_err_t _zh_encoder_task_init(const zh_encoder_init_config_t *config);

/**
 * @brief PCNT watch-point interrupt handler.
 *
 * Reads and clears the unit count, then posts a queue message to the shared processing
 * task. Increments the queue overflow counter when the queue is full.
 *
 * @param unit PCNT unit handle that reached the watch point
 * @param edata Watch-point event data (unused)
 * @param user_ctx Pointer to the encoder handle passed during callback registration
 *
 * @return true if a higher priority task was woken and a context switch is required
 * @return false otherwise
 */
static bool _zh_encoder_isr_handler(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);

/**
 * @brief Shared processing task body for encoder rotation events.
 *
 * Receives queue messages, converts raw PCNT counts into whole steps using the
 * accumulator, clamps the position to the configured range, snaps values closer to
 * zero than half a step to zero, and posts a rotation event when the position changes.
 * Updates the stack watermark on each iteration.
 *
 * @param pvParameter Task parameter (unused)
 *
 * @return Never returns, the task deletes itself when the queue receive fails
 */
static void _zh_encoder_isr_processing_task(void *pvParameter);

/**
 * @brief Button debounce timer callback.
 *
 * Runs in the timer daemon task context after the debounce period elapses. Reads the
 * button level and posts a button event only when the state actually changed. Increments
 * the event post error counter when posting fails.
 *
 * @note @c s_gpio_status is read and written without holding the spinlock. It is safe
 *       because only this callback ever touches the field, so no concurrent access occurs.
 *
 * @param xTimer Timer handle whose ID holds the encoder handle pointer
 *
 * @return No return value
 */
static void _zh_encoder_button_timer_cb(TimerHandle_t xTimer);

/**
 * @brief Push-button GPIO interrupt handler.
 *
 * Restarts the one-shot software debounce timer on every edge. The button state is
 * checked in the timer callback, so the ISR itself performs no event posting.
 *
 * @param arg Pointer to the encoder handle passed during handler registration
 *
 * @return No return value
 */
static void _zh_encoder_button_isr_handler(void *arg);

ESP_EVENT_DEFINE_BASE(ZH_ENCODER);

esp_err_t zh_encoder_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t **handle)
{
    ZH_LOGI("Encoder initialization started.");
    ZH_ERROR_CHECK(config != NULL && handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder initialization failed. Invalid argument.");
    ZH_ERROR_CHECK(*handle == NULL, ESP_ERR_INVALID_STATE, NULL, "Encoder initialization failed. Encoder is already initialized.");
    *handle = heap_caps_calloc(1, sizeof(zh_encoder_handle_t), MALLOC_CAP_8BIT);
    ZH_ERROR_CHECK(*handle != NULL, ESP_ERR_NO_MEM, NULL, "Encoder initialization failed. Failed to allocate encoder handle.");
    ZH_ERROR_CHECK(_zh_encoder_validate_config(config, *handle) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. Initial configuration check failed.");
    if (_vector == NULL)
    {
        ZH_ERROR_CHECK(zh_vector_init(&_vector, sizeof(uint8_t)) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. Failed to create vector.");
    }
    ZH_ERROR_CHECK(zh_vector_push_back(&_vector, &config->encoder_number) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. Failed to add vector data.");
    ZH_ERROR_CHECK(_zh_encoder_resources_init(config) == ESP_OK, ESP_FAIL, zh_vector_delete_back(&_vector); heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. Resources initialization failed.");
    ZH_ERROR_CHECK(_zh_encoder_task_init(config) == ESP_OK, ESP_FAIL,
                   zh_vector_delete_back(&_vector);
                   heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. Processing task initialization failed.");
    ZH_ERROR_CHECK(_zh_encoder_pcnt_init(config, *handle) == ESP_OK, ESP_FAIL,
                   zh_vector_delete_back(&_vector);
                   heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. PCNT initialization failed.");
    ZH_ERROR_CHECK(_zh_encoder_gpio_init(config, *handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(pcnt_unit_stop((*handle)->pcnt_unit_handle) == ESP_OK, NULL, "PCNT unit stop fail.")};
                   {ZH_ERROR_CHECK_CONT(pcnt_unit_disable((*handle)->pcnt_unit_handle) == ESP_OK, NULL, "PCNT unit disable fail.")};
                   {ZH_ERROR_CHECK_CONT(pcnt_del_channel((*handle)->pcnt_channel_a_handle) == ESP_OK, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK_CONT(pcnt_del_channel((*handle)->pcnt_channel_b_handle) == ESP_OK, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK_CONT(pcnt_del_unit((*handle)->pcnt_unit_handle) == ESP_OK, NULL, "PCNT delete unit fail.")};
                   zh_vector_delete_back(&_vector); heap_caps_free(*handle); *handle = NULL, "Encoder initialization failed. GPIO initialization failed.");
    if (_stats.min_stack_size == 0)
    {
        _stats.min_stack_size = config->stack_size;
    }
    ZH_LOGI("Encoder initialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_deinit(zh_encoder_handle_t **handle)
{
    ZH_LOGI("Encoder deinitialization started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder deinitialization failed. Invalid argument.");
    ZH_ERROR_CHECK_CONT(pcnt_unit_stop((*handle)->pcnt_unit_handle) == ESP_OK, NULL, "Encoder deinitialization failed. PCNT unit stop fail.");
    ZH_ERROR_CHECK_CONT(pcnt_unit_disable((*handle)->pcnt_unit_handle) == ESP_OK, NULL, "Encoder deinitialization failed. PCNT unit disable fail.");
    ZH_ERROR_CHECK_CONT(pcnt_unit_remove_watch_point((*handle)->pcnt_unit_handle, (*handle)->pcnt_counts_per_step) == ESP_OK, NULL, "Encoder deinitialization failed. PCNT unit remove watch point fail.");
    ZH_ERROR_CHECK_CONT(pcnt_unit_remove_watch_point((*handle)->pcnt_unit_handle, -(*handle)->pcnt_counts_per_step) == ESP_OK, NULL, "Encoder deinitialization failed. PCNT unit remove watch point fail.");
    ZH_ERROR_CHECK_CONT(pcnt_del_channel((*handle)->pcnt_channel_a_handle) == ESP_OK, NULL, "Encoder deinitialization failed. PCNT delete channel fail.");
    ZH_ERROR_CHECK_CONT(pcnt_del_channel((*handle)->pcnt_channel_b_handle) == ESP_OK, NULL, "Encoder deinitialization failed. PCNT delete channel fail.");
    ZH_ERROR_CHECK_CONT(pcnt_del_unit((*handle)->pcnt_unit_handle) == ESP_OK, NULL, "Encoder deinitialization failed. PCNT delete unit fail.");
    if ((*handle)->s_gpio_number != GPIO_NUM_MAX)
    {
        ZH_ERROR_CHECK_CONT(gpio_isr_handler_remove((*handle)->s_gpio_number) == ESP_OK, NULL, "Encoder deinitialization failed. Remove GPIO isr handler failed.");
        ZH_ERROR_CHECK_CONT(gpio_reset_pin((*handle)->s_gpio_number) == ESP_OK, NULL, "Encoder deinitialization failed. Reset GPIO failed.");
        if ((*handle)->button_timer != NULL)
        {
            xTimerStop((*handle)->button_timer, portMAX_DELAY);
            xTimerDelete((*handle)->button_timer, portMAX_DELAY);
            (*handle)->button_timer = NULL;
        }
    }
    int32_t index = 0;
    ZH_ERROR_CHECK(zh_vector_find_item(&_vector, &(*handle)->encoder_number, &index) == ESP_OK, ESP_ERR_INVALID_ARG, NULL, "Encoder deinitialization failed. Failed to find vector item.");
    ZH_ERROR_CHECK(zh_vector_delete_item(&_vector, (uint16_t)index) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. Vector delete item failed.");
    uint16_t vector_size = 0;
    ZH_ERROR_CHECK(zh_vector_get_size(&_vector, &vector_size) == ESP_OK, ESP_FAIL, NULL, "Encoder deinitialization failed. Failed to get vector size.");
    if (vector_size == 0)
    {
        if (zh_encoder != NULL)
        {
            vTaskDelete(zh_encoder);
            zh_encoder = NULL;
        }
        if (_queue_handle != NULL)
        {
            vQueueDelete(_queue_handle);
            _queue_handle = NULL;
        }
        ZH_ERROR_CHECK_CONT(zh_vector_free(&_vector) == ESP_OK, NULL, "Encoder deinitialization failed. Free vector failed.");
    }
    heap_caps_free(*handle);
    *handle = NULL;
    ZH_LOGI("Encoder deinitialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_reinit(zh_encoder_handle_t **handle, float min, float max, float step)
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
    (*handle)->count_accumulator = 0;
    taskEXIT_CRITICAL(&_spinlock);
    ZH_ERROR_CHECK(pcnt_unit_clear_count((*handle)->pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "Encoder reinitialization failed. Internal error.");
    ZH_LOGI("Encoder reinitialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_set(zh_encoder_handle_t **handle, float position)
{
    ZH_LOGI("Encoder set position started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder set position failed. Invalid argument.");
    taskENTER_CRITICAL(&_spinlock);
    ZH_ERROR_CHECK(position <= (*handle)->encoder_max_value && position >= (*handle)->encoder_min_value, ESP_ERR_INVALID_ARG, taskEXIT_CRITICAL(&_spinlock), "Encoder set position failed. Invalid argument.");
    (*handle)->encoder_position = position;
    (*handle)->count_accumulator = 0;
    taskEXIT_CRITICAL(&_spinlock);
    ZH_ERROR_CHECK(pcnt_unit_clear_count((*handle)->pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "Encoder set position failed. Internal error.");
    ZH_LOGI("Encoder set position completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_get(zh_encoder_handle_t **handle, float *position)
{
    ZH_LOGI("Encoder get position started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL && position != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder get position failed. Invalid argument.");
    taskENTER_CRITICAL(&_spinlock);
    *position = (*handle)->encoder_position;
    taskEXIT_CRITICAL(&_spinlock);
    ZH_LOGI("Encoder get position completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_reset(zh_encoder_handle_t **handle)
{
    ZH_LOGI("Encoder reset started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder reset failed. Invalid argument.");
    taskENTER_CRITICAL(&_spinlock);
    (*handle)->encoder_position = ((*handle)->encoder_min_value + (*handle)->encoder_max_value) / 2;
    (*handle)->count_accumulator = 0;
    taskEXIT_CRITICAL(&_spinlock);
    ZH_ERROR_CHECK(pcnt_unit_clear_count((*handle)->pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "Encoder reset failed. Internal error.");
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

static esp_err_t _zh_encoder_validate_config(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle)
{
    ZH_ERROR_CHECK(config->task_priority >= 1 && config->stack_size >= configMINIMAL_STACK_SIZE, ESP_ERR_INVALID_ARG, NULL, "Invalid task settings.");
    ZH_ERROR_CHECK(config->queue_size >= 1, ESP_ERR_INVALID_ARG, NULL, "Invalid queue size.");
    ZH_ERROR_CHECK(config->encoder_max_value > config->encoder_min_value, ESP_ERR_INVALID_ARG, NULL, "Invalid encoder min/max value.");
    ZH_ERROR_CHECK(config->encoder_step > 0, ESP_ERR_INVALID_ARG, NULL, "Invalid encoder step.");
    ZH_ERROR_CHECK(config->encoder_number > 0, ESP_ERR_INVALID_ARG, NULL, "Invalid encoder number.");
    ZH_ERROR_CHECK(config->pcnt_counts_per_step > 0, ESP_ERR_INVALID_ARG, NULL, "Invalid pcnt_counts_per_step.");
    if (_vector != NULL)
    {
        int32_t index = 0;
        ZH_ERROR_CHECK(zh_vector_find_item(&_vector, &config->encoder_number, &index) == ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_ARG, NULL, "Encoder number already present.");
    }
    handle->encoder_number = config->encoder_number;
    handle->encoder_min_value = config->encoder_min_value;
    handle->encoder_max_value = config->encoder_max_value;
    handle->encoder_step = config->encoder_step;
    handle->pcnt_counts_per_step = config->pcnt_counts_per_step;
    handle->encoder_position = (handle->encoder_min_value + handle->encoder_max_value) / 2;
    handle->s_gpio_number = GPIO_NUM_MAX;
    return ESP_OK;
}

static esp_err_t _zh_encoder_pcnt_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle)
{
    ZH_ERROR_CHECK(config->a_gpio_number < GPIO_NUM_MAX && config->b_gpio_number < GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, NULL, "Invalid GPIO number.")
    ZH_ERROR_CHECK(config->a_gpio_number != config->b_gpio_number, ESP_ERR_INVALID_ARG, NULL, "Encoder A and B GPIO is same.")
    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << config->a_gpio_number) | (1ULL << config->b_gpio_number),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = config->pullup == true ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ZH_ERROR_CHECK(gpio_config(&io_config) == ESP_OK, ESP_FAIL, NULL, "GPIO initialization failed.");
    pcnt_unit_config_t pcnt_unit_config = {
        .high_limit = ZH_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ZH_ENCODER_PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit_handle = NULL;
    ZH_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_config, &pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT initialization failed.");
    pcnt_glitch_filter_config_t pcnt_glitch_filter_config = {
        .max_glitch_ns = 1000,
    };
    ZH_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_handle, &pcnt_glitch_filter_config) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    pcnt_chan_config_t pcnt_chan_a_config = {
        .edge_gpio_num = config->a_gpio_number,
        .level_gpio_num = config->b_gpio_number,
    };
    pcnt_channel_handle_t pcnt_channel_a_handle = NULL;
    ZH_ERROR_CHECK(pcnt_new_channel(pcnt_unit_handle, &pcnt_chan_a_config, &pcnt_channel_a_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    pcnt_chan_config_t pcnt_chan_b_config = {
        .edge_gpio_num = config->b_gpio_number,
        .level_gpio_num = config->a_gpio_number,
    };
    pcnt_channel_handle_t pcnt_channel_b_handle = NULL;
    ZH_ERROR_CHECK(pcnt_new_channel(pcnt_unit_handle, &pcnt_chan_b_config, &pcnt_channel_b_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK_CONT(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_channel_a_handle, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK_CONT(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK_CONT(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel_a_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK_CONT(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK_CONT(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_channel_b_handle, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK_CONT(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK_CONT(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel_b_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK_CONT(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK_CONT(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_handle, handle->pcnt_counts_per_step) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_handle, -handle->pcnt_counts_per_step) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, handle->pcnt_counts_per_step) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    pcnt_event_callbacks_t cbs = {
        .on_reach = _zh_encoder_isr_handler,
    };
    ZH_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit_handle, &cbs, handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, handle->pcnt_counts_per_step) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, -handle->pcnt_counts_per_step) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, handle->pcnt_counts_per_step) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, -handle->pcnt_counts_per_step) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_disable(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT unit disable fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, handle->pcnt_counts_per_step) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, -handle->pcnt_counts_per_step) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_start(pcnt_unit_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_disable(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT unit disable fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, handle->pcnt_counts_per_step) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, -handle->pcnt_counts_per_step) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    handle->pcnt_unit_handle = pcnt_unit_handle;
    handle->pcnt_channel_a_handle = pcnt_channel_a_handle;
    handle->pcnt_channel_b_handle = pcnt_channel_b_handle;
    return ESP_OK;
}

static esp_err_t _zh_encoder_gpio_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle)
{
    ZH_ERROR_CHECK(config->s_gpio_number <= GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, NULL, "Invalid GPIO number.");
    ZH_ERROR_CHECK(config->a_gpio_number != config->s_gpio_number && config->b_gpio_number != config->s_gpio_number, ESP_ERR_INVALID_ARG, NULL, "Encoder GPIO and button GPIO is the same.");
    if (config->s_gpio_number != GPIO_NUM_MAX)
    {
        gpio_config_t pin_config = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = (config->pullup == true) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pin_bit_mask = (1ULL << config->s_gpio_number),
            .intr_type = GPIO_INTR_ANYEDGE};
        ZH_ERROR_CHECK(gpio_config(&pin_config) == ESP_OK, ESP_FAIL, NULL, "GPIO initialization failed.");
        uint32_t debounce_ms = (config->s_gpio_debounce_time + 999) / 1000;
        if (debounce_ms < 1)
        {
            debounce_ms = 1;
        }
        handle->button_timer = xTimerCreate("zh_encoder_button_timer", pdMS_TO_TICKS(debounce_ms), pdFALSE, handle, _zh_encoder_button_timer_cb);
        ZH_ERROR_CHECK(handle->button_timer != NULL, ESP_FAIL, gpio_isr_handler_remove(config->s_gpio_number); gpio_reset_pin(config->s_gpio_number), "Failed to create button debounce timer.");
        handle->s_gpio_number = config->s_gpio_number;
        handle->s_gpio_status = gpio_get_level(config->s_gpio_number);
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
        ZH_ERROR_CHECK(err == ESP_OK || err == ESP_ERR_INVALID_STATE, ESP_FAIL,
                       {ZH_ERROR_CHECK_CONT(gpio_reset_pin(config->s_gpio_number) == ESP_OK, NULL, "Reset GPIO failed.")};
                       xTimerDelete(handle->button_timer, portMAX_DELAY); handle->button_timer = NULL, "Failed install isr service.");
        ZH_ERROR_CHECK(gpio_isr_handler_add(config->s_gpio_number, _zh_encoder_button_isr_handler, handle) == ESP_OK, ESP_FAIL,
                       {ZH_ERROR_CHECK_CONT(gpio_reset_pin(config->s_gpio_number) == ESP_OK, NULL, "Reset GPIO failed.")};
                       xTimerDelete(handle->button_timer, portMAX_DELAY); handle->button_timer = NULL, "Interrupt initialization failed.");
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

static bool _zh_encoder_isr_handler(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    (void)edata;
    zh_encoder_handle_t *encoder_handle = (zh_encoder_handle_t *)user_ctx;
    zh_encoder_queue_t encoder_queue = {0};
    int count = 0;
    taskENTER_CRITICAL_ISR(&_spinlock);
    if (pcnt_unit_get_count(unit, &count) != ESP_OK)
    {
        taskEXIT_CRITICAL_ISR(&_spinlock);
        return false;
    }
    if (pcnt_unit_clear_count(unit) != ESP_OK)
    {
        taskEXIT_CRITICAL_ISR(&_spinlock);
        return false;
    }
    taskEXIT_CRITICAL_ISR(&_spinlock);
    encoder_queue.count = count;
    encoder_queue.handle = encoder_handle;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
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

static void _zh_encoder_isr_processing_task(void *pvParameter)
{
    (void)pvParameter;
    zh_encoder_queue_t encoder_queue = {0};
    while (xQueueReceive(_queue_handle, &encoder_queue, portMAX_DELAY) == pdTRUE)
    {
        zh_encoder_handle_t *handle = encoder_queue.handle;
        bool position_changed = false;
        bool encoder_status = false;
        float encoder_position = 0.0;
        uint8_t encoder_number = 0;
        taskENTER_CRITICAL(&_spinlock);
        int32_t total = handle->count_accumulator + encoder_queue.count;
        int32_t full_steps = total / (int32_t)handle->pcnt_counts_per_step;
        handle->count_accumulator = total - full_steps * (int32_t)handle->pcnt_counts_per_step;
        if (full_steps != 0)
        {
            float candidate = handle->encoder_position + (float)full_steps * handle->encoder_step;
            if (candidate > handle->encoder_max_value)
            {
                candidate = handle->encoder_max_value;
            }
            else if (candidate < handle->encoder_min_value)
            {
                candidate = handle->encoder_min_value;
            }
            if (fabsf(candidate) < handle->encoder_step * 0.5f)
            {
                candidate = 0.0f;
            }
            if (candidate != handle->encoder_position)
            {
                handle->encoder_position = candidate;
                position_changed = true;
                encoder_status = full_steps > 0 ? true : false;
                encoder_position = candidate;
                encoder_number = handle->encoder_number;
            }
        }
        taskEXIT_CRITICAL(&_spinlock);
        if (position_changed == true)
        {
            zh_encoder_event_on_isr_t encoder_data = {0};
            encoder_data.encoder_number = encoder_number;
            encoder_data.encoder_position = encoder_position;
            encoder_data.encoder_status = encoder_status;
            ZH_ERROR_CHECK_CONT(esp_event_post(ZH_ENCODER, ZH_ENCODER_EVENT, &encoder_data, sizeof(zh_encoder_event_on_isr_t), pdMS_TO_TICKS(10)) == ESP_OK, ++_stats.event_post_error, "Encoder isr processing failed. Failed to post interrupt event.");
        }
        _stats.min_stack_size = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    }
    vTaskDelete(NULL);
}

static void _zh_encoder_button_timer_cb(TimerHandle_t xTimer)
{
    zh_encoder_handle_t *encoder_handle = (zh_encoder_handle_t *)pvTimerGetTimerID(xTimer);
    bool s_gpio_status = gpio_get_level(encoder_handle->s_gpio_number);
    if (encoder_handle->s_gpio_status != s_gpio_status)
    {
        encoder_handle->s_gpio_status = s_gpio_status;
        zh_encoder_button_event_on_isr_t encoder_data = {0};
        encoder_data.encoder_number = encoder_handle->encoder_number;
        encoder_data.button_status = s_gpio_status;
        ZH_ERROR_CHECK_CONT(esp_event_post(ZH_ENCODER, ZH_BUTTON_EVENT, &encoder_data, sizeof(encoder_data), pdMS_TO_TICKS(10)) == ESP_OK, ++_stats.event_post_error, "Encoder button isr processing failed. Failed to post interrupt event.");
    }
}

static void _zh_encoder_button_isr_handler(void *arg)
{
    zh_encoder_handle_t *encoder_handle = (zh_encoder_handle_t *)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTimerResetFromISR(encoder_handle->button_timer, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}