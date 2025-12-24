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

#define ZH_ENCODER_DIRECTION_CW 0x10
#define ZH_ENCODER_DIRECTION_CCW 0x20

static const uint8_t _encoder_matrix[7][4] = {
    {0x03, 0x02, 0x01, 0x00},
    {0x23, 0x00, 0x01, 0x00},
    {0x13, 0x02, 0x00, 0x00},
    {0x03, 0x05, 0x04, 0x00},
    {0x03, 0x03, 0x04, 0x00},
    {0x03, 0x05, 0x03, 0x00},
};

TaskHandle_t zh_encoder = NULL;
static QueueHandle_t _queue_handle = NULL;
static bool _is_initialized = false;
static bool _is_prev_gpio_isr_handler = false;
static zh_encoder_stats_t _stats = {0};

static esp_err_t _zh_encoder_validate_config(const zh_encoder_init_config_t *config);
static esp_err_t _zh_encoder_gpio_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle);
static esp_err_t _zh_encoder_resources_init(const zh_encoder_init_config_t *config);
static esp_err_t _zh_encoder_task_init(const zh_encoder_init_config_t *config);
static void _zh_encoder_isr_handler(void *arg);
static void _zh_encoder_isr_processing_task(void *pvParameter);

ESP_EVENT_DEFINE_BASE(ZH_ENCODER);

esp_err_t zh_encoder_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle)
{
    ZH_LOGI("Encoder initialization started.");
    esp_err_t err = _zh_encoder_validate_config(config);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "Encoder initialization failed. Initial configuration check failed.");
    err = _zh_encoder_gpio_init(config, handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "Encoder initialization failed. GPIO initialization failed.");
    err = _zh_encoder_resources_init(config);
    if (_is_prev_gpio_isr_handler == true)
    {
        ZH_ERROR_CHECK(err == ESP_OK, err, gpio_isr_handler_remove((gpio_num_t)config->a_gpio_number); gpio_isr_handler_remove((gpio_num_t)config->b_gpio_number); gpio_reset_pin((gpio_num_t)config->a_gpio_number);
                       gpio_reset_pin((gpio_num_t)config->b_gpio_number), "Encoder initialization failed. Resources initialization failed.");
    }
    else
    {
        ZH_ERROR_CHECK(err == ESP_OK, err, gpio_isr_handler_remove((gpio_num_t)config->a_gpio_number); gpio_isr_handler_remove((gpio_num_t)config->b_gpio_number); gpio_uninstall_isr_service();
                       gpio_reset_pin((gpio_num_t)config->a_gpio_number); gpio_reset_pin((gpio_num_t)config->b_gpio_number), "Encoder initialization failed. Resources initialization failed.");
    }
    err = _zh_encoder_task_init(config);
    if (_is_prev_gpio_isr_handler == true)
    {
        ZH_ERROR_CHECK(err == ESP_OK, err, vQueueDelete(_queue_handle); _queue_handle = NULL; gpio_isr_handler_remove((gpio_num_t)config->a_gpio_number); gpio_isr_handler_remove((gpio_num_t)config->b_gpio_number);
                       gpio_reset_pin((gpio_num_t)config->a_gpio_number); gpio_reset_pin((gpio_num_t)config->b_gpio_number), "Encoder initialization failed. Processing task initialization failed.");
    }
    else
    {
        ZH_ERROR_CHECK(err == ESP_OK, err, vQueueDelete(_queue_handle); _queue_handle = NULL; gpio_isr_handler_remove((gpio_num_t)config->a_gpio_number); gpio_isr_handler_remove((gpio_num_t)config->b_gpio_number);
                       gpio_uninstall_isr_service(); gpio_reset_pin((gpio_num_t)config->a_gpio_number); gpio_reset_pin((gpio_num_t)config->b_gpio_number), "Encoder initialization failed. Processing task initialization failed.");
    }
    handle->a_gpio_number = config->a_gpio_number;
    handle->b_gpio_number = config->b_gpio_number;
    handle->encoder_number = config->encoder_number;
    handle->encoder_min_value = config->encoder_min_value;
    handle->encoder_max_value = config->encoder_max_value;
    handle->encoder_step = config->encoder_step;
    handle->encoder_position = (handle->encoder_min_value + handle->encoder_max_value) / 2;
    handle->is_initialized = true;
    _is_initialized = true;
    ZH_LOGI("Encoder initialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_set(zh_encoder_handle_t *handle, double position)
{
    ZH_LOGI("Encoder set position started.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_FAIL, NULL, "Encoder set position failed. Encoder not initialized.");
    ZH_ERROR_CHECK(position <= handle->encoder_max_value && position >= handle->encoder_min_value, ESP_ERR_INVALID_ARG, NULL, "Encoder set position failed. Invalid argument.");
    handle->encoder_position = position;
    ZH_LOGI("Encoder set position completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_get(const zh_encoder_handle_t *handle, double *position)
{
    ZH_LOGI("Encoder get position started.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_FAIL, NULL, "Encoder get position failed. Encoder not initialized.");
    *position = handle->encoder_position;
    ZH_LOGI("Encoder get position completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_reset(zh_encoder_handle_t *handle)
{
    ZH_LOGI("Encoder reset started.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_FAIL, NULL, "Encoder reset failed. Encoder not initialized.");
    handle->encoder_position = (handle->encoder_min_value + handle->encoder_max_value) / 2;
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

static esp_err_t _zh_encoder_validate_config(const zh_encoder_init_config_t *config)
{
    ZH_ERROR_CHECK(config != NULL, ESP_ERR_INVALID_ARG, NULL, "Invalid configuration.");
    ZH_ERROR_CHECK(config->task_priority >= 1 && config->stack_size >= configMINIMAL_STACK_SIZE, ESP_ERR_INVALID_ARG, NULL, "Invalid task settings.");
    ZH_ERROR_CHECK(config->queue_size >= 1, ESP_ERR_INVALID_ARG, NULL, "Invalid queue size.");
    ZH_ERROR_CHECK(config->encoder_max_value > config->encoder_min_value, ESP_ERR_INVALID_ARG, NULL, "Invalid encoder min/max value.");
    ZH_ERROR_CHECK(config->encoder_step > 0, ESP_ERR_INVALID_ARG, NULL, "Invalid encoder step.");
    return ESP_OK;
}

static esp_err_t _zh_encoder_gpio_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle)
{
    ZH_ERROR_CHECK(config->a_gpio_number < GPIO_NUM_MAX && config->b_gpio_number < GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, NULL, "Invalid GPIO number.")
    ZH_ERROR_CHECK(config->a_gpio_number != config->b_gpio_number, ESP_ERR_INVALID_ARG, NULL, "Both GPIO is same.")
    gpio_config_t pin_config = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << config->a_gpio_number) | (1ULL << config->b_gpio_number),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE};
    esp_err_t err = gpio_config(&pin_config);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "GPIO initialization failed.");
    if (_is_initialized == false)
    {
        err = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
        ZH_ERROR_CHECK(err == ESP_OK || err == ESP_ERR_INVALID_STATE, err, gpio_reset_pin((gpio_num_t)config->a_gpio_number); gpio_reset_pin((gpio_num_t)config->b_gpio_number), "Failed install isr service.");
        if (err == ESP_ERR_INVALID_STATE)
        {
            _is_prev_gpio_isr_handler = true;
        }
    }
    err = gpio_isr_handler_add(config->a_gpio_number, _zh_encoder_isr_handler, handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, gpio_reset_pin((gpio_num_t)config->a_gpio_number); gpio_reset_pin((gpio_num_t)config->b_gpio_number), "Interrupt initialization failed.");
    err = gpio_isr_handler_add(config->b_gpio_number, _zh_encoder_isr_handler, handle);
    if (_is_prev_gpio_isr_handler == true)
    {
        ZH_ERROR_CHECK(err == ESP_OK, err, gpio_isr_handler_remove((gpio_num_t)config->a_gpio_number); gpio_reset_pin((gpio_num_t)config->a_gpio_number); gpio_reset_pin((gpio_num_t)config->b_gpio_number), "Interrupt initialization failed.");
    }
    else
    {
        ZH_ERROR_CHECK(err == ESP_OK, err, gpio_isr_handler_remove((gpio_num_t)config->a_gpio_number); gpio_uninstall_isr_service(); gpio_reset_pin((gpio_num_t)config->a_gpio_number); gpio_reset_pin((gpio_num_t)config->b_gpio_number), "Interrupt initialization failed.");
    }
    return ESP_OK;
}

static esp_err_t _zh_encoder_resources_init(const zh_encoder_init_config_t *config)
{
    if (_is_initialized == false)
    {
        _queue_handle = xQueueCreate(config->queue_size, sizeof(zh_encoder_handle_t));
        ZH_ERROR_CHECK(_queue_handle != NULL, ESP_FAIL, NULL, "Failed to create queue.");
    }
    return ESP_OK;
}

static esp_err_t _zh_encoder_task_init(const zh_encoder_init_config_t *config)
{
    if (_is_initialized == false)
    {
        BaseType_t err = xTaskCreatePinnedToCore(&_zh_encoder_isr_processing_task, "zh_encoder_isr_processing", config->stack_size, NULL, config->task_priority, &zh_encoder, tskNO_AFFINITY);
        ZH_ERROR_CHECK(err == pdPASS, ESP_FAIL, NULL, "Failed to create isr processing task.");
    }
    return ESP_OK;
}

static void IRAM_ATTR _zh_encoder_isr_handler(void *arg)
{
    zh_encoder_handle_t *encoder_handle = (zh_encoder_handle_t *)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    encoder_handle->encoder_state = _encoder_matrix[encoder_handle->encoder_state & 0x0F][(gpio_get_level(encoder_handle->b_gpio_number) << 1) | gpio_get_level(encoder_handle->a_gpio_number)];
    switch (encoder_handle->encoder_state & 0x30)
    {
    case ZH_ENCODER_DIRECTION_CW:
        if (encoder_handle->encoder_position < encoder_handle->encoder_max_value)
        {
            encoder_handle->encoder_position = encoder_handle->encoder_position + encoder_handle->encoder_step;
            if (encoder_handle->encoder_position > encoder_handle->encoder_max_value)
            {
                encoder_handle->encoder_position = encoder_handle->encoder_max_value;
            }
            if (xQueueSendFromISR(_queue_handle, encoder_handle, &xHigherPriorityTaskWoken) != pdTRUE)
            {
                ++_stats.queue_overflow_error;
            }
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
            if (xQueueSendFromISR(_queue_handle, encoder_handle, &xHigherPriorityTaskWoken) != pdTRUE)
            {
                ++_stats.queue_overflow_error;
            }
        }
        break;
    default:
        break;
    }
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    };
}

static void IRAM_ATTR _zh_encoder_isr_processing_task(void *pvParameter)
{
    zh_encoder_handle_t queue = {0};
    zh_encoder_event_on_isr_t encoder_data = {0};
    while (xQueueReceive(_queue_handle, &queue, portMAX_DELAY) == pdTRUE)
    {
        encoder_data.encoder_number = queue.encoder_number;
        encoder_data.encoder_position = queue.encoder_position;
        esp_err_t err = esp_event_post(ZH_ENCODER, 0, &encoder_data, sizeof(zh_encoder_event_on_isr_t), 1000 / portTICK_PERIOD_MS);
        if (err != ESP_OK)
        {
            ++_stats.event_post_error;
            ZH_LOGE("Encoder isr processing failed. Failed to post interrupt event.", err);
        }
        _stats.min_stack_size = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    }
    vTaskDelete(NULL);
}