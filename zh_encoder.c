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

#define ZH_ENCODER_DIRECTION_CW 1
#define ZH_ENCODER_DIRECTION_CCW -1

TaskHandle_t zh_encoder = NULL;
static QueueHandle_t _queue_handle = NULL;
static portMUX_TYPE _spinlock = portMUX_INITIALIZER_UNLOCKED;

static uint8_t _encoder_counter = 0;
static zh_encoder_stats_t _stats = {0};
static uint8_t _encoder_number_matrix[CONFIG_SOC_PCNT_UNITS_PER_GROUP] = {0};

static esp_err_t _zh_encoder_validate_config(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle);
static esp_err_t _zh_encoder_pcnt_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle);
static esp_err_t _zh_encoder_gpio_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle);
static esp_err_t _zh_encoder_resources_init(const zh_encoder_init_config_t *config);
static esp_err_t _zh_encoder_task_init(const zh_encoder_init_config_t *config);
static bool _zh_encoder_isr_handler(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
static void _zh_encoder_isr_processing_task(void *pvParameter);
static void _zh_encoder_button_isr_handler(void *arg);

ESP_EVENT_DEFINE_BASE(ZH_ENCODER);

esp_err_t zh_encoder_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle) // -V2008
{
    ZH_LOGI("Encoder initialization started.");
    ZH_ERROR_CHECK(config != NULL && handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder initialization failed. Invalid argument.");
    ZH_ERROR_CHECK(_encoder_counter < sizeof(_encoder_number_matrix), ESP_ERR_INVALID_ARG, NULL, "Encoder initialization failed. Maximum quantity reached.");
    esp_err_t err = _zh_encoder_validate_config(config, handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "Encoder initialization failed. Initial configuration check failed.");
    err = _zh_encoder_resources_init(config);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "Encoder initialization failed. Resources initialization failed.");
    err = _zh_encoder_task_init(config);
    ZH_ERROR_CHECK(err == ESP_OK, err, vQueueDelete(_queue_handle); _queue_handle = NULL, "Encoder initialization failed. Processing task initialization failed.");
    err = _zh_encoder_pcnt_init(config, handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, vQueueDelete(_queue_handle); _queue_handle = NULL; vTaskDelete(zh_encoder); zh_encoder = NULL,
                                                                                                                   "Encoder initialization failed. PCNT initialization failed.");
    err = _zh_encoder_gpio_init(config, handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_unit_stop(handle->pcnt_unit_handle); pcnt_unit_disable(handle->pcnt_unit_handle); pcnt_del_channel(handle->pcnt_channel_a_handle);
                   pcnt_del_channel(handle->pcnt_channel_b_handle); pcnt_del_unit(handle->pcnt_unit_handle); vQueueDelete(_queue_handle); _queue_handle = NULL;
                   vTaskDelete(zh_encoder); zh_encoder = NULL, "Encoder initialization failed. GPIO initialization failed.");
    handle->is_initialized = true;
    ++_encoder_counter;
    for (uint8_t i = 0; i < sizeof(_encoder_number_matrix); ++i)
    {
        if (_encoder_number_matrix[i] == 0)
        {
            _encoder_number_matrix[i] = handle->encoder_number;
            break;
        }
    }
    ZH_LOGI("Encoder initialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_deinit(zh_encoder_handle_t *handle)
{
    ZH_LOGI("Encoder deinitialization started.");
    ZH_ERROR_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder deinitialization failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_FAIL, NULL, "Encoder deinitialization failed. Encoder not initialized.");
    pcnt_unit_stop(handle->pcnt_unit_handle);
    pcnt_unit_disable(handle->pcnt_unit_handle);
    pcnt_del_channel(handle->pcnt_channel_a_handle);
    pcnt_del_channel(handle->pcnt_channel_b_handle);
    pcnt_del_unit(handle->pcnt_unit_handle);
    if (handle->s_gpio_number != GPIO_NUM_MAX)
    {
        gpio_isr_handler_remove((gpio_num_t)handle->s_gpio_number);
        gpio_reset_pin((gpio_num_t)handle->s_gpio_number);
    }
    if (_encoder_counter == 1)
    {
        vQueueDelete(_queue_handle);
        _queue_handle = NULL;
        vTaskDelete(zh_encoder);
        zh_encoder = NULL;
    }
    handle->is_initialized = false;
    --_encoder_counter;
    for (uint8_t i = 0; i < sizeof(_encoder_number_matrix); ++i)
    {
        if (_encoder_number_matrix[i] == handle->encoder_number)
        {
            _encoder_number_matrix[i] = 0;
            break;
        }
    }
    ZH_LOGI("Encoder deinitialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_reinit(zh_encoder_handle_t *handle, double min, double max, double step) // -V2008
{
    ZH_LOGI("Encoder reinitialization started.");
    ZH_ERROR_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder reinitialization failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_FAIL, NULL, "Encoder reinitialization failed. Encoder not initialized.");
    ZH_ERROR_CHECK(max > min, ESP_ERR_INVALID_ARG, NULL, "Encoder reinitialization failed. Invalid encoder min/max value.");
    ZH_ERROR_CHECK(step > 0, ESP_ERR_INVALID_ARG, NULL, "Encoder reinitialization failed. Invalid encoder step.");
    taskENTER_CRITICAL(&_spinlock);
    handle->encoder_min_value = min;
    handle->encoder_max_value = max;
    handle->encoder_step = step;
    handle->encoder_position = (handle->encoder_min_value + handle->encoder_max_value) / 2;
    taskEXIT_CRITICAL(&_spinlock);
    ZH_LOGI("Encoder reinitialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_set(zh_encoder_handle_t *handle, double position)
{
    ZH_LOGI("Encoder set position started.");
    ZH_ERROR_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder set position failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_FAIL, NULL, "Encoder set position failed. Encoder not initialized.");
    ZH_ERROR_CHECK(position <= handle->encoder_max_value && position >= handle->encoder_min_value, ESP_ERR_INVALID_ARG, NULL, "Encoder set position failed. Invalid argument.");
    taskENTER_CRITICAL(&_spinlock);
    handle->encoder_position = position;
    taskEXIT_CRITICAL(&_spinlock);
    ZH_LOGI("Encoder set position completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_get(const zh_encoder_handle_t *handle, double *position)
{
    ZH_LOGI("Encoder get position started.");
    ZH_ERROR_CHECK(handle != NULL && position != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder get position failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_FAIL, NULL, "Encoder get position failed. Encoder not initialized.");
    *position = handle->encoder_position;
    ZH_LOGI("Encoder get position completed successfully.");
    return ESP_OK;
}

esp_err_t zh_encoder_reset(zh_encoder_handle_t *handle)
{
    ZH_LOGI("Encoder reset started.");
    ZH_ERROR_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Encoder reset failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_FAIL, NULL, "Encoder reset failed. Encoder not initialized.");
    taskENTER_CRITICAL(&_spinlock);
    handle->encoder_position = (handle->encoder_min_value + handle->encoder_max_value) / 2;
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
    ZH_ERROR_CHECK(config != NULL && handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Invalid argument.");
    ZH_ERROR_CHECK(config->task_priority >= 1 && config->stack_size >= configMINIMAL_STACK_SIZE, ESP_ERR_INVALID_ARG, NULL, "Invalid task settings.");
    ZH_ERROR_CHECK(config->queue_size >= 1, ESP_ERR_INVALID_ARG, NULL, "Invalid queue size.");
    ZH_ERROR_CHECK(config->encoder_max_value > config->encoder_min_value, ESP_ERR_INVALID_ARG, NULL, "Invalid encoder min/max value.");
    ZH_ERROR_CHECK(config->encoder_step > 0, ESP_ERR_INVALID_ARG, NULL, "Invalid encoder step.");
    ZH_ERROR_CHECK(config->encoder_number > 0, ESP_ERR_INVALID_ARG, NULL, "Invalid encoder number.");
    for (uint8_t i = 0; i < sizeof(_encoder_number_matrix); ++i)
    {
        ZH_ERROR_CHECK(config->encoder_number != _encoder_number_matrix[i], ESP_ERR_INVALID_ARG, NULL, "Encoder number already present.");
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
    ZH_ERROR_CHECK(config != NULL && handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Invalid argument.");
    ZH_ERROR_CHECK(config->a_gpio_number < GPIO_NUM_MAX && config->b_gpio_number < GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, NULL, "Invalid GPIO number.")
    ZH_ERROR_CHECK(config->a_gpio_number != config->b_gpio_number, ESP_ERR_INVALID_ARG, NULL, "Encoder A and B GPIO is same.")
    pcnt_unit_config_t pcnt_unit_config = {
        .high_limit = 10,
        .low_limit = -10,
    };
    pcnt_unit_handle_t pcnt_unit_handle = NULL;
    esp_err_t err = pcnt_new_unit(&pcnt_unit_config, &pcnt_unit_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "PCNT initialization failed.");
    pcnt_glitch_filter_config_t pcnt_glitch_filter_config = {
        .max_glitch_ns = 1000,
    };
    err = pcnt_unit_set_glitch_filter(pcnt_unit_handle, &pcnt_glitch_filter_config);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    pcnt_chan_config_t pcnt_chan_a_config = {
        .edge_gpio_num = config->a_gpio_number,
        .level_gpio_num = config->b_gpio_number,
    };
    pcnt_channel_handle_t pcnt_channel_a_handle = NULL;
    err = pcnt_new_channel(pcnt_unit_handle, &pcnt_chan_a_config, &pcnt_channel_a_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    pcnt_chan_config_t pcnt_chan_b_config = {
        .edge_gpio_num = config->b_gpio_number,
        .level_gpio_num = config->a_gpio_number,
    };
    pcnt_channel_handle_t pcnt_channel_b_handle = NULL;
    err = pcnt_new_channel(pcnt_unit_handle, &pcnt_chan_b_config, &pcnt_channel_b_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_channel_set_edge_action(pcnt_channel_a_handle, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_channel_set_level_action(pcnt_channel_a_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_HOLD);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_channel_set_edge_action(pcnt_channel_b_handle, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_channel_set_level_action(pcnt_channel_b_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_HOLD);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_unit_add_watch_point(pcnt_unit_handle, ZH_ENCODER_DIRECTION_CW);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_unit_add_watch_point(pcnt_unit_handle, ZH_ENCODER_DIRECTION_CCW);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    pcnt_event_callbacks_t cbs = {
        .on_reach = _zh_encoder_isr_handler,
    };
    err = pcnt_unit_register_event_callbacks(pcnt_unit_handle, &cbs, handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle),
                                                                                                                         "PCNT initialization failed.");
    err = pcnt_unit_enable(pcnt_unit_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle),
                                                                                                                         "PCNT initialization failed.");
    err = pcnt_unit_clear_count(pcnt_unit_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_unit_disable(pcnt_unit_handle); pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle);
                   pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_unit_start(pcnt_unit_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_unit_disable(pcnt_unit_handle); pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle);
                   pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    if (config->pullup == false)
    {
        gpio_pullup_dis((gpio_num_t)config->a_gpio_number);
        gpio_pullup_dis((gpio_num_t)config->b_gpio_number);
    }
    handle->pcnt_unit_handle = pcnt_unit_handle;
    handle->pcnt_channel_a_handle = pcnt_channel_a_handle;
    handle->pcnt_channel_b_handle = pcnt_channel_b_handle;
    return ESP_OK;
}

static esp_err_t _zh_encoder_gpio_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle) // -V2008
{
    ZH_ERROR_CHECK(config != NULL && handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Invalid argument.");
    ZH_ERROR_CHECK(config->s_gpio_number <= GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, NULL, "Invalid GPIO number.")
    ZH_ERROR_CHECK(config->a_gpio_number != config->s_gpio_number && config->b_gpio_number != config->s_gpio_number, ESP_ERR_INVALID_ARG, NULL, "Encoder GPIO and button GPIO is same.")
    if (config->s_gpio_number != GPIO_NUM_MAX)
    {
        gpio_config_t pin_config = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = (config->pullup == true) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pin_bit_mask = (1ULL << config->s_gpio_number),
            .intr_type = GPIO_INTR_ANYEDGE};
        esp_err_t err = gpio_config(&pin_config);
        ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "GPIO initialization failed.");
        err = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
        ZH_ERROR_CHECK(err == ESP_OK || err == ESP_ERR_INVALID_STATE, err, gpio_reset_pin((gpio_num_t)config->s_gpio_number), "Failed install isr service.");
        err = gpio_isr_handler_add((gpio_num_t)config->s_gpio_number, _zh_encoder_button_isr_handler, handle);
        ZH_ERROR_CHECK(err == ESP_OK, err, gpio_reset_pin((gpio_num_t)config->s_gpio_number), "Interrupt initialization failed.");
        handle->s_gpio_number = config->s_gpio_number;
        handle->s_gpio_debounce_time = config->s_gpio_debounce_time;
        handle->s_gpio_status = gpio_get_level((gpio_num_t)config->s_gpio_number);
    }
    return ESP_OK;
}

static esp_err_t _zh_encoder_resources_init(const zh_encoder_init_config_t *config)
{
    if (_encoder_counter == 0)
    {
        _queue_handle = xQueueCreate(config->queue_size, sizeof(zh_encoder_event_on_isr_t));
        ZH_ERROR_CHECK(_queue_handle != NULL, ESP_FAIL, NULL, "Failed to create queue.");
    }
    return ESP_OK;
}

static esp_err_t _zh_encoder_task_init(const zh_encoder_init_config_t *config)
{
    if (_encoder_counter == 0)
    {
        BaseType_t err = xTaskCreatePinnedToCore(&_zh_encoder_isr_processing_task, "zh_encoder_isr_processing", config->stack_size, NULL, config->task_priority,
                                                 &zh_encoder, tskNO_AFFINITY);
        ZH_ERROR_CHECK(err == pdPASS, ESP_FAIL, NULL, "Failed to create isr processing task.");
    }
    return ESP_OK;
}

static bool IRAM_ATTR _zh_encoder_isr_handler(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    zh_encoder_handle_t *encoder_handle = (zh_encoder_handle_t *)user_ctx;
    pcnt_unit_clear_count(unit);
    switch (edata->watch_point_value)
    {
    case ZH_ENCODER_DIRECTION_CW:
        if (encoder_handle->encoder_position < encoder_handle->encoder_max_value)
        {
            encoder_handle->encoder_position = encoder_handle->encoder_position + encoder_handle->encoder_step;
            if (encoder_handle->encoder_position > encoder_handle->encoder_max_value)
            {
                encoder_handle->encoder_position = encoder_handle->encoder_max_value;
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
        }
        break;
    default:
        return false;
        break;
    }
    zh_encoder_event_on_isr_t encoder_data = {0};
    encoder_data.encoder_number = encoder_handle->encoder_number;
    encoder_data.encoder_position = encoder_handle->encoder_position;
    if (xQueueSendFromISR(_queue_handle, &encoder_data, &xHigherPriorityTaskWoken) != pdTRUE)
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
    zh_encoder_event_on_isr_t encoder_data = {0};
    while (xQueueReceive(_queue_handle, &encoder_data, portMAX_DELAY) == pdTRUE)
    {
        esp_err_t err = esp_event_post(ZH_ENCODER, ZH_ENCODER_EVENT, &encoder_data, sizeof(zh_encoder_event_on_isr_t), 1000 / portTICK_PERIOD_MS);
        if (err != ESP_OK)
        {
            ++_stats.event_post_error;
            ZH_LOGE("Encoder isr processing failed. Failed to post interrupt event.", err);
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
        bool s_gpio_status = gpio_get_level((gpio_num_t)encoder_handle->s_gpio_number);
        if (encoder_handle->s_gpio_status != s_gpio_status)
        {
            encoder_handle->s_gpio_status = s_gpio_status;
            zh_encoder_button_event_on_isr_t encoder_data = {0};
            encoder_data.encoder_number = encoder_handle->encoder_number;
            encoder_data.button_status = encoder_handle->s_gpio_status;
            esp_err_t err = esp_event_isr_post(ZH_ENCODER, ZH_BUTTON_EVENT, &encoder_data, sizeof(zh_encoder_button_event_on_isr_t), &xHigherPriorityTaskWoken);
            if (err != ESP_OK)
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