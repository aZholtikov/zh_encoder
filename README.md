# ESP32 ESP-IDF component for rotary encoder (PCNT peripheral)

## Features

1. Quadrature decoding through the PCNT peripheral hardware.
2. Configurable value range and step with float precision.
3. Push-button support with software debouncing timer.
4. Multiple encoders sharing a single processing task (limited by PCNT units and GPIOs).
5. Asynchronous rotation and button notifications through esp_event.
6. Built-in error statistics with queue overflow and stack watermark counters.

## Attention

1. Encoder number must be unique across all initialized encoders and greater than zero.
2. Task priority, stack size and queue size of the first initialized encoder are used by the shared processing task.
3. Position is initialized to the midpoint of the configured range.
4. Position is clamped to the configured minimum and maximum values.

## Using

In an existing project, run the following command to install the component:

```bash
cd ../your_project/components
git clone https://github.com/aZholtikov/zh_encoder
```

In the application, add the component:

```c
#include "zh_encoder.h"
```

## Examples

### Basic Example: Init and Get Position

```c
#include "zh_encoder.h"

zh_encoder_handle_t *encoder_handle = NULL;

void app_main(void)
{
    esp_log_level_set("zh_encoder", ESP_LOG_ERROR);
    zh_encoder_init_config_t config = ZH_ENCODER_INIT_CONFIG_DEFAULT();
    config.a_gpio_number = GPIO_NUM_4;
    config.b_gpio_number = GPIO_NUM_16;
    config.encoder_min_value = -10;
    config.encoder_max_value = 10;
    config.encoder_step = 1;
    config.encoder_number = 1;
    esp_err_t ret = zh_encoder_init(&config, &encoder_handle);
    if (ret != ESP_OK)
    {
        printf("Encoder initialization error: %s\n", esp_err_to_name(ret));
        return;
    }
    float position = 0;
    ret = zh_encoder_get(&encoder_handle, &position);
    if (ret == ESP_OK)
    {
        printf("Encoder position: %0.3f\n", position);
    }
    else
    {
        printf("Error getting position: %s\n", esp_err_to_name(ret));
    }
    ret = zh_encoder_deinit(&encoder_handle);
    if (ret != ESP_OK)
    {
        printf("Error deinitializing encoder: %s\n", esp_err_to_name(ret));
    }
}
```

### Set and Reset Position Example

```c
#include "zh_encoder.h"

zh_encoder_handle_t *encoder_handle = NULL;

void app_main(void)
{
    esp_log_level_set("zh_encoder", ESP_LOG_ERROR);
    zh_encoder_init_config_t config = ZH_ENCODER_INIT_CONFIG_DEFAULT();
    config.a_gpio_number = GPIO_NUM_4;
    config.b_gpio_number = GPIO_NUM_16;
    config.encoder_min_value = -100;
    config.encoder_max_value = 100;
    config.encoder_step = 0.5;
    config.encoder_number = 1;
    esp_err_t ret = zh_encoder_init(&config, &encoder_handle);
    if (ret != ESP_OK)
    {
        printf("Encoder initialization error: %s\n", esp_err_to_name(ret));
        return;
    }
    float position = 25.0f;
    ret = zh_encoder_set(&encoder_handle, position);
    if (ret == ESP_OK)
    {
        printf("Position set to %0.3f\n", position);
    }
    else
    {
        printf("Error setting position: %s\n", esp_err_to_name(ret));
    }
    float current = 0;
    ret = zh_encoder_get(&encoder_handle, &current);
    if (ret == ESP_OK)
    {
        printf("Current position: %0.3f\n", current);
    }
    else
    {
        printf("Error getting position: %s\n", esp_err_to_name(ret));
    }
    ret = zh_encoder_reset(&encoder_handle);
    if (ret == ESP_OK)
    {
        printf("Position reset to midpoint\n");
    }
    else
    {
        printf("Error resetting position: %s\n", esp_err_to_name(ret));
    }
    zh_encoder_deinit(&encoder_handle);
}
```

### Reinit Range Example

```c
#include "zh_encoder.h"

zh_encoder_handle_t *encoder_handle = NULL;

void app_main(void)
{
    esp_log_level_set("zh_encoder", ESP_LOG_ERROR);
    zh_encoder_init_config_t config = ZH_ENCODER_INIT_CONFIG_DEFAULT();
    config.a_gpio_number = GPIO_NUM_4;
    config.b_gpio_number = GPIO_NUM_16;
    config.encoder_min_value = -10;
    config.encoder_max_value = 10;
    config.encoder_step = 1;
    config.encoder_number = 1;
    esp_err_t ret = zh_encoder_init(&config, &encoder_handle);
    if (ret != ESP_OK)
    {
        printf("Encoder initialization error: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = zh_encoder_reinit(&encoder_handle, 0, 255, 1);
    if (ret == ESP_OK)
    {
        printf("Encoder range changed to 0..255\n");
    }
    else
    {
        printf("Error reinitializing encoder: %s\n", esp_err_to_name(ret));
    }
    float position = 0;
    ret = zh_encoder_get(&encoder_handle, &position);
    if (ret == ESP_OK)
    {
        printf("Position after reinit: %0.3f\n", position);
    }
    else
    {
        printf("Error getting position: %s\n", esp_err_to_name(ret));
    }
    zh_encoder_deinit(&encoder_handle);
}
```

---

### Button Event Example

```c
#include "zh_encoder.h"

zh_encoder_handle_t *encoder_handle = NULL;

void zh_encoder_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    if (event_id == ZH_BUTTON_EVENT)
    {
        zh_encoder_button_event_on_isr_t *button_event = (zh_encoder_button_event_on_isr_t *)event_data;
        printf("Encoder %d button %s\n", button_event->encoder_number, button_event->button_status ? "released" : "pressed");
    }
    else if (event_id == ZH_ENCODER_EVENT)
    {
        zh_encoder_event_on_isr_t *encoder_event = (zh_encoder_event_on_isr_t *)event_data;
        printf("Encoder %d position %0.3f %s\n", encoder_event->encoder_number, encoder_event->encoder_position, encoder_event->encoder_status ? "increased" : "reduced");
    }
}

void app_main(void)
{
    esp_log_level_set("zh_encoder", ESP_LOG_ERROR);
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_instance_register(ZH_ENCODER, ESP_EVENT_ANY_ID, &zh_encoder_event_handler, NULL, NULL));
    zh_encoder_init_config_t config = ZH_ENCODER_INIT_CONFIG_DEFAULT();
    config.a_gpio_number = GPIO_NUM_4;
    config.b_gpio_number = GPIO_NUM_16;
    config.s_gpio_number = GPIO_NUM_15;
    config.s_gpio_debounce_time = 50000;
    config.encoder_min_value = 0;
    config.encoder_max_value = 100;
    config.encoder_step = 1;
    config.encoder_number = 1;
    esp_err_t ret = zh_encoder_init(&config, &encoder_handle);
    if (ret != ESP_OK)
    {
        printf("Encoder initialization error: %s\n", esp_err_to_name(ret));
        return;
    }
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---

### Error Statistics Example

```c
#include "zh_encoder.h"

zh_encoder_handle_t *encoder_handle = NULL;

void app_main(void)
{
    esp_log_level_set("zh_encoder", ESP_LOG_ERROR);
    zh_encoder_init_config_t config = ZH_ENCODER_INIT_CONFIG_DEFAULT();
    config.a_gpio_number = GPIO_NUM_4;
    config.b_gpio_number = GPIO_NUM_16;
    config.encoder_min_value = -10;
    config.encoder_max_value = 10;
    config.encoder_step = 1;
    config.encoder_number = 1;
    esp_err_t ret = zh_encoder_init(&config, &encoder_handle);
    if (ret != ESP_OK)
    {
        printf("Encoder initialization error: %s\n", esp_err_to_name(ret));
        return;
    }
    for (;;)
    {
        const zh_encoder_stats_t *stats = zh_encoder_get_stats();
        printf("Event post errors: %lu\n", (unsigned long)stats->event_post_error);
        printf("Queue overflow errors: %lu\n", (unsigned long)stats->queue_overflow_error);
        printf("Minimum free stack: %lu\n", (unsigned long)stats->min_stack_size);
        zh_encoder_reset_stats();
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
```
