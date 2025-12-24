# ESP32 ESP-IDF component for rotary encoder

## Tested on

1. [ESP32 ESP-IDF v5.5.1](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32/index.html)

## Features

1. Support some encoders on one device.

## Using

In an existing project, run the following command to install the components:

```text
cd ../your_project/components
git clone http://git.zh.com.ru/esp_components/zh_encoder
```

In the application, add the component:

```c
#include "zh_encoder.h"
```

## Examples

One encoder on device:

```c
#include "zh_encoder.h"

zh_encoder_handle_t encoder_handle = {0};

void zh_encoder_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

void app_main(void)
{
    esp_log_level_set("zh_encoder", ESP_LOG_ERROR);
    esp_event_loop_create_default();
    esp_event_handler_instance_register(ZH_ENCODER, ESP_EVENT_ANY_ID, &zh_encoder_event_handler, NULL, NULL);
    zh_encoder_init_config_t encoder_init_config = ZH_ENCODER_INIT_CONFIG_DEFAULT();
    encoder_init_config.a_gpio_number = GPIO_NUM_26;
    encoder_init_config.b_gpio_number = GPIO_NUM_27;
    encoder_init_config.encoder_min_value = -10;
    encoder_init_config.encoder_max_value = 20;
    encoder_init_config.encoder_step = 0.1;
    encoder_init_config.encoder_number = 1;
    zh_encoder_init(&encoder_init_config, &encoder_handle);
    double position = 0;
    zh_encoder_get(&encoder_handle, &position);
    printf("Encoder position %0.2f.\n", position);
    zh_encoder_set(&encoder_handle, 5);
    zh_encoder_reset(&encoder_handle);
    for (;;)
    {
        const zh_encoder_stats_t *stats = zh_encoder_get_stats();
        printf("Number of event post error: %ld.\n", stats->event_post_error);
        printf("Number of queue overflow error: %ld.\n", stats->queue_overflow_error);
        printf("Minimum free stack size: %ld.\n", stats->min_stack_size);
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}

void zh_encoder_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    zh_encoder_event_on_isr_t *event = event_data;
    printf("Encoder number %d position %0.2f.\n", event->encoder_number, event->encoder_position);
}
```
