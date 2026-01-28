# ESP32 ESP-IDF component for rotary encoder

## Tested on

1. [ESP32 ESP-IDF v5.5.2](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32/index.html)

## SAST Tools

[PVS-Studio](https://pvs-studio.com/pvs-studio/?utm_source=website&utm_medium=github&utm_campaign=open_source) - static analyzer for C, C++, C#, and Java code.

## Features

1. Support some encoders on one device.

## Attention

1. For correct operation, please enable the following settings in the menuconfig:

```text
GPIO_CTRL_FUNC_IN_IRAM
PCNT_CTRL_FUNC_IN_IRAM
PCNT_ISR_IRAM_SAFE
```

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

One encoder with button on device:

```c
#include "zh_encoder.h"

#define ENCODER_NUMBER 0x01

zh_encoder_handle_t encoder_handle = {0};

void zh_encoder_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

void app_main(void)
{
    esp_log_level_set("zh_encoder", ESP_LOG_ERROR);
    esp_event_loop_create_default();
    esp_event_handler_instance_register(ZH_ENCODER, ESP_EVENT_ANY_ID, &zh_encoder_event_handler, NULL, NULL);
    zh_encoder_init_config_t config = ZH_ENCODER_INIT_CONFIG_DEFAULT();
    config.task_priority = 5;
    config.stack_size = configMINIMAL_STACK_SIZE;
    config.queue_size = 5;
    config.a_gpio_number = GPIO_NUM_4;
    config.b_gpio_number = GPIO_NUM_16;
    config.s_gpio_number = GPIO_NUM_15;
    config.encoder_min_value = -10;
    config.encoder_max_value = 10;
    config.encoder_step = 0.001;
    config.encoder_number = ENCODER_NUMBER;
    zh_encoder_init(&config, &encoder_handle);
    float encoder_position = 0;
    zh_encoder_get(&encoder_handle, &encoder_position);
    printf("Encoder position %0.3f.\n", encoder_position);
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
    switch (event_id)
    {
    case ZH_BUTTON_EVENT:
        zh_encoder_button_event_on_isr_t *button_event = event_data;
        printf("Encoder number %d button %s.\n", button_event->encoder_number, (button_event->button_status == 1) ? "released" : "pressed");
        break;
    case ZH_ENCODER_EVENT:
        zh_encoder_event_on_isr_t *encoder_event = event_data;
        printf("Encoder number %d position %0.3f.\n", encoder_event->encoder_number, encoder_event->encoder_position);
        break;
    default:
        break;
    }
}
```
