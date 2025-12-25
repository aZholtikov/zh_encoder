/**
 * @file zh_encoder.h
 */

#pragma once

#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"

/**
 * @brief Encoder initial default values.
 */
#define ZH_ENCODER_INIT_CONFIG_DEFAULT()        \
    {                                           \
        .task_priority = 1,                     \
        .stack_size = configMINIMAL_STACK_SIZE, \
        .queue_size = 1,                        \
        .a_gpio_number = GPIO_NUM_MAX,          \
        .b_gpio_number = GPIO_NUM_MAX,          \
        .s_gpio_number = GPIO_NUM_MAX,          \
        .encoder_min_value = -100,              \
        .encoder_max_value = 100,               \
        .encoder_step = 1,                      \
        .encoder_number = 0}

#ifdef __cplusplus
extern "C"
{
#endif

    extern TaskHandle_t zh_encoder; /*!< Unique encoder Task Handle. */

    /**
     * @brief Structure for initial initialization of encoder.
     */
    typedef struct
    {
        double encoder_step;       /*!< Encoder step. @note Must be greater than 0. */
        double encoder_min_value; /*!< Encoder min value. @note Must be less than encoder_max_value. */
        double encoder_max_value; /*!< Encoder max value. @note Must be greater than encoder_min_value. */
        uint8_t task_priority;     /*!< Task priority for the encoder isr processing. @note Minimum value is 1. */
        uint8_t queue_size;        /*!< Queue size for task for the encoder processing. @note Minimum value is 1. */
        uint8_t a_gpio_number;     /*!< Encoder A GPIO number. */
        uint8_t b_gpio_number;     /*!< Encoder B GPIO number. */
        uint8_t s_gpio_number;     /*!< Encoder button GPIO number. */
        uint8_t encoder_number;    /*!< Unique encoder number. */
        uint16_t stack_size;       /*!< Stack size for task for the encoder isr processing processing. @note The minimum size is configMINIMAL_STACK_SIZE. */
    } zh_encoder_init_config_t;

    /**
     * @brief Encoder handle.
     */
    typedef struct // -V802
    {
        double encoder_step;       /*!< Encoder step. */
        double encoder_position;   /*!< Encoder position. */
        double encoder_min_value; /*!< Encoder min value. */
        double encoder_max_value; /*!< Encoder max value. */
        uint8_t a_gpio_number;     /*!< Encoder A GPIO number. */
        uint8_t b_gpio_number;     /*!< Encoder B GPIO number. */
        uint8_t s_gpio_number;     /*!< Encoder button GPIO number. */
        uint8_t encoder_number;    /*!< Encoder unique number. */
        uint8_t encoder_state;     /*!< Encoder internal state. */
        bool button_status;        /*!< Encoder button status. */
        bool is_initialized;       /*!< Encoder initialization flag. */
    } zh_encoder_handle_t;

    /**
     * @brief Structure for error statistics storage.
     */
    typedef struct
    {
        uint32_t event_post_error;     /*!< Number of event post error. */
        uint32_t queue_overflow_error; /*!< Number of queue overflow error. */
        uint32_t min_stack_size;       /*!< Minimum free stack size. */
    } zh_encoder_stats_t;

    ESP_EVENT_DECLARE_BASE(ZH_ENCODER);

    /**
     * @brief Structure for sending data to the event handler when cause an interrupt.
     *
     * @note Should be used with ZH_ENCODER event base.
     */
    typedef struct
    {
        double encoder_position; /*!< Encoder current position. */
        uint8_t encoder_number;  /*!< Encoder unique number. */
        bool button_status;      /*!< Encoder button status. */
    } zh_encoder_event_on_isr_t;

    /**
     * @brief Initialize encoder.
     *
     * @note The encoder will be set to the position (encoder_min_value + encoder_max_value)/2.
     *
     * @param[in] config Pointer to encoder initialized configuration structure. Can point to a temporary variable.
     * @param[out] handle Pointer to unique encoder handle.
     *
     * @note Before initialize the encoder recommend initialize zh_encoder_init_config_t structure with default values.
     *
     * @code zh_encoder_init_config_t config = ZH_ENCODER_INIT_CONFIG_DEFAULT() @endcode
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_encoder_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t *handle);

    /**
     * @brief Deinitialize encoder.
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_encoder_deinit(zh_encoder_handle_t *handle);

    /**
     * @brief Set encoder position.
     *
     * @param[in, out] handle Pointer to unique encoder handle.
     * @param[in] position Encoder position (must be between encoder_min_value and encoder_max_value).
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_encoder_set(zh_encoder_handle_t *handle, double position);

    /**
     * @brief Get encoder position.
     *
     * @param[in] handle Pointer to unique encoder handle.
     * @param[out] position Encoder position.
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_encoder_get(const zh_encoder_handle_t *handle, double *position);

    /**
     * @brief Reset encoder position.
     *
     * @note The encoder will be set to the position (encoder_min_value + encoder_max_value)/2.
     *
     * @param[in, out] handle Pointer to unique encoder handle.
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_encoder_reset(zh_encoder_handle_t *handle);

    /**
     * @brief Get error statistics.
     *
     * @return Pointer to the statistics structure.
     */
    const zh_encoder_stats_t *zh_encoder_get_stats(void);

    /**
     * @brief Reset error statistics.
     */
    void zh_encoder_reset_stats(void);

#ifdef __cplusplus
}
#endif