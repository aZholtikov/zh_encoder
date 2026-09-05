/**
 * @file zh_encoder.h
 *
 * @brief Rotary encoder driver for ESP-IDF using PCNT peripheral.
 *
 * Provides a FreeRTOS-based driver for rotary encoders with optional
 * push-button support. Uses ESP-IDF PCNT peripheral for quadrature
 * decoding and esp_event for asynchronous event notification.
 *
 * Key features:
 * - Quadrature decoding via PCNT peripheral
 * - Configurable min/max/step values with float precision
 * - Push-button support with debouncing
 * - Multiple simultaneous encoders (up to 8)
 * - Event-based notification (ISR context)
 * - Error statistics tracking
 */

#pragma once

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "zh_vector.h"

#define ZH_ENCODER_INIT_CONFIG_DEFAULT()        \
    {                                           \
        .task_priority = 1,                     \
        .stack_size = configMINIMAL_STACK_SIZE, \
        .queue_size = 1,                        \
        .a_gpio_number = GPIO_NUM_MAX,          \
        .b_gpio_number = GPIO_NUM_MAX,          \
        .s_gpio_number = GPIO_NUM_MAX,          \
        .s_gpio_debounce_time = 10,             \
        .pullup = true,                         \
        .encoder_min_value = -10,               \
        .encoder_max_value = 10,                \
        .encoder_step = 1,                      \
        .encoder_number = 0}

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Handle to the shared encoder ISR processing task.
     */
    extern TaskHandle_t zh_encoder;

    /**
     * @brief Opaque encoder handle.
     *
     * Opaque pointer to the internal encoder structure.
     * Used to reference a specific encoder instance across all API functions.
     */
    typedef struct _zh_encoder_handle_t zh_encoder_handle_t;

    /**
     * @brief Configuration structure for encoder initialization.
     *
     * Contains all parameters required to configure a rotary encoder,
     * including GPIO pins, value range, step size, and task settings.
     *
     * @note All GPIO numbers must be valid ESP-IDF GPIO numbers or GPIO_NUM_MAX (disabled).
     * @note encoder_number must be unique across all initialized encoders.
     */
    typedef struct
    {
        float encoder_step;            /*!< Encoder step. @note Must be greater than 0 */
        float encoder_min_value;       /*!< Encoder min value. @note Must be less than encoder_max_value */
        float encoder_max_value;       /*!< Encoder max value. @note Must be greater than encoder_min_value */
        gpio_num_t a_gpio_number;      /*!< Encoder A GPIO number */
        gpio_num_t b_gpio_number;      /*!< Encoder B GPIO number */
        gpio_num_t s_gpio_number;      /*!< Encoder button GPIO number */
        uint16_t s_gpio_debounce_time; /*!< Encoder button debounce_time. @note In microseconds */
        uint16_t stack_size;           /*!< Stack size for task for the encoder isr processing processing. @note The minimum size is configMINIMAL_STACK_SIZE */
        uint8_t task_priority;         /*!< Task priority for the encoder isr processing. @note Minimum value is 1 */
        uint8_t queue_size;            /*!< Queue size for task for the encoder processing. @note Minimum value is 1 */
        uint8_t encoder_number;        /*!< Unique encoder number. @note Must be greater than 0 */
        bool pullup;                   /*!< Pullup GPIO enable/disable */
    } zh_encoder_init_config_t;

    /**
     * @brief Structure for encoder error statistics.
     *
     * Tracks runtime errors and task health metrics for diagnostic purposes.
     */
    typedef struct
    {
        uint32_t event_post_error;     /*!< Number of event post error */
        uint32_t queue_overflow_error; /*!< Number of queue overflow error */
        uint32_t min_stack_size;       /*!< Minimum free stack size */
    } zh_encoder_stats_t;

    /**
     * @brief Encoder event identifiers for esp_event base.
     */
    typedef enum
    {
        ZH_BUTTON_EVENT, /*!< Encoder push-button interrupt event */
        ZH_ENCODER_EVENT /*!< Encoder rotation interrupt event */
    } zh_encoder_event_id_t;

    ESP_EVENT_DECLARE_BASE(ZH_ENCODER);

    /**
     * @brief Event data structure for encoder rotation events.
     *
     * Posted from ISR context via esp_event_isr_post when encoder position changes.
     *
     * @note Should be used with ZH_ENCODER event base and ZH_ENCODER_EVENT event ID.
     */
    typedef struct
    {
        float encoder_position; /*!< Encoder current position */
        uint8_t encoder_number; /*!< Encoder unique number */
    } zh_encoder_event_on_isr_t;

    /**
     * @brief Event data structure for encoder button events.
     *
     * Posted from ISR context via esp_event_isr_post when encoder button state changes.
     *
     * @note Should be used with ZH_ENCODER event base and ZH_BUTTON_EVENT event ID.
     */
    typedef struct
    {
        uint8_t encoder_number; /*!< Encoder unique number */
        bool button_status;     /*!< Encoder button status */
    } zh_encoder_button_event_on_isr_t;

    /**
     * @brief Initialize encoder with the provided configuration.
     *
     * Creates and configures a rotary encoder using the ESP-IDF PCNT peripheral.
     * The encoder is initialized at the midpoint of the configured range.
     *
     * @note The encoder will be set to the position (encoder_min_value + encoder_max_value)/2.
     * @note Before initialization, configure zh_encoder_init_config_t with default values.
     *
     * @param[in] config Pointer to encoder configuration structure. Can point to a temporary variable (must not be NULL)
     * @param[out] handle Pointer to store the unique encoder handle (must be NULL)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if config or handle is NULL, or parameters are invalid
     * @return ESP_ERR_INVALID_STATE if the encoder is already initialized
     * @return ESP_FAIL if resource allocation or peripheral initialization fails
     */
    esp_err_t zh_encoder_init(const zh_encoder_init_config_t *config, zh_encoder_handle_t **handle);

    /**
     * @brief Deinitialize encoder and release all resources.
     *
     * Stops the PCNT peripheral, removes GPIO ISR handlers, and frees allocated memory.
     * If this is the last encoder, the shared processing task and queue are also deleted.
     *
     * @param[in,out] handle Pointer to unique encoder handle (must not be NULL)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if handle is NULL or points to an uninitialized encoder
     * @return ESP_FAIL if PCNT or GPIO cleanup fails
     */
    esp_err_t zh_encoder_deinit(zh_encoder_handle_t **handle);

    /**
     * @brief Reinitialize encoder by updating min, max, and step values.
     *
     * Updates the encoder's value range and step size without reallocating resources.
     * The encoder position is reset to the midpoint of the new range.
     *
     * @note The encoder will be set to the position (encoder_min_value + encoder_max_value)/2.
     *
     * @param[in,out] handle Pointer to unique encoder handle (must not be NULL)
     * @param[in] min Encoder minimum value. Must be less than max
     * @param[in] max Encoder maximum value. Must be greater than min
     * @param[in] step Encoder step size. Must be greater than 0
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if handle is NULL, or parameters are invalid
     */
    esp_err_t zh_encoder_reinit(zh_encoder_handle_t **handle, float min, float max, float step);

    /**
     * @brief Set encoder position.
     *
     * @param[in,out] handle Pointer to unique encoder handle (must not be NULL)
     * @param[in] position Encoder position (must be between encoder_min_value and encoder_max_value)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if handle is NULL, or parameters are invalid
     */
    esp_err_t zh_encoder_set(zh_encoder_handle_t **handle, float position);

    /**
     * @brief Get encoder position.
     *
     * @param[in] handle Pointer to unique encoder handle (must not be NULL)
     * @param[out] position Pointer to store the current encoder position
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if handle or position is NULL
     */
    esp_err_t zh_encoder_get(zh_encoder_handle_t **handle, float *position);

    /**
     * @brief Reset encoder position to the midpoint of the configured range.
     *
     * @note The encoder will be set to the position (encoder_min_value + encoder_max_value)/2.
     *
     * @param[in,out] handle Pointer to unique encoder handle (must not be NULL)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if handle is NULL
     */
    esp_err_t zh_encoder_reset(zh_encoder_handle_t **handle);

    /**
     * @brief Get a pointer to the encoder error statistics.
     *
     * Returns a read-only pointer to the global statistics structure
     * containing error counts and task health metrics.
     *
     * @return Pointer to the statistics structure. Valid until zh_encoder_reset_stats() is called
     */
    const zh_encoder_stats_t *zh_encoder_get_stats(void);

    /**
     * @brief Reset all error statistics to zero.
     *
     * Clears error counters and stack watermarks.
     */
    void zh_encoder_reset_stats(void);

#ifdef __cplusplus
}
#endif