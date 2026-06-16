#ifndef ESP32_EMBEDDED_COMMON_LIB_DC_MOTOR_H
#define ESP32_EMBEDDED_COMMON_LIB_DC_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── TT DC motor specs (3–6 V, no encoder) ──────────────────────────────── */
/** 1:48 gearbox reduction ratio. */
#define ESP32_COMMON_TT_MOTOR_GEAR_RATIO   48U
/** Nominal no-load output shaft RPM at 6 V. */
#define ESP32_COMMON_TT_MOTOR_RPM_AT_6V   200U

/* ── Driver defaults ─────────────────────────────────────────────────────── */
/** Default PWM carrier frequency — above human hearing, safe for most motors. */
#define ESP32_COMMON_DC_MOTOR_PWM_FREQ_HZ    10000U
/** 10-bit resolution → 1024 duty levels. */
#define ESP32_COMMON_DC_MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT

/**
 * @brief H-bridge wiring and LEDC configuration for one DC motor.
 *
 * Typical logic table (IN1, IN2, PWM):
 *   1  0  duty  → forward   at (duty / max) × 100 %
 *   0  1  duty  → reverse   at (duty / max) × 100 %
 *   0  0   0    → coast     (free-wheel)
 *   1  1   0    → brake     (short-circuit across motor)
 *
 * Compatible bridges: L298N, DRV8833, TB6612, L9110S, etc.
 */
typedef struct {
    gpio_num_t       pin_in1;        /**< H-bridge direction pin 1.            */
    gpio_num_t       pin_in2;        /**< H-bridge direction pin 2.            */
    gpio_num_t       pin_pwm;        /**< H-bridge enable / speed (PWM) pin.   */
    ledc_timer_t     ledc_timer;     /**< LEDC hardware timer (0–3).           */
    ledc_channel_t   ledc_channel;   /**< LEDC hardware channel (0–7).         */
    uint32_t         pwm_freq_hz;    /**< PWM frequency in Hz.                 */
    ledc_timer_bit_t pwm_resolution; /**< Duty-cycle bit-width.                */
    bool             brake_on_stop;  /**< true = active brake; false = coast.  */
} esp32_common_dc_motor_config_t;

/** Runtime state for one DC motor instance. */
typedef struct {
    esp32_common_dc_motor_config_t config;
    bool   initialized;
    int8_t speed_pct; /**< Last commanded speed (−100 … +100 %). */
} esp32_common_dc_motor_t;

/**
 * @brief Build a default configuration.
 *
 * Selects LEDC_TIMER_0 / LEDC_CHANNEL_0, 10 kHz, 10-bit, coast on stop.
 * Override any field before calling esp32_common_dc_motor_init().
 *
 * @param pin_in1  H-bridge IN1 GPIO.
 * @param pin_in2  H-bridge IN2 GPIO.
 * @param pin_pwm  H-bridge PWM/EN GPIO.
 */
esp32_common_dc_motor_config_t esp32_common_dc_motor_default_config(
    gpio_num_t pin_in1,
    gpio_num_t pin_in2,
    gpio_num_t pin_pwm
);

/**
 * @brief Initialise a DC motor (configure GPIOs + LEDC timer/channel).
 *
 * @param motor   Pointer to an uninitialised motor instance.
 * @param config  Hardware configuration (copied into the instance).
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if any argument is invalid.
 */
esp_err_t esp32_common_dc_motor_init(
    esp32_common_dc_motor_t              *motor,
    const esp32_common_dc_motor_config_t *config
);

/**
 * @brief Set motor speed and direction.
 *
 * @param motor     Initialised motor instance.
 * @param speed_pct Speed in the range [−100, +100] percent.
 *                  Positive = forward, negative = reverse, 0 = stop.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialised.
 */
esp_err_t esp32_common_dc_motor_set_speed(
    esp32_common_dc_motor_t *motor,
    int8_t speed_pct
);

/**
 * @brief Stop the motor (coast or active brake depending on config.brake_on_stop).
 */
esp_err_t esp32_common_dc_motor_stop(esp32_common_dc_motor_t *motor);

/**
 * @brief Release LEDC resources and reset the instance.
 */
esp_err_t esp32_common_dc_motor_deinit(esp32_common_dc_motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_EMBEDDED_COMMON_LIB_DC_MOTOR_H */
