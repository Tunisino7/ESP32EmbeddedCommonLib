#ifndef ECL_DC_MOTOR_ENCODER_H
#define ECL_DC_MOTOR_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "ESP32EmbeddedCommonLib/motor/dc_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── N20 motor specs ─────────────────────────────────────────────────────── */
/** Magnetic Hall-effect pulses per motor shaft revolution (pre-gearbox). */
#define ECL_N20_MOTOR_PPR       7U
/** Nominal supply voltage (V). */
#define ECL_N20_MOTOR_VOLTAGE_V 6U

/* ── PCNT hardware counter limits ───────────────────────────────────────── */
#define ECL_DC_MOTOR_ENCODER_PCNT_HIGH  32767
#define ECL_DC_MOTOR_ENCODER_PCNT_LOW  (-32768)

/**
 * @brief Configuration for an N20 (or similar) DC motor with a quadrature
 *        Hall-effect encoder.
 *
 * The motor is driven via an H-bridge (reuses dc_motor config).
 * The encoder uses PCNT quadrature decoding:
 *   - pin_enc_b provided → 4× mode (full quadrature, highest resolution).
 *   - pin_enc_b = GPIO_NUM_NC → 2× mode (both edges on channel A only).
 */
typedef struct {
    ecl_dc_motor_config_t motor;     /**< Underlying H-bridge / LEDC config. */
    gpio_num_t pin_enc_a;                     /**< Encoder channel A.                 */
    gpio_num_t pin_enc_b;                     /**< Encoder channel B (GPIO_NUM_NC = 2× mode). */
    uint16_t   pulses_per_rev;                /**< Hall PPR before gearbox (7 for N20). */
    uint16_t   gear_ratio;                    /**< Gearbox reduction (e.g. 50, 100). */
} ecl_dc_motor_encoder_config_t;

/**
 * @brief Runtime state for a DC motor with quadrature encoder.
 *
 * Pulse accumulation is overflow-safe: an ISR callback adds the PCNT limit
 * value to accum_pulses whenever the hardware counter wraps, and a spinlock
 * makes the combined read (accum + hardware count) atomic with respect to
 * that ISR.
 */
typedef struct {
    ecl_dc_motor_encoder_config_t config;
    ecl_dc_motor_t                motor;            /**< Underlying motor driver.       */
    bool                                   initialized;
    pcnt_unit_handle_t                     pcnt_unit;
    pcnt_channel_handle_t                  pcnt_chan_a;
    pcnt_channel_handle_t                  pcnt_chan_b;       /**< NULL in 2× mode.               */
    portMUX_TYPE                           spinlock;          /**< Guards accum_pulses vs ISR.    */
    volatile int64_t                       accum_pulses;      /**< Overflow accumulator.          */
    uint16_t                               counts_per_motor_rev; /**< PPR × decode multiplier.   */
    int64_t                                rpm_ref_pulses;    /**< Pulse snapshot for RPM delta.  */
    int64_t                                rpm_ref_time_us;   /**< Timestamp for RPM delta (µs).  */
    float                                  rpm;               /**< Last computed output-shaft RPM. */
} ecl_dc_motor_encoder_t;

/**
 * @brief Build a default N20 motor configuration.
 *
 * Sets pulses_per_rev = 7, gear_ratio = 1.
 * Override gear_ratio to match your physical variant (15, 25, 50, 100 …).
 * Override motor.ledc_timer / motor.ledc_channel when driving multiple motors.
 *
 * @param pin_in1   H-bridge IN1.
 * @param pin_in2   H-bridge IN2.
 * @param pin_pwm   H-bridge PWM/EN.
 * @param pin_enc_a Encoder channel A.
 * @param pin_enc_b Encoder channel B (GPIO_NUM_NC for 2× mode).
 */
ecl_dc_motor_encoder_config_t ecl_dc_motor_encoder_default_config(
    gpio_num_t pin_in1,
    gpio_num_t pin_in2,
    gpio_num_t pin_pwm,
    gpio_num_t pin_enc_a,
    gpio_num_t pin_enc_b
);

/**
 * @brief Initialise the motor and encoder.
 *
 * Configures LEDC PWM (via dc_motor), PCNT quadrature decoder, overflow
 * watch-points, and the ISR accumulator callback.
 *
 * @param motor   Pointer to an uninitialised instance.
 * @param config  Hardware configuration (copied into the instance).
 * @return ESP_OK on success.
 */
esp_err_t ecl_dc_motor_encoder_init(
    ecl_dc_motor_encoder_t              *motor,
    const ecl_dc_motor_encoder_config_t *config
);

/** @brief Set motor speed [−100, +100] %. */
esp_err_t ecl_dc_motor_encoder_set_speed(
    ecl_dc_motor_encoder_t *motor,
    int8_t speed_pct
);

/** @brief Stop the motor (coast or brake per config). */
esp_err_t ecl_dc_motor_encoder_stop(
    ecl_dc_motor_encoder_t *motor
);

/**
 * @brief Read the accumulated encoder pulse count since init or last reset.
 *
 * Sign convention: positive = net forward rotation; negative = net reverse.
 * The count is quadrature-decoded (4× if enc_b is wired, 2× otherwise).
 *
 * @param motor   Initialised instance.
 * @param pulses  Output: total pulse count (int64_t, overflow-safe).
 */
esp_err_t ecl_dc_motor_encoder_get_pulses(
    ecl_dc_motor_encoder_t *motor,
    int64_t *pulses
);

/**
 * @brief Compute instantaneous output-shaft RPM.
 *
 * Call periodically from a single task (recommended interval: 50–200 ms).
 * The first call seeds the reference and returns 0.0 RPM.
 *
 * @param motor  Initialised instance.
 * @param rpm    Output: RPM at the gearbox output shaft (positive = forward).
 */
esp_err_t ecl_dc_motor_encoder_get_rpm(
    ecl_dc_motor_encoder_t *motor,
    float *rpm
);

/**
 * @brief Reset the pulse accumulator and PCNT hardware counter to zero.
 *
 * Also resets the RPM reference so the next get_rpm() call re-seeds.
 */
esp_err_t ecl_dc_motor_encoder_reset_count(
    ecl_dc_motor_encoder_t *motor
);

/**
 * @brief Stop the motor and release all LEDC + PCNT hardware resources.
 */
esp_err_t ecl_dc_motor_encoder_deinit(
    ecl_dc_motor_encoder_t *motor
);

#ifdef __cplusplus
}
#endif

#endif /* ECL_DC_MOTOR_ENCODER_H */
