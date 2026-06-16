#ifndef ECL_MOTOR_CONTROL_H
#define ECL_MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "ESP32EmbeddedCommonLib/motor/hbridge.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Physical parameters for one motor + wheel assembly.
 *
 * rpm_max and wheel_radius_m are robot-specific and must be provided by the
 * application.  All other conversions are derived from these two values.
 *
 * @note  rpm_max is the no-load output-shaft RPM at 100 % duty.  Measure it
 *        empirically for best accuracy; it varies with supply voltage and load.
 */
typedef struct {
    float rpm_max;        /**< No-load output-shaft RPM at 100 % duty.          */
    float wheel_radius_m; /**< Driven wheel radius in metres.                   */
} ecl_motor_control_config_t;

/**
 * @brief Single-motor control handle.
 *
 * Wraps any ecl_hbridge_t with physical unit conversion.
 * Chip-agnostic: works with DRV8833, TB6612, L298N, or any other driver
 * that provides an ecl_hbridge_t adapter.
 *
 * Two instances (left, right) cover a differential-drive robot.
 */
typedef struct {
    ecl_hbridge_t              hbridge;     /**< Chip-agnostic motor interface. */
    ecl_motor_control_config_t config;
    bool                                initialized;
} ecl_motor_control_t;

/**
 * @brief Initialise a motor control handle.
 *
 * The hbridge must already be bound (e.g. via ecl_drv8833_bind_hbridge)
 * before calling this function.
 *
 * @param motor    Pointer to an uninitialised handle.
 * @param hbridge  Chip-agnostic H-bridge handle for this motor.
 * @param config   Physical parameters (rpm_max, wheel_radius_m).
 */
esp_err_t ecl_motor_control_init(
    ecl_motor_control_t              *motor,
    const ecl_hbridge_t              *hbridge,
    const ecl_motor_control_config_t *config
);

/**
 * @brief Set motor speed as raw duty percentage.
 *
 * @param motor  Initialised handle.
 * @param pct    Speed [-100, +100] %. Positive = forward, negative = reverse.
 */
esp_err_t ecl_motor_control_set_speed_pct(
    ecl_motor_control_t *motor,
    int8_t pct
);

/**
 * @brief Set motor speed in output-shaft RPM (open-loop, linear mapping).
 *
 * Converts: duty = (rpm / rpm_max) × 100 %
 * Clamped to [-100, +100] %.
 *
 * @param motor  Initialised handle.
 * @param rpm    Target RPM. Positive = forward.
 */
esp_err_t ecl_motor_control_set_speed_rpm(
    ecl_motor_control_t *motor,
    float rpm
);

/**
 * @brief Set motor speed in metres per second.
 *
 * Converts: rpm = (v × 60) / (2π × wheel_radius_m)
 * Then delegates to set_speed_rpm.
 *
 * @param motor  Initialised handle.
 * @param ms     Target speed in m/s. Positive = forward.
 */
esp_err_t ecl_motor_control_set_speed_ms(
    ecl_motor_control_t *motor,
    float ms
);

/**
 * @brief Stop the motor (coast or brake per DRV8833 config.slow_decay).
 */
esp_err_t ecl_motor_control_stop(ecl_motor_control_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* ECL_MOTOR_CONTROL_H */
