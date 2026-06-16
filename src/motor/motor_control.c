/*
 * motor_control.c — Chip-agnostic single-motor control with unit conversion
 *
 * Wraps any ecl_hbridge_t (vtable interface) and converts physical
 * speed units to a duty-cycle percentage before delegating to the chip driver.
 *
 * Conversion chain:
 *   m/s  →  rpm  =  (v × 60) / (2π × wheel_radius_m)
 *   rpm  →  pct  =  (rpm / rpm_max) × 100   — open-loop, linear mapping
 *   pct  →  H-bridge via hbridge.set_speed()
 *
 * This module does NOT include math.h; 2π is defined as a literal constant.
 */
#include "ESP32EmbeddedCommonLib/motor/motor_control.h"

#include <stddef.h>

/* Avoid relying on M_PI which is not guaranteed in C99/C11 without GNU
   extensions.  Use a local compile-time constant instead. */
#define MOTOR_CTRL_TWO_PI  6.28318530717958647692f

/* ── Private helpers ─────────────────────────────────────────────────────── */

static int8_t motor_ctrl_clamp_pct(float v)
{
    if (v >  100.0f) return  100;
    if (v < -100.0f) return -100;
    return (int8_t)v;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t ecl_motor_control_init(
    ecl_motor_control_t              *motor,
    const ecl_hbridge_t              *hbridge,
    const ecl_motor_control_config_t *config)
{
    if (motor == NULL || hbridge == NULL || config == NULL) return ESP_ERR_INVALID_ARG;
    if (hbridge->set_speed == NULL || hbridge->stop == NULL) return ESP_ERR_INVALID_ARG;
    if (config->rpm_max        <= 0.0f)                      return ESP_ERR_INVALID_ARG;
    if (config->wheel_radius_m <= 0.0f)                      return ESP_ERR_INVALID_ARG;

    motor->hbridge     = *hbridge;
    motor->config      = *config;
    motor->initialized = true;
    return ESP_OK;
}

esp_err_t ecl_motor_control_set_speed_pct(
    ecl_motor_control_t *motor,
    int8_t pct)
{
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_STATE;
    return motor->hbridge.set_speed(motor->hbridge.ctx, pct);
}

esp_err_t ecl_motor_control_set_speed_rpm(
    ecl_motor_control_t *motor,
    float rpm)
{
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_STATE;

    int8_t pct = motor_ctrl_clamp_pct((rpm / motor->config.rpm_max) * 100.0f);
    return ecl_motor_control_set_speed_pct(motor, pct);
}

esp_err_t ecl_motor_control_set_speed_ms(
    ecl_motor_control_t *motor,
    float ms)
{
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_STATE;

    /* v [m/s] → output-shaft RPM:  rpm = (v × 60) / (2π × r) */
    float rpm = (ms * 60.0f) / (MOTOR_CTRL_TWO_PI * motor->config.wheel_radius_m);
    return ecl_motor_control_set_speed_rpm(motor, rpm);
}

esp_err_t ecl_motor_control_stop(ecl_motor_control_t *motor)
{
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_STATE;
    return motor->hbridge.stop(motor->hbridge.ctx);
}
