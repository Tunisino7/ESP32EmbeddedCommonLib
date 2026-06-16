#include "ESP32EmbeddedCommonLib/algo/pid.h"

#include <stddef.h>

/* ── Public API ──────────────────────────────────────────────────────────── */

void esp32_common_pid_init(
    esp32_common_pid_t *pid,
    float kp, float ki, float kd,
    float output_min, float output_max,
    float integral_limit)
{
    if (pid == NULL) return;

    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->output_min     = output_min;
    pid->output_max     = output_max;
    pid->integral_limit = integral_limit;
    pid->integral       = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first_update   = true;
}

float esp32_common_pid_update(
    esp32_common_pid_t *pid,
    float setpoint,
    float measurement,
    float dt_s)
{
    if (pid == NULL || dt_s <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    /* Proportional */
    float p_term = pid->kp * error;

    /* Integral with anti-windup clamp */
    pid->integral += pid->ki * error * dt_s;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    /* Derivative on measurement (avoids kick on setpoint step) */
    float d_term = 0.0f;
    if (!pid->first_update) {
        d_term = -pid->kd * (measurement - pid->prev_measurement) / dt_s;
    }
    pid->first_update     = false;
    pid->prev_measurement = measurement;

    /* Sum and clamp output */
    float output = p_term + pid->integral + d_term;
    if (output >  pid->output_max) output =  pid->output_max;
    if (output <  pid->output_min) output =  pid->output_min;

    return output;
}

void esp32_common_pid_reset(esp32_common_pid_t *pid)
{
    if (pid == NULL) return;
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first_update     = true;
}
