/*
 * pid.c — Discrete position-form PID controller
 *
 * Algorithm:
 *   output = Kp*e + Ki*∫e·dt + Kd*(−dMeasurement/dt)
 *
 * Design choices:
 *   - Derivative on measurement (not on error): differentiating the error
 *     produces a large spike ("derivative kick") whenever the setpoint
 *     changes.  Differentiating the measurement avoids this at the cost of
 *     opposing setpoint changes slightly — acceptable for motor/balance control.
 *
 *   - Anti-windup: without a limit the integral accumulates without bound
 *     while the output is saturated, causing severe overshoot on recovery.
 *     Clamping the integral to ±integral_limit prevents this.
 *
 *   - first_update flag: on the very first call prev_measurement is 0, so
 *     the derivative term would produce a false spike.  We skip the D term
 *     for the first iteration only.
 */
#include "ESP32EmbeddedCommonLib/algo/pid.h"

#include <stddef.h>

/* ── Public API ──────────────────────────────────────────────────────────── */

void ecl_pid_init(
    ecl_pid_t *pid,
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

float ecl_pid_update(
    ecl_pid_t *pid,
    float setpoint,
    float measurement,
    float dt_s)
{
    if (pid == NULL || dt_s <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    /* P term: proportional to current error. */
    float p_term = pid->kp * error;

    /* I term: accumulated error over time; clamped to prevent windup.
     * Accumulate first, then clamp — ensures the clamp is always applied
     * before the value is used in the output sum. */
    pid->integral += pid->ki * error * dt_s;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    /* D term: rate of change of the *measurement* (not the error) to avoid
     * derivative kick on setpoint steps.  Negated because:
     *   d(error)/dt = d(sp - meas)/dt = -d(meas)/dt  (sp is constant between steps). */
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

void ecl_pid_reset(ecl_pid_t *pid)
{
    if (pid == NULL) return;
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first_update     = true;
}
