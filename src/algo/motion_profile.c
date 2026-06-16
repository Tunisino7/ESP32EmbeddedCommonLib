/*
 * motion_profile.c — Trapezoidal velocity motion profile
 *
 * Generates a velocity setpoint that ramps smoothly between values instead of
 * stepping instantly.  This avoids wheel slip, motor stall, and mechanical
 * shock when starting or stopping.
 *
 * On each call to _update(dt_s):
 *   1. Compute the velocity error (target − current).
 *   2. Choose max_accel if accelerating, max_decel if decelerating.
 *   3. Advance current_vel by at most (rate * dt_s) toward target_vel.
 *
 * The profile is symmetric: it works for both positive and negative velocities
 * and handles direction reversals (ramp to zero then ramp back up).
 */
#include "ESP32EmbeddedCommonLib/algo/motion_profile.h"

#include <stddef.h>

/* ── Private helpers ─────────────────────────────────────────────────────── */

static float mp_fabsf(float v) { return v < 0.0f ? -v : v; }

static float mp_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void esp32_common_motion_profile_init(
    esp32_common_motion_profile_t *mp,
    float max_accel,
    float max_decel,
    float initial_vel)
{
    if (mp == NULL) return;
    if (max_accel <= 0.0f) max_accel = 1.0f;
    if (max_decel <= 0.0f) max_decel = 1.0f;

    mp->max_accel   = max_accel;
    mp->max_decel   = max_decel;
    mp->current_vel = initial_vel;
    mp->target_vel  = initial_vel;
}

void esp32_common_motion_profile_set_target(
    esp32_common_motion_profile_t *mp,
    float target_vel)
{
    if (mp == NULL) return;
    mp->target_vel = target_vel;
}

float esp32_common_motion_profile_update(
    esp32_common_motion_profile_t *mp,
    float dt_s)
{
    if (mp == NULL || dt_s <= 0.0f) return 0.0f;

    float error = mp->target_vel - mp->current_vel;

    if (mp_fabsf(error) < 1e-6f) {
        /* Snap to target when the residual is below float precision noise.
         * Avoids infinite oscillation around the target due to rounding. */
        mp->current_vel = mp->target_vel;
        return mp->current_vel;
    }

    /* Use deceleration rate when slowing down (error opposes current motion),
     * acceleration rate when speeding up. */
    float rate = (error > 0.0f) ? mp->max_accel : mp->max_decel;
    float max_step = rate * dt_s;

    /* Clamp step so we never overshoot the target. */
    if (mp_fabsf(error) <= max_step) {
        mp->current_vel = mp->target_vel;
    } else {
        mp->current_vel += (error > 0.0f ? max_step : -max_step);
    }

    return mp->current_vel;
}

bool esp32_common_motion_profile_is_settled(
    const esp32_common_motion_profile_t *mp)
{
    if (mp == NULL) return true;
    float diff = mp->target_vel - mp->current_vel;
    if (diff < 0.0f) diff = -diff;
    return diff < 1e-6f;
}
