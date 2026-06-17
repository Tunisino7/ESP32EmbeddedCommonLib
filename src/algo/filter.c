/*
 * filter.c — Signal filters for sensor fusion and noise reduction
 *
 * Three independent filter types:
 *
 *   Complementary filter:
 *     Fuses a gyroscope (fast, drifts) with an accelerometer (noisy, accurate
 *     long-term) to produce a clean angle estimate.  High-pass the gyro,
 *     low-pass the accel, sum to 1.
 *     Formula:  angle = α * (angle + ω*dt) + (1-α) * accel_angle
 *     Typical α: 0.95–0.99.  Higher α → less accel noise, more gyro drift.
 *
 *   Moving-average filter:
 *     O(1) implementation using a circular buffer + running sum.
 *     On each update: subtract oldest sample, add new sample, divide by count.
 *     No re-summation of the entire window needed.
 *
 *   Low-pass IIR filter (single-pole):
 *     Formula:  output = α*input + (1-α)*prev_output
 *     Equivalent to an exponentially weighted moving average.
 *     α = dt / (τ + dt)  where τ is the desired time constant.
 */
#include "ESP32EmbeddedCommonLib/algo/filter.h"

#include <stddef.h>
#include <string.h>

/* ── Complementary filter ────────────────────────────────────────────────── */

/* Initialise a complementary filter with a clamped gyro weight and angle. */
void ecl_complementary_filter_init(
    ecl_complementary_filter_t *f,
    float alpha,
    float initial_angle)
{
    if (f == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    f->alpha = alpha;
    f->angle = initial_angle;
}

/* Fuse one gyro rate and accelerometer angle sample into a filtered angle. */
float ecl_complementary_filter_update(
    ecl_complementary_filter_t *f,
    float gyro_dps,
    float accel_angle_deg,
    float dt_s)
{
    if (f == NULL || dt_s <= 0.0f) return 0.0f;

    f->angle = f->alpha * (f->angle + gyro_dps * dt_s)
             + (1.0f - f->alpha) * accel_angle_deg;
    /* The first term integrates the gyro rate over dt (high-pass behaviour).
     * The second term pulls the angle toward the accelerometer reading
     * (low-pass behaviour).  Alpha controls the crossover frequency. */
    return f->angle;
}

/* ── Moving-average filter ───────────────────────────────────────────────── */

/* Initialise a moving-average filter and clear its circular sample buffer. */
void ecl_moving_avg_init(
    ecl_moving_avg_t *f,
    uint32_t window)
{
    if (f == NULL) return;
    if (window == 0)                             window = 1;
    if (window > ECL_MOVING_AVG_MAX_WINDOW) window = ECL_MOVING_AVG_MAX_WINDOW;

    memset(f->buf, 0, sizeof(f->buf));
    f->window = window;
    f->head   = 0;
    f->count  = 0;
    f->sum    = 0.0f;
}

/* Add one sample to the moving-average filter and return the current mean. */
float ecl_moving_avg_update(
    ecl_moving_avg_t *f,
    float value)
{
    if (f == NULL) return 0.0f;

    /* Subtract the oldest value that will be overwritten.
     * If the buffer is not yet full (count < window) the slot is 0, so we
     * only subtract when the window is fully populated. */
    if (f->count == f->window) {
        f->sum -= f->buf[f->head];
    } else {
        f->count++;
    }

    f->buf[f->head] = value;
    f->sum         += value;
    f->head         = (f->head + 1) % f->window; /* advance write cursor, wrapping */

    return f->sum / (float)f->count;
}

/* Clear all moving-average samples while preserving the configured window. */
void ecl_moving_avg_reset(ecl_moving_avg_t *f)
{
    if (f == NULL) return;
    memset(f->buf, 0, sizeof(f->buf));
    f->head  = 0;
    f->count = 0;
    f->sum   = 0.0f;
}

/* ── Low-pass filter ─────────────────────────────────────────────────────── */

/* Initialise a first-order low-pass filter with a clamped smoothing factor. */
void ecl_lpf_init(
    ecl_lpf_t *f,
    float alpha,
    float initial_value)
{
    if (f == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    f->alpha  = alpha;
    f->output = initial_value;
}

/* Apply one first-order IIR low-pass update and return the filtered output. */
float ecl_lpf_update(ecl_lpf_t *f, float input)
{
    if (f == NULL) return input;
    f->output = f->alpha * input + (1.0f - f->alpha) * f->output;
    return f->output;
}
