#include "ESP32EmbeddedCommonLib/algo/filter.h"

#include <stddef.h>
#include <string.h>

/* ── Complementary filter ────────────────────────────────────────────────── */

void esp32_common_complementary_filter_init(
    esp32_common_complementary_filter_t *f,
    float alpha,
    float initial_angle)
{
    if (f == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    f->alpha = alpha;
    f->angle = initial_angle;
}

float esp32_common_complementary_filter_update(
    esp32_common_complementary_filter_t *f,
    float gyro_dps,
    float accel_angle_deg,
    float dt_s)
{
    if (f == NULL || dt_s <= 0.0f) return 0.0f;

    f->angle = f->alpha * (f->angle + gyro_dps * dt_s)
             + (1.0f - f->alpha) * accel_angle_deg;
    return f->angle;
}

/* ── Moving-average filter ───────────────────────────────────────────────── */

void esp32_common_moving_avg_init(
    esp32_common_moving_avg_t *f,
    uint32_t window)
{
    if (f == NULL) return;
    if (window == 0)                             window = 1;
    if (window > ESP32_COMMON_MOVING_AVG_MAX_WINDOW) window = ESP32_COMMON_MOVING_AVG_MAX_WINDOW;

    memset(f->buf, 0, sizeof(f->buf));
    f->window = window;
    f->head   = 0;
    f->count  = 0;
    f->sum    = 0.0f;
}

float esp32_common_moving_avg_update(
    esp32_common_moving_avg_t *f,
    float value)
{
    if (f == NULL) return 0.0f;

    /* Subtract the oldest value that will be overwritten. */
    if (f->count == f->window) {
        f->sum -= f->buf[f->head];
    } else {
        f->count++;
    }

    f->buf[f->head] = value;
    f->sum         += value;
    f->head         = (f->head + 1) % f->window;

    return f->sum / (float)f->count;
}

void esp32_common_moving_avg_reset(esp32_common_moving_avg_t *f)
{
    if (f == NULL) return;
    memset(f->buf, 0, sizeof(f->buf));
    f->head  = 0;
    f->count = 0;
    f->sum   = 0.0f;
}

/* ── Low-pass filter ─────────────────────────────────────────────────────── */

void esp32_common_lpf_init(
    esp32_common_lpf_t *f,
    float alpha,
    float initial_value)
{
    if (f == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    f->alpha  = alpha;
    f->output = initial_value;
}

float esp32_common_lpf_update(esp32_common_lpf_t *f, float input)
{
    if (f == NULL) return input;
    f->output = f->alpha * input + (1.0f - f->alpha) * f->output;
    return f->output;
}
