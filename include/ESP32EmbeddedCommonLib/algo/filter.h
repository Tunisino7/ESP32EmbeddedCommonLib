#ifndef ESP32_EMBEDDED_COMMON_LIB_ALGO_FILTER_H
#define ESP32_EMBEDDED_COMMON_LIB_ALGO_FILTER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Complementary filter ────────────────────────────────────────────────── */

/**
 * @brief Complementary filter for IMU angle fusion.
 *
 * Fuses a high-frequency gyroscope with a low-frequency accelerometer reading
 * to estimate a tilt angle with low noise and no long-term drift.
 *
 * Output angle = alpha * (prev_angle + gyro_dps * dt_s)
 *              + (1 - alpha) * accel_angle_deg
 *
 * Typical alpha: 0.95 – 0.99 (higher = more gyro weight).
 */
typedef struct {
    float alpha;  /**< Gyro weight [0, 1]. Higher = less accel noise, more drift.*/
    float angle;  /**< Current fused angle estimate (degrees).                  */
} esp32_common_complementary_filter_t;

/**
 * @brief Initialise the complementary filter.
 *
 * @param f              Filter instance.
 * @param alpha          Gyro weight, typically 0.95–0.99.
 * @param initial_angle  Starting angle estimate (degrees).
 */
void esp32_common_complementary_filter_init(
    esp32_common_complementary_filter_t *f,
    float alpha,
    float initial_angle
);

/**
 * @brief Update the filter with one IMU sample.
 *
 * @param f              Initialised filter.
 * @param gyro_dps       Gyroscope rate in degrees per second (for this axis).
 * @param accel_angle_deg Tilt angle derived from accelerometer (degrees).
 * @param dt_s           Time since last update (seconds).
 * @return               Fused angle estimate (degrees).
 */
float esp32_common_complementary_filter_update(
    esp32_common_complementary_filter_t *f,
    float gyro_dps,
    float accel_angle_deg,
    float dt_s
);

/* ── Moving-average filter ───────────────────────────────────────────────── */

/** Maximum window size for the moving-average filter. Adjust as needed. */
#define ESP32_COMMON_MOVING_AVG_MAX_WINDOW  32U

/**
 * @brief Circular-buffer moving-average filter.
 *
 * Maintains a fixed-size window of past samples and returns their mean.
 * O(1) update: adds new sample, subtracts oldest, divides by window_size.
 */
typedef struct {
    float    buf[ESP32_COMMON_MOVING_AVG_MAX_WINDOW]; /**< Circular sample buffer.*/
    uint32_t window;   /**< Active window size (1 … MAX_WINDOW).                  */
    uint32_t head;     /**< Next write index.                                     */
    uint32_t count;    /**< Number of samples filled (saturates at window).       */
    float    sum;      /**< Running sum for O(1) mean.                            */
} esp32_common_moving_avg_t;

/**
 * @brief Initialise the moving-average filter.
 *
 * @param f       Filter instance.
 * @param window  Number of samples to average (1 … MAX_WINDOW).
 */
void esp32_common_moving_avg_init(
    esp32_common_moving_avg_t *f,
    uint32_t window
);

/**
 * @brief Push a new sample and return the current average.
 *
 * @param f      Initialised filter.
 * @param value  New sample.
 * @return       Running average over the window.
 */
float esp32_common_moving_avg_update(
    esp32_common_moving_avg_t *f,
    float value
);

/**
 * @brief Reset the filter (clears buffer and running sum).
 */
void esp32_common_moving_avg_reset(esp32_common_moving_avg_t *f);

/* ── Low-pass filter (single-pole IIR) ──────────────────────────────────── */

/**
 * @brief First-order IIR low-pass filter.
 *
 * output = alpha * input + (1 - alpha) * prev_output
 *
 * alpha = dt / (tau + dt)  where tau is the time constant.
 * Small alpha → heavier smoothing. Large alpha → faster response.
 */
typedef struct {
    float alpha;  /**< Smoothing coefficient [0, 1].                           */
    float output; /**< Last filtered output.                                   */
} esp32_common_lpf_t;

/**
 * @brief Initialise the low-pass filter.
 *
 * @param f              Filter instance.
 * @param alpha          Smoothing coefficient (0 < alpha ≤ 1).
 * @param initial_value  Starting output value.
 */
void esp32_common_lpf_init(
    esp32_common_lpf_t *f,
    float alpha,
    float initial_value
);

/**
 * @brief Update the low-pass filter with a new sample.
 *
 * @param f      Initialised filter.
 * @param input  Raw input sample.
 * @return       Filtered output.
 */
float esp32_common_lpf_update(esp32_common_lpf_t *f, float input);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_EMBEDDED_COMMON_LIB_ALGO_FILTER_H */
