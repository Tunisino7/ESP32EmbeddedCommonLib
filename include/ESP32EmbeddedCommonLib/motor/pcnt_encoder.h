#ifndef ECL_PCNT_ENCODER_H
#define ECL_PCNT_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Hardware counter limits ─────────────────────────────────────────────── */
#define ECL_PCNT_ENCODER_HIGH  32767
#define ECL_PCNT_ENCODER_LOW  (-32768)

/**
 * @brief Configuration for a quadrature (or single-channel) Hall encoder.
 *
 * Decode mode depends on whether pin_b is wired:
 *   - pin_b != GPIO_NUM_NC → 4× quadrature (both channels, direction-aware)
 *   - pin_b == GPIO_NUM_NC → 2× single-channel (both edges of A, counts up only)
 *
 * For N20 motors: pulses_per_rev = 7 (pre-gearbox magnetic PPR).
 */
typedef struct {
    gpio_num_t pin_a;           /**< Encoder channel A.                          */
    gpio_num_t pin_b;           /**< Encoder channel B. GPIO_NUM_NC = 2× mode.   */
    uint16_t   pulses_per_rev;  /**< PPR before gearbox.                         */
    uint16_t   gear_ratio;      /**< Gearbox reduction (1 = no gearbox).         */
} ecl_pcnt_encoder_config_t;

/**
 * @brief Runtime state for one encoder instance.
 *
 * Overflow-safe: an ISR callback accumulates the hardware watch-point value
 * into accum_pulses whenever the 16-bit PCNT counter wraps. A spinlock makes
 * the combined read (accum + live hardware count) atomic with respect to that
 * ISR.
 */
typedef struct {
    ecl_pcnt_encoder_config_t config;
    bool                  initialized;
    pcnt_unit_handle_t    pcnt_unit;
    pcnt_channel_handle_t pcnt_chan_a;
    pcnt_channel_handle_t pcnt_chan_b;    /**< NULL in 2× mode.          */
    portMUX_TYPE          spinlock;
    volatile int64_t      accum_pulses;   /**< Overflow accumulator.     */
    uint16_t              counts_per_motor_rev; /**< PPR × decode factor.*/
    int64_t               rpm_ref_pulses;
    int64_t               rpm_ref_time_us;
    float                 rpm;            /**< Last computed output RPM. */
} ecl_pcnt_encoder_t;

/**
 * @brief Build a default configuration.
 *
 * Sets pulses_per_rev = 7 (N20 Hall), gear_ratio = 1.
 *
 * @param pin_a Encoder channel A.
 * @param pin_b Encoder channel B (GPIO_NUM_NC for 2× mode).
 */
ecl_pcnt_encoder_config_t ecl_pcnt_encoder_default_config(
    gpio_num_t pin_a,
    gpio_num_t pin_b
);

/**
 * @brief Initialise the PCNT encoder.
 *
 * Configures PCNT quadrature decoding, overflow watch-points and ISR callback.
 *
 * @param enc     Pointer to an uninitialised encoder instance.
 * @param config  Hardware configuration (copied into the instance).
 */
esp_err_t ecl_pcnt_encoder_init(
    ecl_pcnt_encoder_t              *enc,
    const ecl_pcnt_encoder_config_t *config
);

/**
 * @brief Read accumulated pulse count since init or last reset.
 *
 * Sign: positive = net forward, negative = net reverse.
 *
 * @param enc     Initialised encoder.
 * @param pulses  Output: int64 overflow-safe count.
 */
esp_err_t ecl_pcnt_encoder_get_pulses(
    ecl_pcnt_encoder_t *enc,
    int64_t *pulses
);

/**
 * @brief Compute instantaneous output-shaft RPM.
 *
 * Call periodically from a single task (50–200 ms recommended).
 * First call seeds the reference and returns 0.0.
 *
 * @param enc  Initialised encoder.
 * @param rpm  Output: output-shaft RPM (positive = forward).
 */
esp_err_t ecl_pcnt_encoder_get_rpm(
    ecl_pcnt_encoder_t *enc,
    float *rpm
);

/**
 * @brief Reset pulse accumulator, PCNT hardware counter, and RPM reference.
 */
esp_err_t ecl_pcnt_encoder_reset(ecl_pcnt_encoder_t *enc);

/**
 * @brief Release all PCNT hardware resources.
 */
esp_err_t ecl_pcnt_encoder_deinit(ecl_pcnt_encoder_t *enc);

#ifdef __cplusplus
}
#endif

#endif /* ECL_PCNT_ENCODER_H */
