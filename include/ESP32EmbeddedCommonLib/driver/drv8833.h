#ifndef ESP32_EMBEDDED_COMMON_LIB_DRV8833_H
#define ESP32_EMBEDDED_COMMON_LIB_DRV8833_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "ESP32EmbeddedCommonLib/motor/hbridge.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── DRV8833 chip constants ──────────────────────────────────────────────── */
/** Number of independent H-bridge channels per DRV8833 chip. */
#define ESP32_COMMON_DRV8833_CHANNELS         2U
/** Maximum continuous output current per channel (A, datasheet). */
#define ESP32_COMMON_DRV8833_CURRENT_MAX_A    1.5f
/** Peak output current per channel (A, datasheet). */
#define ESP32_COMMON_DRV8833_CURRENT_PEAK_A   2.0f
/** Minimum motor supply voltage (V). */
#define ESP32_COMMON_DRV8833_VM_MIN_V         2.7f
/** Maximum motor supply voltage (V). */
#define ESP32_COMMON_DRV8833_VM_MAX_V         10.8f

/* ── PWM defaults ────────────────────────────────────────────────────────── */
#define ESP32_COMMON_DRV8833_PWM_FREQ_HZ     10000U
#define ESP32_COMMON_DRV8833_PWM_RESOLUTION  LEDC_TIMER_10_BIT

/**
 * @brief Identifies one of the two H-bridge channels on a DRV8833 chip.
 */
typedef enum {
    ESP32_COMMON_DRV8833_CHANNEL_A = 0, /**< AIN1 / AIN2 (motor A). */
    ESP32_COMMON_DRV8833_CHANNEL_B = 1, /**< BIN1 / BIN2 (motor B). */
} esp32_common_drv8833_channel_t;

/**
 * @brief Pin and LEDC assignments for one H-bridge channel.
 *
 * Unlike L298N / TB6612 (IN1, IN2, EN), the DRV8833 has **no separate enable
 * pin**.  Speed is controlled by PWM-modulating xIN1 or xIN2 directly, so
 * two independent LEDC channels are required per H-bridge channel.
 */
typedef struct {
    gpio_num_t     pin_in1;      /**< xIN1 GPIO (AIN1 or BIN1).          */
    gpio_num_t     pin_in2;      /**< xIN2 GPIO (AIN2 or BIN2).          */
    ledc_channel_t ledc_ch_in1;  /**< LEDC channel assigned to pin_in1.  */
    ledc_channel_t ledc_ch_in2;  /**< LEDC channel assigned to pin_in2.  */
} esp32_common_drv8833_channel_cfg_t;

/**
 * @brief Chip-level configuration for one DRV8833.
 *
 * **Decay mode** determines what happens during the OFF phase of the PWM:
 *
 *   | slow_decay | OFF-phase state           | Effect                          |
 *   |------------|---------------------------|---------------------------------|
 *   | false (default, fast decay) | both low-sides open → Hi-Z | coast, lower EMF |
 *   | true  (slow decay)          | both low-sides closed      | recirculate, gentler decel |
 *
 * Stopping (speed = 0) with slow decay = active brake (both low-sides ON).
 * Stopping with fast decay = coast (Hi-Z).
 *
 * **nSLEEP** (active-LOW): drive LOW to put the chip into ultra-low-power sleep;
 * drive HIGH (or leave floating with internal pull-up) to enable.  Set
 * pin_nsleep = GPIO_NUM_NC when tied directly to 3.3 V.
 *
 * **nFAULT** (active-LOW open-drain output): the chip asserts this LOW on
 * overcurrent or overtemperature.  Set pin_nfault = GPIO_NUM_NC if not used.
 */
typedef struct {
    esp32_common_drv8833_channel_cfg_t channel[ESP32_COMMON_DRV8833_CHANNELS];
    ledc_timer_t     ledc_timer;     /**< Shared LEDC timer for all 4 INx channels.      */
    uint32_t         pwm_freq_hz;    /**< PWM carrier frequency (Hz).                    */
    ledc_timer_bit_t pwm_resolution; /**< Duty-cycle bit-width.                          */
    bool             slow_decay;     /**< Decay mode: false = fast, true = slow.         */
    gpio_num_t       pin_nsleep;     /**< nSLEEP GPIO (active-LOW). GPIO_NUM_NC = N/C.   */
    gpio_num_t       pin_nfault;     /**< nFAULT GPIO (active-LOW in). GPIO_NUM_NC = N/C.*/
} esp32_common_drv8833_config_t;

/** Runtime state for one DRV8833 chip. */
typedef struct {
    esp32_common_drv8833_config_t config;
    bool     initialized;
    bool     sleeping;
    int8_t   speed[ESP32_COMMON_DRV8833_CHANNELS]; /**< Last commanded speed per channel. */
    uint32_t max_duty;                              /**< (1 << pwm_resolution) − 1.        */
} esp32_common_drv8833_t;

/**
 * @brief Build a default configuration for a DRV8833 chip.
 *
 * Assigns LEDC_TIMER_1, channels LEDC_CHANNEL_0…3 (A_IN1, A_IN2, B_IN1,
 * B_IN2), 10 kHz, 10-bit, fast decay, nSLEEP / nFAULT not connected.
 *
 * @note If another driver already uses LEDC_TIMER_1 or channels 0–3, update
 *       config.ledc_timer and config.channel[].ledc_ch_in1/in2 before init.
 *
 * @param pin_ain1 AIN1 GPIO.  @param pin_ain2 AIN2 GPIO.
 * @param pin_bin1 BIN1 GPIO.  @param pin_bin2 BIN2 GPIO.
 */
esp32_common_drv8833_config_t esp32_common_drv8833_default_config(
    gpio_num_t pin_ain1, gpio_num_t pin_ain2,
    gpio_num_t pin_bin1, gpio_num_t pin_bin2
);

/**
 * @brief Initialise a DRV8833 (configure GPIOs + LEDC timer and channels).
 *
 * Wakes the chip (deasserts nSLEEP) if pin_nsleep is configured.
 * Both motor outputs start in the coast/brake-0 state.
 *
 * @param drv     Pointer to an uninitialised instance.
 * @param config  Hardware configuration (copied into the instance).
 */
esp_err_t esp32_common_drv8833_init(
    esp32_common_drv8833_t              *drv,
    const esp32_common_drv8833_config_t *config
);

/**
 * @brief Set speed for one channel.
 *
 * @param drv       Initialised DRV8833 instance.
 * @param ch        CHANNEL_A or CHANNEL_B.
 * @param speed_pct Speed [−100, +100] %. Positive = forward, negative = reverse.
 */
esp_err_t esp32_common_drv8833_set_speed(
    esp32_common_drv8833_t        *drv,
    esp32_common_drv8833_channel_t ch,
    int8_t speed_pct
);

/**
 * @brief Stop one channel.
 *
 * Behaviour depends on config.slow_decay:
 *   - false → coast (both INx = 0, Hi-Z output)
 *   - true  → active brake (both INx = 1, low-side recirculation)
 */
esp_err_t esp32_common_drv8833_stop(
    esp32_common_drv8833_t        *drv,
    esp32_common_drv8833_channel_t ch
);

/**
 * @brief Assert nSLEEP LOW — put the chip into low-power sleep mode.
 *
 * Both motor outputs become Hi-Z (coast) regardless of INx states.
 * No-op if pin_nsleep was not configured (GPIO_NUM_NC).
 */
esp_err_t esp32_common_drv8833_sleep(esp32_common_drv8833_t *drv);

/**
 * @brief Deassert nSLEEP HIGH — wake the chip from sleep mode.
 *
 * No-op if pin_nsleep was not configured (GPIO_NUM_NC).
 */
esp_err_t esp32_common_drv8833_wake(esp32_common_drv8833_t *drv);

/**
 * @brief Read the nFAULT pin.
 *
 * nFAULT is driven LOW by the chip on overcurrent or overtemperature.
 *
 * @param drv    Initialised instance.
 * @param fault  Output: true = fault active (nFAULT = LOW).
 * @return ESP_ERR_NOT_SUPPORTED if pin_nfault = GPIO_NUM_NC.
 */
esp_err_t esp32_common_drv8833_is_fault(
    const esp32_common_drv8833_t *drv,
    bool *fault
);

/**
 * @brief Stop both channels and release all LEDC resources.
 */
esp_err_t esp32_common_drv8833_deinit(esp32_common_drv8833_t *drv);

/* ── H-bridge adapter ────────────────────────────────────────────────────── */

/**
 * @brief Opaque per-channel context for the generic hbridge adapter.
 *
 * Allocate one instance per motor channel and keep it alive for as long as
 * the associated esp32_common_hbridge_t is in use.
 */
typedef struct {
    esp32_common_drv8833_t        *drv;     /**< Pointer to the initialised DRV8833.   */
    esp32_common_drv8833_channel_t channel; /**< H-bridge channel this motor uses.     */
} esp32_common_drv8833_hbridge_ctx_t;

/**
 * @brief Bind one DRV8833 channel to a generic esp32_common_hbridge_t.
 *
 * Populates @p out with set_speed / stop callbacks that delegate to the
 * specified channel on @p drv.  The caller must allocate @p ctx and keep it
 * alive for the lifetime of @p out.
 *
 * Usage:
 * @code
 *   static esp32_common_drv8833_hbridge_ctx_t ctx_left;
 *   static esp32_common_hbridge_t             hbridge_left;
 *   esp32_common_drv8833_bind_hbridge(
 *       &bridge, ESP32_COMMON_DRV8833_CHANNEL_A, &ctx_left, &hbridge_left);
 * @endcode
 *
 * @param drv      Initialised DRV8833 instance.
 * @param channel  Channel to bind (A or B).
 * @param ctx      Caller-allocated context storage.
 * @param out      hbridge handle to populate.
 * @return ESP_ERR_INVALID_ARG if any pointer is NULL.
 */
esp_err_t esp32_common_drv8833_bind_hbridge(
    esp32_common_drv8833_t             *drv,
    esp32_common_drv8833_channel_t      channel,
    esp32_common_drv8833_hbridge_ctx_t *ctx,
    esp32_common_hbridge_t             *out
);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_EMBEDDED_COMMON_LIB_DRV8833_H */
