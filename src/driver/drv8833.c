#include "ESP32EmbeddedCommonLib/driver/drv8833.h"

#include <stddef.h>
#include <stdint.h>

/* ── Private helpers ─────────────────────────────────────────────────────── */

/* Set and latch an LEDC duty value for one DRV8833 input pin. */
static esp_err_t drv8833_ledc_set(ledc_channel_t ch, uint32_t duty)
{
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    if (err != ESP_OK) return err;
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/*
 * Translates a signed speed percentage to DRV8833 INx duty values and
 * applies them.
 *
 * DRV8833 truth table (xIN1, xIN2):
 *   0, 0 → coast (Hi-Z)
 *   1, 0 → forward
 *   0, 1 → reverse
 *   1, 1 → brake (low-side slow decay)
 *
 * Fast decay — low-side opens during OFF time (motor coasts):
 *   Forward at X%:  IN1 = duty,      IN2 = 0
 *   Reverse at X%:  IN1 = 0,         IN2 = duty
 *   Stop:           IN1 = 0,         IN2 = 0  (coast)
 *
 * Slow decay — low-side stays closed during OFF time (current recirculates):
 *   Forward at X%:  IN1 = max,       IN2 = max - duty  (IN2 = inverted PWM)
 *   Reverse at X%:  IN1 = max - duty, IN2 = max
 *   Stop:           IN1 = max,       IN2 = max  (active brake)
 */
static esp_err_t drv8833_apply_speed(ecl_drv8833_t        *drv,
                                      ecl_drv8833_channel_t ch,
                                      int8_t speed_pct)
{
    const ecl_drv8833_channel_cfg_t *cfg = &drv->config.channel[ch];

    bool     fwd     = (speed_pct >= 0);
    uint8_t  abs_pct = (uint8_t)(fwd ? speed_pct : -speed_pct);
    uint32_t duty    = ((uint32_t)abs_pct * drv->max_duty) / 100U;

    uint32_t d_in1, d_in2;

    if (drv->config.slow_decay) {
        d_in1 = fwd ? drv->max_duty            : drv->max_duty - duty;
        d_in2 = fwd ? drv->max_duty - duty     : drv->max_duty;
    } else {
        d_in1 = fwd ? duty : 0U;
        d_in2 = fwd ? 0U   : duty;
    }

    esp_err_t err = drv8833_ledc_set(cfg->ledc_ch_in1, d_in1);
    if (err != ESP_OK) return err;
    return drv8833_ledc_set(cfg->ledc_ch_in2, d_in2);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

/* Build a default two-channel DRV8833 configuration for the supplied pins. */
ecl_drv8833_config_t ecl_drv8833_default_config(
    gpio_num_t pin_ain1, gpio_num_t pin_ain2,
    gpio_num_t pin_bin1, gpio_num_t pin_bin2)
{
    ecl_drv8833_config_t cfg = {
        .channel = {
            [ECL_DRV8833_CHANNEL_A] = {
                .pin_in1     = pin_ain1,
                .pin_in2     = pin_ain2,
                .ledc_ch_in1 = LEDC_CHANNEL_0,
                .ledc_ch_in2 = LEDC_CHANNEL_1,
            },
            [ECL_DRV8833_CHANNEL_B] = {
                .pin_in1     = pin_bin1,
                .pin_in2     = pin_bin2,
                .ledc_ch_in1 = LEDC_CHANNEL_2,
                .ledc_ch_in2 = LEDC_CHANNEL_3,
            },
        },
        .ledc_timer     = LEDC_TIMER_1,
        .pwm_freq_hz    = ECL_DRV8833_PWM_FREQ_HZ,
        .pwm_resolution = ECL_DRV8833_PWM_RESOLUTION,
        .slow_decay     = false,
        .pin_nsleep     = GPIO_NUM_NC,
        .pin_nfault     = GPIO_NUM_NC,
    };
    return cfg;
}

/* Configure DRV8833 GPIOs, optional control pins, and shared PWM channels. */
esp_err_t ecl_drv8833_init(
    ecl_drv8833_t              *drv,
    const ecl_drv8833_config_t *config)
{
    if (drv == NULL || config == NULL) return ESP_ERR_INVALID_ARG;

    for (int i = 0; i < (int)ECL_DRV8833_CHANNELS; i++) {
        if (config->channel[i].pin_in1 == GPIO_NUM_NC) return ESP_ERR_INVALID_ARG;
        if (config->channel[i].pin_in2 == GPIO_NUM_NC) return ESP_ERR_INVALID_ARG;
    }

    drv->initialized = false;
    drv->sleeping    = false;
    drv->config      = *config;
    drv->speed[0]    = 0;
    drv->speed[1]    = 0;
    drv->max_duty    = (1U << config->pwm_resolution) - 1U;

    /* ── nSLEEP: output, deasserted HIGH to enable the chip ─────────────── */
    if (config->pin_nsleep != GPIO_NUM_NC) {
        gpio_config_t io_cfg = {
            .pin_bit_mask = (1ULL << (uint32_t)config->pin_nsleep),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&io_cfg);
        if (err != ESP_OK) return err;

        err = gpio_set_level(config->pin_nsleep, 1); /* HIGH = chip enabled */
        if (err != ESP_OK) return err;
    }

    /* ── nFAULT: input with pull-up (open-drain active-low output) ──────── */
    if (config->pin_nfault != GPIO_NUM_NC) {
        gpio_config_t io_cfg = {
            .pin_bit_mask = (1ULL << (uint32_t)config->pin_nfault),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&io_cfg);
        if (err != ESP_OK) return err;
    }

    /* ── Shared LEDC timer ───────────────────────────────────────────────── */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = config->pwm_resolution,
        .timer_num       = config->ledc_timer,
        .freq_hz         = config->pwm_freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) return err;

    /* ── LEDC channels for all 4 INx pins ───────────────────────────────── */
    for (int i = 0; i < (int)ECL_DRV8833_CHANNELS; i++) {
        const ecl_drv8833_channel_cfg_t *ch = &config->channel[i];

        ledc_channel_config_t ch_in1 = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = ch->ledc_ch_in1,
            .timer_sel  = config->ledc_timer,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = (int)ch->pin_in1,
            .duty       = 0,
            .hpoint     = 0,
        };
        err = ledc_channel_config(&ch_in1);
        if (err != ESP_OK) return err;

        ledc_channel_config_t ch_in2 = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = ch->ledc_ch_in2,
            .timer_sel  = config->ledc_timer,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = (int)ch->pin_in2,
            .duty       = 0,
            .hpoint     = 0,
        };
        err = ledc_channel_config(&ch_in2);
        if (err != ESP_OK) return err;
    }

    drv->initialized = true;
    return ESP_OK;
}

/* Set a signed speed percentage for one DRV8833 output channel. */
esp_err_t ecl_drv8833_set_speed(
    ecl_drv8833_t        *drv,
    ecl_drv8833_channel_t ch,
    int8_t speed_pct)
{
    if (drv == NULL || !drv->initialized)           return ESP_ERR_INVALID_STATE;
    if ((unsigned)ch >= ECL_DRV8833_CHANNELS) return ESP_ERR_INVALID_ARG;

    if (speed_pct >  100) speed_pct =  100;
    if (speed_pct < -100) speed_pct = -100;

    esp_err_t err = drv8833_apply_speed(drv, ch, speed_pct);
    if (err != ESP_OK) return err;

    drv->speed[ch] = speed_pct;
    return ESP_OK;
}

/* Stop one DRV8833 channel using coast or brake behavior from decay mode. */
esp_err_t ecl_drv8833_stop(
    ecl_drv8833_t        *drv,
    ecl_drv8833_channel_t ch)
{
    if (drv == NULL || !drv->initialized)           return ESP_ERR_INVALID_STATE;
    if ((unsigned)ch >= ECL_DRV8833_CHANNELS) return ESP_ERR_INVALID_ARG;

    /*
     * speed_pct = 0 with slow_decay:
     *   d_in1 = max, d_in2 = max  → both low-sides ON → active brake.
     * speed_pct = 0 with fast_decay:
     *   d_in1 = 0,   d_in2 = 0   → both Hi-Z → coast.
     */
    esp_err_t err = drv8833_apply_speed(drv, ch, 0);
    if (err != ESP_OK) return err;

    drv->speed[ch] = 0;
    return ESP_OK;
}

/* Assert nSLEEP when available to put the DRV8833 into low-power sleep. */
esp_err_t ecl_drv8833_sleep(ecl_drv8833_t *drv)
{
    if (drv == NULL || !drv->initialized)  return ESP_ERR_INVALID_STATE;
    if (drv->config.pin_nsleep == GPIO_NUM_NC) return ESP_OK; /* tied high, no-op */

    esp_err_t err = gpio_set_level(drv->config.pin_nsleep, 0); /* LOW = sleep */
    if (err != ESP_OK) return err;

    drv->sleeping = true;
    return ESP_OK;
}

/* Deassert nSLEEP when available to wake the DRV8833 for operation. */
esp_err_t ecl_drv8833_wake(ecl_drv8833_t *drv)
{
    if (drv == NULL || !drv->initialized)  return ESP_ERR_INVALID_STATE;
    if (drv->config.pin_nsleep == GPIO_NUM_NC) return ESP_OK;

    esp_err_t err = gpio_set_level(drv->config.pin_nsleep, 1); /* HIGH = run */
    if (err != ESP_OK) return err;

    drv->sleeping = false;
    return ESP_OK;
}

/* Read the active-low nFAULT pin and report whether a fault is active. */
esp_err_t ecl_drv8833_is_fault(
    const ecl_drv8833_t *drv,
    bool *fault)
{
    if (drv == NULL || !drv->initialized) return ESP_ERR_INVALID_STATE;
    if (fault == NULL)                    return ESP_ERR_INVALID_ARG;
    if (drv->config.pin_nfault == GPIO_NUM_NC) return ESP_ERR_NOT_SUPPORTED;

    /* nFAULT is active-low: reading 0 means a fault is active. */
    *fault = (gpio_get_level(drv->config.pin_nfault) == 0);
    return ESP_OK;
}

/* Stop all channels, release PWM outputs, and optionally sleep the driver. */
esp_err_t ecl_drv8833_deinit(ecl_drv8833_t *drv)
{
    if (drv == NULL || !drv->initialized) return ESP_ERR_INVALID_STATE;

    for (int i = 0; i < (int)ECL_DRV8833_CHANNELS; i++) {
        (void)drv8833_apply_speed(drv, (ecl_drv8833_channel_t)i, 0);
        ledc_stop(LEDC_LOW_SPEED_MODE, drv->config.channel[i].ledc_ch_in1, 0);
        ledc_stop(LEDC_LOW_SPEED_MODE, drv->config.channel[i].ledc_ch_in2, 0);
    }

    if (drv->config.pin_nsleep != GPIO_NUM_NC) {
        /* Put the chip to sleep on deinit to minimise power draw. */
        gpio_set_level(drv->config.pin_nsleep, 0);
        drv->sleeping = true;
    }

    drv->initialized = false;
    return ESP_OK;
}

/* ── H-bridge adapter ────────────────────────────────────────────────────── */

/* Adapter callback that maps the generic H-bridge speed command to DRV8833. */
static esp_err_t drv8833_hb_set_speed(void *ctx, int8_t speed_pct)
{
    const ecl_drv8833_hbridge_ctx_t *c =
        (const ecl_drv8833_hbridge_ctx_t *)ctx;
    return ecl_drv8833_set_speed(c->drv, c->channel, speed_pct);
}

/* Adapter callback that maps the generic H-bridge stop command to DRV8833. */
static esp_err_t drv8833_hb_stop(void *ctx)
{
    const ecl_drv8833_hbridge_ctx_t *c =
        (const ecl_drv8833_hbridge_ctx_t *)ctx;
    return ecl_drv8833_stop(c->drv, c->channel);
}

/* Bind one DRV8833 channel to the generic H-bridge interface. */
esp_err_t ecl_drv8833_bind_hbridge(
    ecl_drv8833_t             *drv,
    ecl_drv8833_channel_t      channel,
    ecl_drv8833_hbridge_ctx_t *ctx,
    ecl_hbridge_t             *out)
{
    if (drv == NULL || ctx == NULL || out == NULL) return ESP_ERR_INVALID_ARG;

    ctx->drv     = drv;
    ctx->channel = channel;
    out->set_speed = drv8833_hb_set_speed;
    out->stop      = drv8833_hb_stop;
    out->ctx       = ctx;
    return ESP_OK;
}
