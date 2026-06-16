#include "ESP32EmbeddedCommonLib/motor/pcnt_encoder.h"

#include <stddef.h>
#include <stdint.h>

#include "esp_timer.h"

/* ── Private: overflow ISR callback ─────────────────────────────────────── */

static bool IRAM_ATTR pcnt_encoder_overflow_cb(
    pcnt_unit_handle_t unit,
    const pcnt_watch_event_data_t *edata,
    void *user_data)
{
    (void)unit;
    ecl_pcnt_encoder_t *enc = (ecl_pcnt_encoder_t *)user_data;

    portENTER_CRITICAL_ISR(&enc->spinlock);
    enc->accum_pulses += (int64_t)edata->watch_point_val;
    portEXIT_CRITICAL_ISR(&enc->spinlock);

    return false;
}

/* ── Private: atomic snapshot ────────────────────────────────────────────── */

static int64_t pcnt_encoder_total(ecl_pcnt_encoder_t *enc)
{
    int hw = 0;

    portENTER_CRITICAL(&enc->spinlock);
    pcnt_unit_get_count(enc->pcnt_unit, &hw);
    int64_t total = enc->accum_pulses + (int64_t)hw;
    portEXIT_CRITICAL(&enc->spinlock);

    return total;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

ecl_pcnt_encoder_config_t ecl_pcnt_encoder_default_config(
    gpio_num_t pin_a,
    gpio_num_t pin_b)
{
    ecl_pcnt_encoder_config_t cfg = {
        .pin_a          = pin_a,
        .pin_b          = pin_b,
        .pulses_per_rev = 7U,   /* N20 Hall PPR */
        .gear_ratio     = 1U,
    };
    return cfg;
}

esp_err_t ecl_pcnt_encoder_init(
    ecl_pcnt_encoder_t              *enc,
    const ecl_pcnt_encoder_config_t *config)
{
    if (enc == NULL || config == NULL)   return ESP_ERR_INVALID_ARG;
    if (config->pin_a == GPIO_NUM_NC)    return ESP_ERR_INVALID_ARG;
    if (config->pulses_per_rev == 0)     return ESP_ERR_INVALID_ARG;
    if (config->gear_ratio == 0)         return ESP_ERR_INVALID_ARG;

    enc->initialized       = false;
    enc->config            = *config;
    enc->accum_pulses      = 0;
    enc->rpm_ref_pulses    = 0;
    enc->rpm_ref_time_us   = 0;
    enc->rpm               = 0.0f;
    enc->pcnt_chan_b        = NULL;
    enc->spinlock           = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

    /* PCNT unit */
    pcnt_unit_config_t unit_cfg = {
        .high_limit = ECL_PCNT_ENCODER_HIGH,
        .low_limit  = ECL_PCNT_ENCODER_LOW,
    };
    esp_err_t err = pcnt_new_unit(&unit_cfg, &enc->pcnt_unit);
    if (err != ESP_OK) return err;

    /* Watch-points at hardware limits */
    err = pcnt_unit_add_watch_point(enc->pcnt_unit, ECL_PCNT_ENCODER_HIGH);
    if (err != ESP_OK) goto fail_unit;

    err = pcnt_unit_add_watch_point(enc->pcnt_unit, ECL_PCNT_ENCODER_LOW);
    if (err != ESP_OK) goto fail_unit;

    /* Overflow callback */
    pcnt_event_callbacks_t cbs = { .on_reach = pcnt_encoder_overflow_cb };
    err = pcnt_unit_register_event_callbacks(enc->pcnt_unit, &cbs, enc);
    if (err != ESP_OK) goto fail_unit;

    bool have_b = (config->pin_b != GPIO_NUM_NC);

    /* Channel A */
    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num  = (int)config->pin_a,
        .level_gpio_num = have_b ? (int)config->pin_b : PCNT_PIN_NOT_USED,
    };
    err = pcnt_new_channel(enc->pcnt_unit, &chan_a_cfg, &enc->pcnt_chan_a);
    if (err != ESP_OK) goto fail_unit;

    err = pcnt_channel_set_edge_action(enc->pcnt_chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    if (err != ESP_OK) goto fail_chan_a;

    if (have_b) {
        err = pcnt_channel_set_level_action(enc->pcnt_chan_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        if (err != ESP_OK) goto fail_chan_a;

        /* Channel B */
        pcnt_chan_config_t chan_b_cfg = {
            .edge_gpio_num  = (int)config->pin_b,
            .level_gpio_num = (int)config->pin_a,
        };
        err = pcnt_new_channel(enc->pcnt_unit, &chan_b_cfg, &enc->pcnt_chan_b);
        if (err != ESP_OK) goto fail_chan_a;

        err = pcnt_channel_set_edge_action(enc->pcnt_chan_b,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        if (err != ESP_OK) goto fail_chan_b;

        err = pcnt_channel_set_level_action(enc->pcnt_chan_b,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        if (err != ESP_OK) goto fail_chan_b;

        enc->counts_per_motor_rev = config->pulses_per_rev * 4U;
    } else {
        enc->counts_per_motor_rev = config->pulses_per_rev * 2U;
    }

    err = pcnt_unit_enable(enc->pcnt_unit);
    if (err != ESP_OK) goto fail_chan_b;

    err = pcnt_unit_clear_count(enc->pcnt_unit);
    if (err != ESP_OK) {
        pcnt_unit_disable(enc->pcnt_unit);
        goto fail_chan_b;
    }

    err = pcnt_unit_start(enc->pcnt_unit);
    if (err != ESP_OK) {
        pcnt_unit_disable(enc->pcnt_unit);
        goto fail_chan_b;
    }

    enc->initialized = true;
    return ESP_OK;

fail_chan_b:
    if (enc->pcnt_chan_b != NULL) {
        pcnt_del_channel(enc->pcnt_chan_b);
        enc->pcnt_chan_b = NULL;
    }
fail_chan_a:
    pcnt_del_channel(enc->pcnt_chan_a);
fail_unit:
    pcnt_del_unit(enc->pcnt_unit);
    return err;
}

esp_err_t ecl_pcnt_encoder_get_pulses(
    ecl_pcnt_encoder_t *enc,
    int64_t *pulses)
{
    if (enc == NULL || !enc->initialized) return ESP_ERR_INVALID_STATE;
    if (pulses == NULL)                   return ESP_ERR_INVALID_ARG;

    *pulses = pcnt_encoder_total(enc);
    return ESP_OK;
}

esp_err_t ecl_pcnt_encoder_get_rpm(
    ecl_pcnt_encoder_t *enc,
    float *rpm)
{
    if (enc == NULL || !enc->initialized) return ESP_ERR_INVALID_STATE;
    if (rpm == NULL)                      return ESP_ERR_INVALID_ARG;

    int64_t now_us     = (int64_t)esp_timer_get_time();
    int64_t now_pulses = pcnt_encoder_total(enc);

    if (enc->rpm_ref_time_us == 0) {
        enc->rpm_ref_time_us = now_us;
        enc->rpm_ref_pulses  = now_pulses;
        *rpm = 0.0f;
        return ESP_OK;
    }

    int64_t delta_pulses = now_pulses - enc->rpm_ref_pulses;
    int64_t delta_us     = now_us     - enc->rpm_ref_time_us;

    if (delta_us <= 0) {
        *rpm = enc->rpm;
        return ESP_OK;
    }

    float delta_s               = (float)delta_us * 1e-6f;
    float counts_per_output_rev = (float)enc->counts_per_motor_rev
                                  * (float)enc->config.gear_ratio;
    enc->rpm = ((float)delta_pulses / counts_per_output_rev) * (60.0f / delta_s);

    enc->rpm_ref_time_us = now_us;
    enc->rpm_ref_pulses  = now_pulses;

    *rpm = enc->rpm;
    return ESP_OK;
}

esp_err_t ecl_pcnt_encoder_reset(ecl_pcnt_encoder_t *enc)
{
    if (enc == NULL || !enc->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t err = pcnt_unit_clear_count(enc->pcnt_unit);
    if (err != ESP_OK) return err;

    portENTER_CRITICAL(&enc->spinlock);
    enc->accum_pulses    = 0;
    enc->rpm_ref_pulses  = 0;
    enc->rpm_ref_time_us = 0;
    portEXIT_CRITICAL(&enc->spinlock);

    return ESP_OK;
}

esp_err_t ecl_pcnt_encoder_deinit(ecl_pcnt_encoder_t *enc)
{
    if (enc == NULL || !enc->initialized) return ESP_ERR_INVALID_STATE;

    pcnt_unit_stop(enc->pcnt_unit);
    pcnt_unit_disable(enc->pcnt_unit);

    if (enc->pcnt_chan_b != NULL) {
        pcnt_del_channel(enc->pcnt_chan_b);
        enc->pcnt_chan_b = NULL;
    }
    pcnt_del_channel(enc->pcnt_chan_a);
    pcnt_del_unit(enc->pcnt_unit);

    enc->initialized = false;
    return ESP_OK;
}
