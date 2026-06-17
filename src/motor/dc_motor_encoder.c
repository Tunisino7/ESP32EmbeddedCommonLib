#include "ESP32EmbeddedCommonLib/motor/dc_motor_encoder.h"

#include <stddef.h>
#include <stdint.h>

#include "esp_timer.h"

/* ── Private: PCNT overflow / underflow ISR callback ────────────────────── */

/*
 * Called from ISR when the PCNT hardware counter reaches the high or low
 * watch-point.  edata->watch_point_value carries the exact limit value
 * (+32767 or −32768), which we accumulate so the logical counter never
 * saturates.
 */
static bool IRAM_ATTR ecl_motor_dc_encoder_pcnt_overflow_cb(
    pcnt_unit_handle_t unit,
    const pcnt_watch_event_data_t *edata,
    void *user_data)
{
    (void)unit;
    ecl_dc_motor_encoder_t *enc =
        (ecl_dc_motor_encoder_t *)user_data;

    portENTER_CRITICAL_ISR(&enc->spinlock);
    enc->accum_pulses += (int64_t)edata->watch_point_value;
    portEXIT_CRITICAL_ISR(&enc->spinlock);

    return false; /* No higher-priority task woken. */
}

/* ── Private: atomic total-pulse snapshot ───────────────────────────────── */

/*
 * Reads both the hardware PCNT counter and the software accumulator inside a
 * critical section so that a mid-read overflow callback cannot split the two
 * values and produce an incorrect sum.
 */
static int64_t ecl_motor_dc_encoder_get_total(ecl_dc_motor_encoder_t *motor)
{
    int hw = 0;

    portENTER_CRITICAL(&motor->spinlock);
    pcnt_unit_get_count(motor->pcnt_unit, &hw);
    int64_t total = motor->accum_pulses + (int64_t)hw;
    portEXIT_CRITICAL(&motor->spinlock);

    return total;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

/* Build a default combined DC motor and quadrature encoder configuration. */
ecl_dc_motor_encoder_config_t ecl_motor_dc_encoder_default_config(
    gpio_num_t pin_in1,
    gpio_num_t pin_in2,
    gpio_num_t pin_pwm,
    gpio_num_t pin_enc_a,
    gpio_num_t pin_enc_b)
{
    ecl_dc_motor_encoder_config_t cfg = {
        .motor          = ecl_motor_dc_default_config(pin_in1, pin_in2, pin_pwm),
        .pin_enc_a      = pin_enc_a,
        .pin_enc_b      = pin_enc_b,
        .pulses_per_rev = ECL_N20_MOTOR_PPR,
        .gear_ratio     = 1U,
    };
    return cfg;
}

/* Initialise the motor driver plus PCNT encoder channels and accumulator. */
esp_err_t ecl_motor_dc_encoder_init(
    ecl_dc_motor_encoder_t              *motor,
    const ecl_dc_motor_encoder_config_t *config)
{
    if (motor == NULL || config == NULL)  return ESP_ERR_INVALID_ARG;
    if (config->pin_enc_a == GPIO_NUM_NC) return ESP_ERR_INVALID_ARG;
    if (config->pulses_per_rev == 0)      return ESP_ERR_INVALID_ARG;
    if (config->gear_ratio == 0)          return ESP_ERR_INVALID_ARG;

    motor->initialized       = false;
    motor->config            = *config;
    motor->accum_pulses      = 0;
    motor->rpm_ref_pulses    = 0;
    motor->rpm_ref_time_us   = 0;
    motor->rpm               = 0.0f;
    motor->pcnt_chan_b        = NULL;
    motor->spinlock           = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

    /* Initialise underlying H-bridge / LEDC motor driver. */
    esp_err_t err = ecl_motor_dc_init(&motor->motor, &config->motor);
    if (err != ESP_OK) return err;

    /* PCNT unit with symmetric ±32767 hardware limits. */
    pcnt_unit_config_t unit_cfg = {
        .high_limit = ECL_DC_MOTOR_ENCODER_PCNT_HIGH,
        .low_limit  = ECL_DC_MOTOR_ENCODER_PCNT_LOW,
    };
    err = pcnt_new_unit(&unit_cfg, &motor->pcnt_unit);
    if (err != ESP_OK) goto fail_motor;

    /* Watch-points at the hardware limits trigger the accumulator callback. */
    err = pcnt_unit_add_watch_point(motor->pcnt_unit,
                                    ECL_DC_MOTOR_ENCODER_PCNT_HIGH);
    if (err != ESP_OK) goto fail_unit;

    err = pcnt_unit_add_watch_point(motor->pcnt_unit,
                                    ECL_DC_MOTOR_ENCODER_PCNT_LOW);
    if (err != ESP_OK) goto fail_unit;

    /* Register overflow / underflow callback. */
    pcnt_event_callbacks_t cbs = {
        .on_reach = ecl_motor_dc_encoder_pcnt_overflow_cb,
    };
    err = pcnt_unit_register_event_callbacks(motor->pcnt_unit, &cbs, motor);
    if (err != ESP_OK) goto fail_unit;

    /* ── Channel A ─────────────────────────────────────────────────────── */
    /*
     * Channel A counts on both edges.  Channel B (when wired) serves as the
     * level signal to discriminate forward from reverse (quadrature 4× mode).
     * Without B, we still count both edges of A (2× mode) but have no
     * direction information, so the counter only increments.
     */
    bool have_b = (config->pin_enc_b != GPIO_NUM_NC);

    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num  = (int)config->pin_enc_a,
        .level_gpio_num = have_b ? (int)config->pin_enc_b : -1,
    };
    err = pcnt_new_channel(motor->pcnt_unit, &chan_a_cfg, &motor->pcnt_chan_a);
    if (err != ESP_OK) goto fail_unit;

    /* Rising edge A = +1, falling edge A = −1. */
    err = pcnt_channel_set_edge_action(motor->pcnt_chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,   /* falling */
        PCNT_CHANNEL_EDGE_ACTION_INCREASE);  /* rising  */
    if (err != ESP_OK) goto fail_chan_a;

    if (have_b) {
        /* When B = LOW: keep direction.  When B = HIGH: invert direction. */
        err = pcnt_channel_set_level_action(motor->pcnt_chan_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        if (err != ESP_OK) goto fail_chan_a;

        /* ── Channel B (adds 4× full-quadrature resolution) ────────────── */
        pcnt_chan_config_t chan_b_cfg = {
            .edge_gpio_num  = (int)config->pin_enc_b,
            .level_gpio_num = (int)config->pin_enc_a,
        };
        err = pcnt_new_channel(motor->pcnt_unit, &chan_b_cfg, &motor->pcnt_chan_b);
        if (err != ESP_OK) goto fail_chan_a;

        err = pcnt_channel_set_edge_action(motor->pcnt_chan_b,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,   /* falling B when A=LOW → forward */
            PCNT_CHANNEL_EDGE_ACTION_DECREASE);  /* rising  B when A=LOW → reverse */
        if (err != ESP_OK) goto fail_chan_b;

        err = pcnt_channel_set_level_action(motor->pcnt_chan_b,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        if (err != ESP_OK) goto fail_chan_b;

        motor->counts_per_motor_rev = config->pulses_per_rev * 4U;
    } else {
        motor->counts_per_motor_rev = config->pulses_per_rev * 2U;
    }

    /* Enable, clear, and start the PCNT unit. */
    err = pcnt_unit_enable(motor->pcnt_unit);
    if (err != ESP_OK) goto fail_chan_b;

    err = pcnt_unit_clear_count(motor->pcnt_unit);
    if (err != ESP_OK) {
        pcnt_unit_disable(motor->pcnt_unit);
        goto fail_chan_b;
    }

    err = pcnt_unit_start(motor->pcnt_unit);
    if (err != ESP_OK) {
        pcnt_unit_disable(motor->pcnt_unit);
        goto fail_chan_b;
    }

    motor->initialized = true;
    return ESP_OK;

    /* ── Error unwind ────────────────────────────────────────────────────── */
fail_chan_b:
    if (motor->pcnt_chan_b != NULL) {
        pcnt_del_channel(motor->pcnt_chan_b);
        motor->pcnt_chan_b = NULL;
    }
fail_chan_a:
    pcnt_del_channel(motor->pcnt_chan_a);
fail_unit:
    pcnt_del_unit(motor->pcnt_unit);
fail_motor:
    ecl_motor_dc_deinit(&motor->motor);
    return err;
}

/* Set the underlying DC motor speed while preserving encoder state. */
esp_err_t ecl_motor_dc_encoder_set_speed(
    ecl_dc_motor_encoder_t *motor,
    int8_t speed_pct)
{
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_STATE;
    return ecl_motor_dc_set_speed(&motor->motor, speed_pct);
}

/* Stop the underlying DC motor while leaving encoder counting available. */
esp_err_t ecl_motor_dc_encoder_stop(
    ecl_dc_motor_encoder_t *motor)
{
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_STATE;
    return ecl_motor_dc_stop(&motor->motor);
}

/* Read the accumulated signed encoder pulse count. */
esp_err_t ecl_motor_dc_encoder_get_pulses(
    ecl_dc_motor_encoder_t *motor,
    int64_t *pulses)
{
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_STATE;
    if (pulses == NULL)                       return ESP_ERR_INVALID_ARG;

    *pulses = ecl_motor_dc_encoder_get_total(motor);
    return ESP_OK;
}

/* Compute output-shaft RPM since the previous RPM sample. */
esp_err_t ecl_motor_dc_encoder_get_rpm(
    ecl_dc_motor_encoder_t *motor,
    float *rpm)
{
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_STATE;
    if (rpm == NULL)                          return ESP_ERR_INVALID_ARG;

    int64_t now_us     = (int64_t)esp_timer_get_time();
    int64_t now_pulses = ecl_motor_dc_encoder_get_total(motor);

    if (motor->rpm_ref_time_us == 0) {
        /* First call: seed the reference and return 0 RPM. */
        motor->rpm_ref_time_us = now_us;
        motor->rpm_ref_pulses  = now_pulses;
        *rpm = 0.0f;
        return ESP_OK;
    }

    int64_t delta_pulses = now_pulses - motor->rpm_ref_pulses;
    int64_t delta_us     = now_us     - motor->rpm_ref_time_us;

    if (delta_us <= 0) {
        *rpm = motor->rpm;
        return ESP_OK;
    }

    /*
     * RPM at output shaft:
     *   counts_per_output_rev = counts_per_motor_rev × gear_ratio
     *   rpm = (delta_pulses / counts_per_output_rev) × (60 / delta_s)
     */
    float delta_s               = (float)delta_us * 1e-6f;
    float counts_per_output_rev = (float)motor->counts_per_motor_rev
                                  * (float)motor->config.gear_ratio;
    motor->rpm = ((float)delta_pulses / counts_per_output_rev) * (60.0f / delta_s);

    motor->rpm_ref_time_us = now_us;
    motor->rpm_ref_pulses  = now_pulses;

    *rpm = motor->rpm;
    return ESP_OK;
}

/* Clear encoder counts and reset RPM reference timing. */
esp_err_t ecl_motor_dc_encoder_reset_count(
    ecl_dc_motor_encoder_t *motor)
{
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t err = pcnt_unit_clear_count(motor->pcnt_unit);
    if (err != ESP_OK) return err;

    portENTER_CRITICAL(&motor->spinlock);
    motor->accum_pulses    = 0;
    motor->rpm_ref_pulses  = 0;
    motor->rpm_ref_time_us = 0;
    portEXIT_CRITICAL(&motor->spinlock);

    return ESP_OK;
}

/* Stop and release the motor driver and PCNT encoder resources. */
esp_err_t ecl_motor_dc_encoder_deinit(
    ecl_dc_motor_encoder_t *motor)
{
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t err = ecl_motor_dc_encoder_stop(motor);
    if (err != ESP_OK) return err;

    pcnt_unit_stop(motor->pcnt_unit);
    pcnt_unit_disable(motor->pcnt_unit);

    if (motor->pcnt_chan_b != NULL) {
        pcnt_del_channel(motor->pcnt_chan_b);
        motor->pcnt_chan_b = NULL;
    }
    pcnt_del_channel(motor->pcnt_chan_a);
    pcnt_del_unit(motor->pcnt_unit);

    ecl_motor_dc_deinit(&motor->motor);
    motor->initialized = false;
    return ESP_OK;
}
