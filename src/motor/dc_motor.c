#include "ESP32EmbeddedCommonLib/motor/dc_motor.h"

#include <stddef.h>
#include <stdint.h>

/* ── Private helpers ─────────────────────────────────────────────────────── */

static esp_err_t dc_motor_validate(const esp32_common_dc_motor_config_t *config)
{
    if (config == NULL)                 return ESP_ERR_INVALID_ARG;
    if (config->pin_in1 == GPIO_NUM_NC) return ESP_ERR_INVALID_ARG;
    if (config->pin_in2 == GPIO_NUM_NC) return ESP_ERR_INVALID_ARG;
    if (config->pin_pwm == GPIO_NUM_NC) return ESP_ERR_INVALID_ARG;
    return ESP_OK;
}

static esp_err_t dc_motor_set_dir(const esp32_common_dc_motor_t *motor,
                                   bool in1, bool in2)
{
    esp_err_t err = gpio_set_level(motor->config.pin_in1, in1 ? 1 : 0);
    if (err != ESP_OK) return err;
    return gpio_set_level(motor->config.pin_in2, in2 ? 1 : 0);
}

static esp_err_t dc_motor_set_duty(const esp32_common_dc_motor_t *motor,
                                    uint32_t duty)
{
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                  motor->config.ledc_channel, duty);
    if (err != ESP_OK) return err;
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->config.ledc_channel);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp32_common_dc_motor_config_t esp32_common_dc_motor_default_config(
    gpio_num_t pin_in1,
    gpio_num_t pin_in2,
    gpio_num_t pin_pwm)
{
    esp32_common_dc_motor_config_t cfg = {
        .pin_in1        = pin_in1,
        .pin_in2        = pin_in2,
        .pin_pwm        = pin_pwm,
        .ledc_timer     = LEDC_TIMER_0,
        .ledc_channel   = LEDC_CHANNEL_0,
        .pwm_freq_hz    = ESP32_COMMON_DC_MOTOR_PWM_FREQ_HZ,
        .pwm_resolution = ESP32_COMMON_DC_MOTOR_PWM_RESOLUTION,
        .brake_on_stop  = false,
    };
    return cfg;
}

esp_err_t esp32_common_dc_motor_init(
    esp32_common_dc_motor_t              *motor,
    const esp32_common_dc_motor_config_t *config)
{
    if (motor == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t err = dc_motor_validate(config);
    if (err != ESP_OK) return err;

    motor->initialized = false;
    motor->config      = *config;
    motor->speed_pct   = 0;

    /* Configure direction GPIOs. */
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << config->pin_in1) | (1ULL << config->pin_in2),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&io_cfg);
    if (err != ESP_OK) return err;

    err = gpio_set_level(config->pin_in1, 0);
    if (err != ESP_OK) return err;
    err = gpio_set_level(config->pin_in2, 0);
    if (err != ESP_OK) return err;

    /* Configure LEDC timer. */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = config->pwm_resolution,
        .timer_num       = config->ledc_timer,
        .freq_hz         = config->pwm_freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) return err;

    /* Configure LEDC channel. */
    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = config->ledc_channel,
        .timer_sel  = config->ledc_timer,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = (int)config->pin_pwm,
        .duty       = 0,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ch_cfg);
    if (err != ESP_OK) return err;

    motor->initialized = true;
    return ESP_OK;
}

esp_err_t esp32_common_dc_motor_set_speed(
    esp32_common_dc_motor_t *motor,
    int8_t speed_pct)
{
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_STATE;

    /* Clamp to valid range. */
    if (speed_pct >  100) speed_pct =  100;
    if (speed_pct < -100) speed_pct = -100;

    if (speed_pct == 0) {
        return esp32_common_dc_motor_stop(motor);
    }

    bool     fwd      = (speed_pct > 0);
    uint8_t  abs_pct  = (uint8_t)(fwd ? speed_pct : -speed_pct);
    uint32_t max_duty = (1U << motor->config.pwm_resolution) - 1U;
    uint32_t duty     = ((uint32_t)abs_pct * max_duty) / 100U;

    esp_err_t err = dc_motor_set_dir(motor, fwd, !fwd);
    if (err != ESP_OK) return err;

    err = dc_motor_set_duty(motor, duty);
    if (err != ESP_OK) return err;

    motor->speed_pct = speed_pct;
    return ESP_OK;
}

esp_err_t esp32_common_dc_motor_stop(esp32_common_dc_motor_t *motor)
{
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t err = dc_motor_set_duty(motor, 0);
    if (err != ESP_OK) return err;

    if (motor->config.brake_on_stop) {
        err = dc_motor_set_dir(motor, true, true);
    } else {
        err = dc_motor_set_dir(motor, false, false);
    }

    motor->speed_pct = 0;
    return err;
}

esp_err_t esp32_common_dc_motor_deinit(esp32_common_dc_motor_t *motor)
{
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t err = esp32_common_dc_motor_stop(motor);
    if (err != ESP_OK) return err;

    ledc_stop(LEDC_LOW_SPEED_MODE, motor->config.ledc_channel, 0);
    motor->initialized = false;
    return ESP_OK;
}
