/*
 * led.c — GPIO LED driver
 *
 * Supports active-high (LED anode to GPIO, cathode to GND) and active-low
 * (LED cathode to GPIO via transistor, or common-anode LED) wiring.
 * The active_high flag in the config controls the polarity inversion so all
 * higher-level code calls on/off regardless of the physical wiring.
 */
#include "ESP32EmbeddedCommonLib/io/led.h"

static esp_err_t led_validate_config(const ecl_led_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->pin == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t led_write(const ecl_led_t *led, bool on) {
    /* Translate logical on/off to a GPIO level, honouring polarity.
     * active_high=true:  on→GPIO=1, off→GPIO=0  (common wiring)
     * active_high=false: on→GPIO=0, off→GPIO=1  (active-low / open-drain) */
    int level = (led->config.active_high ? on : !on) ? 1 : 0;
    return gpio_set_level(led->config.pin, level);
}

ecl_led_config_t ecl_led_default_config(gpio_num_t pin) {
    ecl_led_config_t config = {
        .pin = pin,
        .active_high = true,
    };

    return config;
}

esp_err_t ecl_led_init(
    ecl_led_t *led,
    const ecl_led_config_t *config
) {
    if (led == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = led_validate_config(config);
    if (err != ESP_OK) {
        return err;
    }

    led->initialized = false;
    led->state = false;
    led->config = *config;

    err = gpio_reset_pin(led->config.pin);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_set_direction(led->config.pin, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        return err;
    }

    err = led_write(led, false);
    if (err != ESP_OK) {
        return err;
    }

    led->initialized = true;

    return ESP_OK;
}

esp_err_t ecl_led_on(ecl_led_t *led) {
    return ecl_led_set(led, true);
}

esp_err_t ecl_led_off(ecl_led_t *led) {
    return ecl_led_set(led, false);
}

esp_err_t ecl_led_toggle(ecl_led_t *led) {
    if (led == NULL || !led->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return ecl_led_set(led, !led->state);
}

esp_err_t ecl_led_set(ecl_led_t *led, bool on) {
    if (led == NULL || !led->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = led_write(led, on);
    if (err != ESP_OK) {
        return err;
    }

    led->state = on;

    return ESP_OK;
}
