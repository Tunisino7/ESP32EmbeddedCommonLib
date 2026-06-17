/*
 * led.c — GPIO LED driver
 *
 * Supports active-high (LED anode to GPIO, cathode to GND) and active-low
 * (LED cathode to GPIO via transistor, or common-anode LED) wiring.
 * The active_high flag in the config controls the polarity inversion so all
 * higher-level code calls on/off regardless of the physical wiring.
 */
#include "ESP32EmbeddedCommonLib/io/led.h"

/* Validate that the LED configuration contains a usable GPIO pin. */
static esp_err_t led_validate_config(const ecl_led_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->pin == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/* Write a logical LED state to GPIO, applying active-high/active-low polarity. */
static esp_err_t led_write(const ecl_led_t *led, bool on) {
    /* Translate logical on/off to a GPIO level, honouring polarity.
     * active_high=true:  on→GPIO=1, off→GPIO=0  (common wiring)
     * active_high=false: on→GPIO=0, off→GPIO=1  (active-low / open-drain) */
    int level = (led->config.active_high ? on : !on) ? 1 : 0;
    return gpio_set_level(led->config.pin, level);
}

/* Build a default active-high LED configuration for the supplied GPIO pin. */
ecl_led_config_t ecl_led_default_config(gpio_num_t pin) {
    ecl_led_config_t config = {
        .pin = pin,
        .active_high = true,
    };

    return config;
}

/* Configure the LED GPIO as an output and initialise it to off. */
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

/* Turn the LED on using the configured polarity. */
esp_err_t ecl_led_on(ecl_led_t *led) {
    return ecl_led_set(led, true);
}

/* Turn the LED off using the configured polarity. */
esp_err_t ecl_led_off(ecl_led_t *led) {
    return ecl_led_set(led, false);
}

/* Toggle the remembered logical LED state. */
esp_err_t ecl_led_toggle(ecl_led_t *led) {
    if (led == NULL || !led->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return ecl_led_set(led, !led->state);
}

/* Set and remember the logical LED state. */
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
