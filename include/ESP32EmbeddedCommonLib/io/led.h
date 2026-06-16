#ifndef ESP32_EMBEDDED_COMMON_LIB_LED_H
#define ESP32_EMBEDDED_COMMON_LIB_LED_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    gpio_num_t pin;

    /* true  → logic 1 turns the LED on  (common anode to 3V3, active-high).
       false → logic 0 turns the LED on  (common cathode to GND, active-low). */
    bool active_high;
} esp32_common_led_config_t;

typedef struct {
    esp32_common_led_config_t config;
    bool initialized;
    bool state; /* logical state: true = on, false = off */
} esp32_common_led_t;

esp32_common_led_config_t esp32_common_led_default_config(gpio_num_t pin);

esp_err_t esp32_common_led_init(
    esp32_common_led_t *led,
    const esp32_common_led_config_t *config
);

esp_err_t esp32_common_led_on(esp32_common_led_t *led);

esp_err_t esp32_common_led_off(esp32_common_led_t *led);

esp_err_t esp32_common_led_toggle(esp32_common_led_t *led);

esp_err_t esp32_common_led_set(esp32_common_led_t *led, bool on);

#endif /* ESP32_EMBEDDED_COMMON_LIB_LED_H */
