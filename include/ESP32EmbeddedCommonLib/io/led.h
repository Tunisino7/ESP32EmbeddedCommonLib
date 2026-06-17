#ifndef ECL_LED_H
#define ECL_LED_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    gpio_num_t pin;

    /* true  → logic 1 turns the LED on  (common anode to 3V3, active-high).
       false → logic 0 turns the LED on  (common cathode to GND, active-low). */
    bool active_high;
} ecl_led_config_t;

typedef struct {
    ecl_led_config_t config;
    bool initialized;
    bool state; /* logical state: true = on, false = off */
} ecl_led_t;

ecl_led_config_t ecl_io_led_default_config(gpio_num_t pin);

esp_err_t ecl_io_led_init(
    ecl_led_t *led,
    const ecl_led_config_t *config
);

esp_err_t ecl_io_led_on(ecl_led_t *led);

esp_err_t ecl_io_led_off(ecl_led_t *led);

esp_err_t ecl_io_led_toggle(ecl_led_t *led);

esp_err_t ecl_io_led_set(ecl_led_t *led, bool on);

#endif /* ECL_LED_H */
