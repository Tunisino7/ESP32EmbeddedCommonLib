#ifndef ECL_ULTRASONIC_SENSOR_H
#define ECL_ULTRASONIC_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    gpio_num_t trigger_pin;
    gpio_num_t echo_pin;

    /* HC-SR04 practical default: about 30 ms covers about 5 m round trip. */
    uint32_t timeout_us;

    /* Speed of sound at about 20 °C. Adjust if temperature compensation matters. */
    float speed_of_sound_mps;
} ecl_ultrasonic_sensor_config_t;

typedef struct {
    ecl_ultrasonic_sensor_config_t config;
    bool initialized;
} ecl_ultrasonic_sensor_t;

ecl_ultrasonic_sensor_config_t ecl_ultrasonic_sensor_default_config(
    gpio_num_t trigger_pin,
    gpio_num_t echo_pin
);

esp_err_t ecl_ultrasonic_sensor_init(
    ecl_ultrasonic_sensor_t *sensor,
    const ecl_ultrasonic_sensor_config_t *config
);

esp_err_t ecl_ultrasonic_sensor_measure_distance_cm(
    ecl_ultrasonic_sensor_t *sensor,
    float *distance_cm
);

esp_err_t ecl_ultrasonic_sensor_measure_pulse_us(
    ecl_ultrasonic_sensor_t *sensor,
    uint32_t *pulse_width_us
);

#endif /* ECL_ULTRASONIC_SENSOR_H */
