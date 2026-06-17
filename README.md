# ESP32EmbeddedCommonLib

Reusable ESP32 / ESP-IDF components for PlatformIO projects.

This library is written in pure C, not Arduino and not C++.

## Current components

- `ultrasonic_sensor`: blocking HC-SR04-style ultrasonic distance sensor driver using ESP-IDF GPIO and timers.
- `ir_line_sensor`, `bmi160`: sensor helpers.
- `drv8833`, `dc_motor`, `dc_motor_encoder`, `motor_control`, `pcnt_encoder`: motor drivers and control helpers.
- `pid`, `maze_solver`, `filter`, `motion_profile`, `odometry`: reusable robot algorithms.

## Include

```c
#include "ESP32EmbeddedCommonLib/sensor/ultrasonic_sensor.h"
```

## Example

```c
ecl_ultrasonic_sensor_config_t config =
    ecl_sensor_ultrasonic_sensor_default_config(GPIO_NUM_5, GPIO_NUM_18);

ecl_ultrasonic_sensor_t sensor = {0};
esp_err_t err = ecl_sensor_ultrasonic_sensor_init(&sensor, &config);

float distance_cm = 0.0f;
err = ecl_sensor_ultrasonic_sensor_measure_distance_cm(&sensor, &distance_cm);
if (err == ESP_OK) {
    /* use distance_cm */
}
```

## API

```c
ecl_ultrasonic_sensor_config_t ecl_sensor_ultrasonic_sensor_default_config(
    gpio_num_t trigger_pin,
    gpio_num_t echo_pin
);

esp_err_t ecl_sensor_ultrasonic_sensor_init(
    ecl_ultrasonic_sensor_t *sensor,
    const ecl_ultrasonic_sensor_config_t *config
);

esp_err_t ecl_sensor_ultrasonic_sensor_measure_distance_cm(
    ecl_ultrasonic_sensor_t *sensor,
    float *distance_cm
);

esp_err_t ecl_sensor_ultrasonic_sensor_measure_pulse_us(
    ecl_ultrasonic_sensor_t *sensor,
    uint32_t *pulse_width_us
);
```

## Notes

- HC-SR04 `ECHO` is often 5 V. Use a voltage divider or level shifter before connecting it to an ESP32 GPIO.
- This first implementation is blocking. For time-critical balancing control loops, run distance measurements in a separate task or call them at a low rate.
