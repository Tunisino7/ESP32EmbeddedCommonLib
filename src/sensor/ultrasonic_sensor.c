#include "ESP32EmbeddedCommonLib/sensor/ultrasonic_sensor.h"

#include "esp_rom_sys.h"
#include "esp_timer.h"

static esp_err_t ultrasonic_sensor_validate_config(
    const esp32_common_ultrasonic_sensor_config_t *config
) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->trigger_pin == GPIO_NUM_NC || config->echo_pin == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->timeout_us == 0U || config->speed_of_sound_mps <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t ultrasonic_sensor_trigger_measurement(
    const esp32_common_ultrasonic_sensor_t *sensor
) {
    esp_err_t err = gpio_set_level(sensor->config.trigger_pin, 0);
    if (err != ESP_OK) {
        return err;
    }
    esp_rom_delay_us(2);

    err = gpio_set_level(sensor->config.trigger_pin, 1);
    if (err != ESP_OK) {
        return err;
    }
    esp_rom_delay_us(10);

    return gpio_set_level(sensor->config.trigger_pin, 0);
}

static esp_err_t ultrasonic_sensor_wait_for_echo_level(
    const esp32_common_ultrasonic_sensor_t *sensor,
    int level,
    int64_t timeout_at_us
) {
    while (gpio_get_level(sensor->config.echo_pin) != level) {
        if (esp_timer_get_time() > timeout_at_us) {
            return ESP_ERR_TIMEOUT;
        }
    }

    return ESP_OK;
}

esp32_common_ultrasonic_sensor_config_t esp32_common_ultrasonic_sensor_default_config(
    gpio_num_t trigger_pin,
    gpio_num_t echo_pin
) {
    esp32_common_ultrasonic_sensor_config_t config = {
        .trigger_pin = trigger_pin,
        .echo_pin = echo_pin,
        .timeout_us = 30000U,
        .speed_of_sound_mps = 343.2f,
    };

    return config;
}

esp_err_t esp32_common_ultrasonic_sensor_init(
    esp32_common_ultrasonic_sensor_t *sensor,
    const esp32_common_ultrasonic_sensor_config_t *config
) {
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ultrasonic_sensor_validate_config(config);
    if (err != ESP_OK) {
        return err;
    }

    sensor->initialized = false;
    sensor->config = *config;

    err = gpio_reset_pin(sensor->config.trigger_pin);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_reset_pin(sensor->config.echo_pin);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_set_direction(sensor->config.trigger_pin, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_set_direction(sensor->config.echo_pin, GPIO_MODE_INPUT);
    if (err != ESP_OK) {
        return err;
    }

    /* Keep the echo pin stable when the sensor is disconnected or idle. */
    err = gpio_pulldown_en(sensor->config.echo_pin);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_pullup_dis(sensor->config.echo_pin);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_set_level(sensor->config.trigger_pin, 0);
    if (err != ESP_OK) {
        return err;
    }

    sensor->initialized = true;
    return ESP_OK;
}

esp_err_t esp32_common_ultrasonic_sensor_measure_pulse_us(
    esp32_common_ultrasonic_sensor_t *sensor,
    uint32_t *pulse_width_us
) {
    if (sensor == NULL || pulse_width_us == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!sensor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ultrasonic_sensor_trigger_measurement(sensor);
    if (err != ESP_OK) {
        return err;
    }

    const int64_t timeout_at_us = esp_timer_get_time() + sensor->config.timeout_us;

    /* Wait until ECHO rises. */
    err = ultrasonic_sensor_wait_for_echo_level(sensor, 1, timeout_at_us);
    if (err != ESP_OK) {
        return err;
    }

    const int64_t pulse_start_us = esp_timer_get_time();

    /* Wait until ECHO falls. */
    err = ultrasonic_sensor_wait_for_echo_level(sensor, 0, timeout_at_us);
    if (err != ESP_OK) {
        return err;
    }

    const int64_t pulse_end_us = esp_timer_get_time();
    *pulse_width_us = (uint32_t)(pulse_end_us - pulse_start_us);

    return ESP_OK;
}

esp_err_t esp32_common_ultrasonic_sensor_measure_distance_cm(
    esp32_common_ultrasonic_sensor_t *sensor,
    float *distance_cm
) {
    if (distance_cm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t pulse_width_us = 0U;
    esp_err_t err = esp32_common_ultrasonic_sensor_measure_pulse_us(sensor, &pulse_width_us);
    if (err != ESP_OK) {
        return err;
    }

    /* Distance = time * speed / 2. Convert microseconds to seconds and meters to cm. */
    *distance_cm = ((float)pulse_width_us * sensor->config.speed_of_sound_mps) / 20000.0f;

    return ESP_OK;
}
