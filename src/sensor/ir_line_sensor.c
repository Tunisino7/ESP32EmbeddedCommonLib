#include "ESP32EmbeddedCommonLib/sensor/ir_line_sensor.h"

/* Validate the required digital input pin for an IR line sensor. */
static esp_err_t ir_line_sensor_validate_config(
    const ecl_ir_line_sensor_config_t *config
) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->digital_pin == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/* Build a default digital-only IR line sensor configuration. */
ecl_ir_line_sensor_config_t ecl_ir_line_sensor_default_config(
    gpio_num_t digital_pin
) {
    ecl_ir_line_sensor_config_t config = {
        .digital_pin    = digital_pin,
        .active_high    = false,
        .analog_enabled = false,
        .analog_unit    = ADC_UNIT_1,
        .analog_channel = ADC_CHANNEL_0,
        .analog_atten   = ADC_ATTEN_DB_12,
    };

    return config;
}

/* Build an IR line sensor configuration with ADC analog reading enabled. */
ecl_ir_line_sensor_config_t ecl_ir_line_sensor_config_with_analog(
    gpio_num_t    digital_pin,
    bool          active_high,
    adc_unit_t    adc_unit,
    adc_channel_t adc_channel
) {
    ecl_ir_line_sensor_config_t config = {
        .digital_pin    = digital_pin,
        .active_high    = active_high,
        .analog_enabled = true,
        .analog_unit    = adc_unit,
        .analog_channel = adc_channel,
        .analog_atten   = ADC_ATTEN_DB_12,
    };

    return config;
}

/* Configure digital GPIO and optional ADC resources for the line sensor. */
esp_err_t ecl_ir_line_sensor_init(
    ecl_ir_line_sensor_t *sensor,
    const ecl_ir_line_sensor_config_t *config
) {
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ir_line_sensor_validate_config(config);
    if (err != ESP_OK) {
        return err;
    }

    sensor->initialized = false;
    sensor->adc_handle  = NULL;
    sensor->config      = *config;

    /* D0 — digital input */
    err = gpio_reset_pin(sensor->config.digital_pin);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_set_direction(sensor->config.digital_pin, GPIO_MODE_INPUT);
    if (err != ESP_OK) {
        return err;
    }

    /* A0 — analog input (optional) */
    if (sensor->config.analog_enabled) {
        adc_oneshot_unit_init_cfg_t unit_cfg = {
            .unit_id = sensor->config.analog_unit,
        };

        err = adc_oneshot_new_unit(&unit_cfg, &sensor->adc_handle);
        if (err != ESP_OK) {
            return err;
        }

        adc_oneshot_chan_cfg_t chan_cfg = {
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten    = sensor->config.analog_atten,
        };

        err = adc_oneshot_config_channel(
            sensor->adc_handle,
            sensor->config.analog_channel,
            &chan_cfg
        );

        if (err != ESP_OK) {
            adc_oneshot_del_unit(sensor->adc_handle);
            sensor->adc_handle = NULL;
            return err;
        }
    }

    sensor->initialized = true;

    return ESP_OK;
}

/* Release optional ADC resources and mark the line sensor inactive. */
esp_err_t ecl_ir_line_sensor_deinit(ecl_ir_line_sensor_t *sensor) {
    if (sensor == NULL || !sensor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (sensor->adc_handle != NULL) {
        esp_err_t err = adc_oneshot_del_unit(sensor->adc_handle);
        if (err != ESP_OK) {
            return err;
        }
        sensor->adc_handle = NULL;
    }

    sensor->initialized = false;

    return ESP_OK;
}

/* Read the digital comparator output and convert it to detected/not detected. */
esp_err_t ecl_ir_line_sensor_read(
    const ecl_ir_line_sensor_t *sensor,
    bool *detected
) {
    if (sensor == NULL || !sensor->initialized || detected == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int level = gpio_get_level(sensor->config.digital_pin);

    *detected = sensor->config.active_high ? (level == 1) : (level == 0);

    return ESP_OK;
}

/* Read the optional raw ADC value from the analog output. */
esp_err_t ecl_ir_line_sensor_read_raw(
    const ecl_ir_line_sensor_t *sensor,
    int *raw
) {
    if (sensor == NULL || !sensor->initialized || raw == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!sensor->config.analog_enabled || sensor->adc_handle == NULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    return adc_oneshot_read(sensor->adc_handle, sensor->config.analog_channel, raw);
}
