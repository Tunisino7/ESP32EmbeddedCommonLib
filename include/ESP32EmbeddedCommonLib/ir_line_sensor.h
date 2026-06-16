#ifndef ESP32_EMBEDDED_COMMON_LIB_IR_LINE_SENSOR_H
#define ESP32_EMBEDDED_COMMON_LIB_IR_LINE_SENSOR_H

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

typedef struct {
    /* D0 — digital comparator output */
    gpio_num_t digital_pin;

    /* TCRT5000 module with comparator: by default the digital output is
       active-low (LOW = line/obstacle detected).  Set active_high = true
       only if your module inverts the output logic. */
    bool active_high;

    /* A0 — raw analog output (optional).
       Set analog_enabled = false to use digital only. */
    bool         analog_enabled;
    adc_unit_t   analog_unit;
    adc_channel_t analog_channel;

    /* Attenuation for the ADC input.  ADC_ATTEN_DB_12 covers 0–3.3 V. */
    adc_atten_t  analog_atten;
} esp32_common_ir_line_sensor_config_t;

typedef struct {
    esp32_common_ir_line_sensor_config_t config;
    bool initialized;
    adc_oneshot_unit_handle_t adc_handle; /* valid only when analog_enabled */
} esp32_common_ir_line_sensor_t;

/* D0 only; analog disabled. active_high defaults to false (standard TCRT5000). */
esp32_common_ir_line_sensor_config_t esp32_common_ir_line_sensor_default_config(
    gpio_num_t digital_pin
);

/* D0 + A0 analog enabled in one call. */
esp32_common_ir_line_sensor_config_t esp32_common_ir_line_sensor_config_with_analog(
    gpio_num_t    digital_pin,
    bool          active_high,
    adc_unit_t    adc_unit,
    adc_channel_t adc_channel
);

esp_err_t esp32_common_ir_line_sensor_init(
    esp32_common_ir_line_sensor_t *sensor,
    const esp32_common_ir_line_sensor_config_t *config
);

esp_err_t esp32_common_ir_line_sensor_deinit(esp32_common_ir_line_sensor_t *sensor);

/* detected = true when a reflective surface (line) is under the sensor. */
esp_err_t esp32_common_ir_line_sensor_read(
    const esp32_common_ir_line_sensor_t *sensor,
    bool *detected
);

/* Raw ADC value (0–4095 at 12-bit).  Returns ESP_ERR_NOT_SUPPORTED if
   analog was not enabled in the config. */
esp_err_t esp32_common_ir_line_sensor_read_raw(
    const esp32_common_ir_line_sensor_t *sensor,
    int *raw
);

#endif /* ESP32_EMBEDDED_COMMON_LIB_IR_LINE_SENSOR_H */
