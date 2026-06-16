#ifndef ESP32_EMBEDDED_COMMON_LIB_BMI160_H
#define ESP32_EMBEDDED_COMMON_LIB_BMI160_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#define ESP32_COMMON_BMI160_I2C_ADDR_PRIMARY   0x68U /* SDO = GND (module default) */
#define ESP32_COMMON_BMI160_I2C_ADDR_SECONDARY 0x69U /* SDO = VDD */

typedef enum {
    ESP32_COMMON_BMI160_ACCEL_RANGE_2G  = 0x03,
    ESP32_COMMON_BMI160_ACCEL_RANGE_4G  = 0x05,
    ESP32_COMMON_BMI160_ACCEL_RANGE_8G  = 0x08,
    ESP32_COMMON_BMI160_ACCEL_RANGE_16G = 0x0C,
} esp32_common_bmi160_accel_range_t;

typedef enum {
    ESP32_COMMON_BMI160_GYRO_RANGE_2000DPS = 0x00,
    ESP32_COMMON_BMI160_GYRO_RANGE_1000DPS = 0x01,
    ESP32_COMMON_BMI160_GYRO_RANGE_500DPS  = 0x02,
    ESP32_COMMON_BMI160_GYRO_RANGE_250DPS  = 0x03,
    ESP32_COMMON_BMI160_GYRO_RANGE_125DPS  = 0x04,
} esp32_common_bmi160_gyro_range_t;

typedef struct {
    float x;
    float y;
    float z;
} esp32_common_bmi160_vec3_t;

typedef struct {
    i2c_master_bus_handle_t           bus;
    uint8_t                           address;
    esp32_common_bmi160_accel_range_t accel_range;
    esp32_common_bmi160_gyro_range_t  gyro_range;
} esp32_common_bmi160_config_t;

typedef struct {
    esp32_common_bmi160_config_t config;
    bool                         initialized;
    i2c_master_dev_handle_t      dev_handle;
    float                        accel_scale; /* g per LSB */
    float                        gyro_scale;  /* deg/s per LSB */
} esp32_common_bmi160_t;

/* Defaults: address 0x68, ±2 g accel, ±250 dps gyro. */
esp32_common_bmi160_config_t esp32_common_bmi160_default_config(i2c_master_bus_handle_t bus);

esp_err_t esp32_common_bmi160_init(
    esp32_common_bmi160_t              *imu,
    const esp32_common_bmi160_config_t *config
);

esp_err_t esp32_common_bmi160_deinit(esp32_common_bmi160_t *imu);

/* Accelerometer in g (range depends on config). */
esp_err_t esp32_common_bmi160_read_accel(
    const esp32_common_bmi160_t *imu,
    esp32_common_bmi160_vec3_t  *accel_g
);

/* Gyroscope in deg/s. */
esp_err_t esp32_common_bmi160_read_gyro(
    const esp32_common_bmi160_t *imu,
    esp32_common_bmi160_vec3_t  *gyro_dps
);

#endif /* ESP32_EMBEDDED_COMMON_LIB_BMI160_H */
