#ifndef ECL_BMI160_H
#define ECL_BMI160_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#define ECL_BMI160_I2C_ADDR_PRIMARY   0x68U /* SDO = GND (module default) */
#define ECL_BMI160_I2C_ADDR_SECONDARY 0x69U /* SDO = VDD */

typedef enum {
    ECL_BMI160_ACCEL_RANGE_2G  = 0x03,
    ECL_BMI160_ACCEL_RANGE_4G  = 0x05,
    ECL_BMI160_ACCEL_RANGE_8G  = 0x08,
    ECL_BMI160_ACCEL_RANGE_16G = 0x0C,
} ecl_bmi160_accel_range_t;

typedef enum {
    ECL_BMI160_GYRO_RANGE_2000DPS = 0x00,
    ECL_BMI160_GYRO_RANGE_1000DPS = 0x01,
    ECL_BMI160_GYRO_RANGE_500DPS  = 0x02,
    ECL_BMI160_GYRO_RANGE_250DPS  = 0x03,
    ECL_BMI160_GYRO_RANGE_125DPS  = 0x04,
} ecl_bmi160_gyro_range_t;

typedef struct {
    float x;
    float y;
    float z;
} ecl_bmi160_vec3_t;

typedef struct {
    i2c_master_bus_handle_t           bus;
    uint8_t                           address;
    ecl_bmi160_accel_range_t accel_range;
    ecl_bmi160_gyro_range_t  gyro_range;
} ecl_bmi160_config_t;

typedef struct {
    ecl_bmi160_config_t config;
    bool                         initialized;
    i2c_master_dev_handle_t      dev_handle;
    float                        accel_scale; /* g per LSB */
    float                        gyro_scale;  /* deg/s per LSB */
} ecl_bmi160_t;

/* Defaults: address 0x68, ±2 g accel, ±250 dps gyro. */
ecl_bmi160_config_t ecl_sensor_bmi160_default_config(i2c_master_bus_handle_t bus);

esp_err_t ecl_sensor_bmi160_init(
    ecl_bmi160_t              *imu,
    const ecl_bmi160_config_t *config
);

esp_err_t ecl_sensor_bmi160_deinit(ecl_bmi160_t *imu);

/* Accelerometer in g (range depends on config). */
esp_err_t ecl_sensor_bmi160_read_accel(
    const ecl_bmi160_t *imu,
    ecl_bmi160_vec3_t  *accel_g
);

/* Gyroscope in deg/s. */
esp_err_t ecl_sensor_bmi160_read_gyro(
    const ecl_bmi160_t *imu,
    ecl_bmi160_vec3_t  *gyro_dps
);

#endif /* ECL_BMI160_H */
