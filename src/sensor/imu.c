#include "ESP32EmbeddedCommonLib/sensor/imu.h"

#include <stddef.h>

esp_err_t ecl_sensor_imu_read_accel(
    const ecl_sensor_imu_t *imu,
    ecl_sensor_imu_vec3_t *accel_g
)
{
    if (imu == NULL || imu->read_accel == NULL || accel_g == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return imu->read_accel(imu->ctx, accel_g);
}

esp_err_t ecl_sensor_imu_read_gyro(
    const ecl_sensor_imu_t *imu,
    ecl_sensor_imu_vec3_t *gyro_dps
)
{
    if (imu == NULL || imu->read_gyro == NULL || gyro_dps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return imu->read_gyro(imu->ctx, gyro_dps);
}

esp_err_t ecl_sensor_imu_deinit(ecl_sensor_imu_t *imu)
{
    if (imu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (imu->deinit == NULL) {
        return ESP_OK;
    }

    esp_err_t err = imu->deinit(imu->ctx);
    if (err == ESP_OK) {
        imu->read_accel = NULL;
        imu->read_gyro  = NULL;
        imu->deinit     = NULL;
        imu->ctx        = NULL;
    }
    return err;
}
