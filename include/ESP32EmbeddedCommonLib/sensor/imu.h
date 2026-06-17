#ifndef ECL_SENSOR_IMU_H
#define ECL_SENSOR_IMU_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} ecl_sensor_imu_vec3_t;

/**
 * @brief Generic inertial measurement unit interface.
 *
 * Chip-specific drivers populate this handle through their bind function so
 * application code can read acceleration and gyroscope values without knowing
 * which IMU chip is attached.
 *
 * @note @p ctx must remain valid for the lifetime of the handle.
 */
typedef struct {
    /** Read accelerometer values in g. */
    esp_err_t (*read_accel)(void *ctx, ecl_sensor_imu_vec3_t *accel_g);
    /** Read gyroscope values in degrees per second. */
    esp_err_t (*read_gyro)(void *ctx, ecl_sensor_imu_vec3_t *gyro_dps);
    /** Optional backend deinitialiser. */
    esp_err_t (*deinit)(void *ctx);
    /** Opaque context pointer passed to callbacks. */
    void *ctx;
} ecl_sensor_imu_t;

esp_err_t ecl_sensor_imu_read_accel(
    const ecl_sensor_imu_t *imu,
    ecl_sensor_imu_vec3_t *accel_g
);

esp_err_t ecl_sensor_imu_read_gyro(
    const ecl_sensor_imu_t *imu,
    ecl_sensor_imu_vec3_t *gyro_dps
);

esp_err_t ecl_sensor_imu_deinit(ecl_sensor_imu_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* ECL_SENSOR_IMU_H */
