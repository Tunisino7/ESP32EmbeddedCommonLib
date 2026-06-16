#include "ESP32EmbeddedCommonLib/sensor/bmi160.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bmi160";

/* ---- register map -------------------------------------------------------- */

#define BMI160_REG_CHIP_ID    0x00U
#define BMI160_CHIP_ID        0xD1U

#define BMI160_REG_STATUS     0x1BU  /* cmd_rdy is bit 4 */
#define BMI160_STATUS_CMD_RDY 0x10U

#define BMI160_REG_PMU_STATUS 0x03U  /* power mode status */

/* Gyro data: Gx_L Gx_H Gy_L Gy_H Gz_L Gz_H  (6 bytes) */
#define BMI160_REG_GYRO_DATA  0x0CU
/* Accel data: Ax_L Ax_H Ay_L Ay_H Az_L Az_H  (6 bytes) */
#define BMI160_REG_ACCEL_DATA 0x12U

#define BMI160_REG_GYR_CONF   0x42U  /* gyro ODR + BWP */
#define BMI160_REG_ACC_RANGE  0x41U
#define BMI160_REG_GYR_RANGE  0x43U
#define BMI160_REG_CMD        0x7EU

/* PMU_STATUS bit masks */
#define BMI160_PMU_GYR_NORMAL 0x04U  /* bits [3:2] = 01 */

#define BMI160_CMD_SOFTRESET  0xB6U
#define BMI160_CMD_ACC_NORMAL 0x11U
#define BMI160_CMD_GYR_NORMAL 0x15U

/* Startup delays per datasheet */
#define BMI160_SOFTRESET_DELAY_MS  80U  /* conservative; datasheet: 1ms min */
#define BMI160_ACC_STARTUP_MS      10U
#define BMI160_GYR_STARTUP_MS     100U  /* datasheet max; 81 ms typical */
#define BMI160_CMD_POLL_TIMEOUT_MS 200U

/* ---- I2C helpers --------------------------------------------------------- */

static esp_err_t bmi160_write(const esp32_common_bmi160_t *imu, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(imu->dev_handle, buf, sizeof(buf), -1);
}

static esp_err_t bmi160_read(const esp32_common_bmi160_t *imu,
                              uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(imu->dev_handle, &reg, 1, data, len, -1);
}

/* Wait until cmd_rdy bit is set (BMI160 ready to accept a new CMD). */
static esp_err_t bmi160_wait_cmd_ready(const esp32_common_bmi160_t *imu) {
    uint32_t elapsed_ms = 0;
    while (elapsed_ms < BMI160_CMD_POLL_TIMEOUT_MS) {
        uint8_t status = 0;
        esp_err_t err = bmi160_read(imu, BMI160_REG_STATUS, &status, 1);
        if (err != ESP_OK) {
            return err;
        }
        if (status & BMI160_STATUS_CMD_RDY) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        elapsed_ms++;
    }
    return ESP_ERR_TIMEOUT;
}

/* ---- scale factors ------------------------------------------------------- */

static float accel_scale_from_range(esp32_common_bmi160_accel_range_t range) {
    switch (range) {
        case ESP32_COMMON_BMI160_ACCEL_RANGE_2G:  return 1.0f / 16384.0f;
        case ESP32_COMMON_BMI160_ACCEL_RANGE_4G:  return 1.0f / 8192.0f;
        case ESP32_COMMON_BMI160_ACCEL_RANGE_8G:  return 1.0f / 4096.0f;
        case ESP32_COMMON_BMI160_ACCEL_RANGE_16G: return 1.0f / 2048.0f;
        default:                                   return 1.0f / 16384.0f;
    }
}

static float gyro_scale_from_range(esp32_common_bmi160_gyro_range_t range) {
    switch (range) {
        case ESP32_COMMON_BMI160_GYRO_RANGE_2000DPS: return 1.0f / 16.4f;
        case ESP32_COMMON_BMI160_GYRO_RANGE_1000DPS: return 1.0f / 32.8f;
        case ESP32_COMMON_BMI160_GYRO_RANGE_500DPS:  return 1.0f / 65.6f;
        case ESP32_COMMON_BMI160_GYRO_RANGE_250DPS:  return 1.0f / 131.2f;
        case ESP32_COMMON_BMI160_GYRO_RANGE_125DPS:  return 1.0f / 262.4f;
        default:                                      return 1.0f / 131.2f;
    }
}

/* ---- public API ---------------------------------------------------------- */

esp32_common_bmi160_config_t esp32_common_bmi160_default_config(i2c_master_bus_handle_t bus) {
    esp32_common_bmi160_config_t config = {
        .bus         = bus,
        .address     = ESP32_COMMON_BMI160_I2C_ADDR_PRIMARY,
        .accel_range = ESP32_COMMON_BMI160_ACCEL_RANGE_2G,
        .gyro_range  = ESP32_COMMON_BMI160_GYRO_RANGE_250DPS,
    };
    return config;
}

esp_err_t esp32_common_bmi160_init(
    esp32_common_bmi160_t              *imu,
    const esp32_common_bmi160_config_t *config
) {
    if (imu == NULL || config == NULL || config->bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    imu->initialized = false;
    imu->config      = *config;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = config->address,
        .scl_speed_hz    = 400000U,
    };

    esp_err_t err = i2c_master_bus_add_device(config->bus, &dev_cfg, &imu->dev_handle);
    if (err != ESP_OK) {
        return err;
    }

    /* Soft reset — resets all registers to default */
    err = bmi160_write(imu, BMI160_REG_CMD, BMI160_CMD_SOFTRESET);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(imu->dev_handle);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(BMI160_SOFTRESET_DELAY_MS));

    /* Verify chip ID */
    uint8_t chip_id = 0;
    err = bmi160_read(imu, BMI160_REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Chip ID read failed: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(imu->dev_handle);
        return err;
    }
    if (chip_id != BMI160_CHIP_ID) {
        ESP_LOGE(TAG, "Wrong chip ID: got 0x%02X, expected 0x%02X. Check wiring and I2C address.",
                 chip_id, BMI160_CHIP_ID);
        i2c_master_bus_rm_device(imu->dev_handle);
        return ESP_ERR_NOT_FOUND;
    }

    /* Power up accelerometer */
    err = bmi160_wait_cmd_ready(imu);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(imu->dev_handle);
        return err;
    }
    err = bmi160_write(imu, BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(imu->dev_handle);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(BMI160_ACC_STARTUP_MS));

    /* Power up gyroscope (needs longer startup) */
    err = bmi160_wait_cmd_ready(imu);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(imu->dev_handle);
        return err;
    }
    err = bmi160_write(imu, BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(imu->dev_handle);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(BMI160_GYR_STARTUP_MS));

    /* Verify gyro is in normal power mode */
    uint8_t pmu_status = 0;
    err = bmi160_read(imu, BMI160_REG_PMU_STATUS, &pmu_status, 1);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(imu->dev_handle);
        return err;
    }
    ESP_LOGI(TAG, "PMU_STATUS = 0x%02X (gyro bits[3:2] should be 01 = 0x04)", pmu_status);
    if ((pmu_status & 0x0CU) != BMI160_PMU_GYR_NORMAL) {
        ESP_LOGE(TAG, "Gyro did not enter normal mode (PMU_STATUS=0x%02X)", pmu_status);
        i2c_master_bus_rm_device(imu->dev_handle);
        return ESP_ERR_INVALID_STATE;
    }

    /* Set measurement ranges */
    err = bmi160_write(imu, BMI160_REG_ACC_RANGE, (uint8_t)config->accel_range);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(imu->dev_handle);
        return err;
    }

    err = bmi160_write(imu, BMI160_REG_GYR_RANGE, (uint8_t)config->gyro_range);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(imu->dev_handle);
        return err;
    }

    imu->accel_scale = accel_scale_from_range(config->accel_range);
    imu->gyro_scale  = gyro_scale_from_range(config->gyro_range);
    imu->initialized = true;

    return ESP_OK;
}

esp_err_t esp32_common_bmi160_deinit(esp32_common_bmi160_t *imu) {
    if (imu == NULL || !imu->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = i2c_master_bus_rm_device(imu->dev_handle);
    if (err != ESP_OK) {
        return err;
    }

    imu->initialized = false;
    return ESP_OK;
}

esp_err_t esp32_common_bmi160_read_accel(
    const esp32_common_bmi160_t *imu,
    esp32_common_bmi160_vec3_t  *accel_g
) {
    if (imu == NULL || !imu->initialized || accel_g == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[6];
    esp_err_t err = bmi160_read(imu, BMI160_REG_ACCEL_DATA, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }

    accel_g->x = (float)(int16_t)((buf[1] << 8) | buf[0]) * imu->accel_scale;
    accel_g->y = (float)(int16_t)((buf[3] << 8) | buf[2]) * imu->accel_scale;
    accel_g->z = (float)(int16_t)((buf[5] << 8) | buf[4]) * imu->accel_scale;

    return ESP_OK;
}

esp_err_t esp32_common_bmi160_read_gyro(
    const esp32_common_bmi160_t *imu,
    esp32_common_bmi160_vec3_t  *gyro_dps
) {
    if (imu == NULL || !imu->initialized || gyro_dps == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[6];
    esp_err_t err = bmi160_read(imu, BMI160_REG_GYRO_DATA, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }

    gyro_dps->x = (float)(int16_t)((buf[1] << 8) | buf[0]) * imu->gyro_scale;
    gyro_dps->y = (float)(int16_t)((buf[3] << 8) | buf[2]) * imu->gyro_scale;
    gyro_dps->z = (float)(int16_t)((buf[5] << 8) | buf[4]) * imu->gyro_scale;

    ESP_LOGD(TAG, "Gyro raw: x=%d y=%d z=%d",
             (int16_t)((buf[1] << 8) | buf[0]),
             (int16_t)((buf[3] << 8) | buf[2]),
             (int16_t)((buf[5] << 8) | buf[4]));

    return ESP_OK;
}
