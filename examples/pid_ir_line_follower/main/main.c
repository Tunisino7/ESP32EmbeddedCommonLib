/*
 * PID line follower example
 *
 * Uses three digital TCRT5000-style IR line sensors to estimate line position,
 * then applies a PID steering correction to two DRV8833-driven motors.
 */

#include "ESP32EmbeddedCommonLib/algo/pid.h"
#include "ESP32EmbeddedCommonLib/driver/drv8833.h"
#include "ESP32EmbeddedCommonLib/motor/motor_control.h"
#include "ESP32EmbeddedCommonLib/sensor/ir_line_sensor.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_AIN1       GPIO_NUM_25
#define PIN_AIN2       GPIO_NUM_26
#define PIN_BIN1       GPIO_NUM_27
#define PIN_BIN2       GPIO_NUM_14
#define PIN_NSLEEP     GPIO_NUM_12
#define PIN_NFAULT     GPIO_NUM_13

#define PIN_IR_LEFT    GPIO_NUM_32
#define PIN_IR_CENTER  GPIO_NUM_33
#define PIN_IR_RIGHT   GPIO_NUM_34

#define CONTROL_DT_MS  20U
#define BASE_SPEED_PCT 38.0f
#define MAX_SPEED_PCT  70.0f

static const char *TAG = "pid_line";

static ecl_drv8833_t s_bridge;
static ecl_drv8833_hbridge_ctx_t s_left_ctx;
static ecl_drv8833_hbridge_ctx_t s_right_ctx;
static ecl_hbridge_t s_left_hbridge;
static ecl_hbridge_t s_right_hbridge;
static ecl_motor_control_t s_left_motor;
static ecl_motor_control_t s_right_motor;
static ecl_ir_line_sensor_t s_ir_left;
static ecl_ir_line_sensor_t s_ir_center;
static ecl_ir_line_sensor_t s_ir_right;
static ecl_pid_t s_line_pid;

static int8_t clamp_speed_pct(float speed)
{
    if (speed > MAX_SPEED_PCT) {
        return (int8_t)MAX_SPEED_PCT;
    }
    if (speed < -MAX_SPEED_PCT) {
        return (int8_t)-MAX_SPEED_PCT;
    }
    return (int8_t)speed;
}

static float line_position_from_sensors(bool left, bool center, bool right)
{
    int sum = 0;
    int count = 0;

    if (left) {
        sum -= 1;
        count++;
    }
    if (center) {
        count++;
    }
    if (right) {
        sum += 1;
        count++;
    }

    if (count == 0) {
        return 0.0f;
    }
    return (float)sum / (float)count;
}

static void init_drive(void)
{
    ecl_drv8833_config_t bridge_cfg =
        ecl_driver_drv8833_default_config(PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2);
    bridge_cfg.pin_nsleep = PIN_NSLEEP;
    bridge_cfg.pin_nfault = PIN_NFAULT;

    ESP_ERROR_CHECK(ecl_driver_drv8833_init(&s_bridge, &bridge_cfg));
    ESP_ERROR_CHECK(ecl_driver_drv8833_bind_hbridge(
        &s_bridge, ECL_DRV8833_CHANNEL_A, &s_left_ctx, &s_left_hbridge));
    ESP_ERROR_CHECK(ecl_driver_drv8833_bind_hbridge(
        &s_bridge, ECL_DRV8833_CHANNEL_B, &s_right_ctx, &s_right_hbridge));

    const ecl_motor_control_config_t motor_cfg = {
        .rpm_max = 300.0f,
        .wheel_radius_m = 0.0325f,
    };
    ESP_ERROR_CHECK(ecl_motor_control_init(&s_left_motor, &s_left_hbridge, &motor_cfg));
    ESP_ERROR_CHECK(ecl_motor_control_init(&s_right_motor, &s_right_hbridge, &motor_cfg));
}

static void init_line_sensors(void)
{
    ecl_ir_line_sensor_config_t left_cfg = ecl_sensor_ir_line_sensor_default_config(PIN_IR_LEFT);
    ecl_ir_line_sensor_config_t center_cfg = ecl_sensor_ir_line_sensor_default_config(PIN_IR_CENTER);
    ecl_ir_line_sensor_config_t right_cfg = ecl_sensor_ir_line_sensor_default_config(PIN_IR_RIGHT);

    ESP_ERROR_CHECK(ecl_sensor_ir_line_sensor_init(&s_ir_left, &left_cfg));
    ESP_ERROR_CHECK(ecl_sensor_ir_line_sensor_init(&s_ir_center, &center_cfg));
    ESP_ERROR_CHECK(ecl_sensor_ir_line_sensor_init(&s_ir_right, &right_cfg));
}

void app_main(void)
{
    init_drive();
    init_line_sensors();

    ecl_algo_pid_init(&s_line_pid, 32.0f, 0.0f, 8.0f, -35.0f, 35.0f, 10.0f);

    while (true) {
        bool left = false;
        bool center = false;
        bool right = false;

        ESP_ERROR_CHECK(ecl_sensor_ir_line_sensor_read(&s_ir_left, &left));
        ESP_ERROR_CHECK(ecl_sensor_ir_line_sensor_read(&s_ir_center, &center));
        ESP_ERROR_CHECK(ecl_sensor_ir_line_sensor_read(&s_ir_right, &right));

        float position = line_position_from_sensors(left, center, right);
        float correction = -ecl_algo_pid_update(
            &s_line_pid, 0.0f, position, (float)CONTROL_DT_MS / 1000.0f);

        int8_t left_speed = clamp_speed_pct(BASE_SPEED_PCT + correction);
        int8_t right_speed = clamp_speed_pct(BASE_SPEED_PCT - correction);

        ESP_ERROR_CHECK(ecl_motor_control_set_speed_pct(&s_left_motor, left_speed));
        ESP_ERROR_CHECK(ecl_motor_control_set_speed_pct(&s_right_motor, right_speed));

        ESP_LOGI(TAG,
                 "IR LCR=%d%d%d pos=%.2f corr=%.1f speed L/R=%d/%d",
                 left, center, right, position, correction, left_speed, right_speed);

        vTaskDelay(pdMS_TO_TICKS(CONTROL_DT_MS));
    }
}
