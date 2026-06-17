/*
 * Filter example
 *
 * Smooths a front ultrasonic range reading with the moving-average and low-pass
 * filters, then slows or stops a two-motor drive as obstacles get closer.
 */

#include "ESP32EmbeddedCommonLib/algo/filter.h"
#include "ESP32EmbeddedCommonLib/driver/drv8833.h"
#include "ESP32EmbeddedCommonLib/motor/motor_control.h"
#include "ESP32EmbeddedCommonLib/sensor/ultrasonic_sensor.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_AIN1       GPIO_NUM_25
#define PIN_AIN2       GPIO_NUM_26
#define PIN_BIN1       GPIO_NUM_27
#define PIN_BIN2       GPIO_NUM_14
#define PIN_US_TRIG    GPIO_NUM_5
#define PIN_US_ECHO    GPIO_NUM_18

#define LOOP_MS        100U
#define STOP_CM        15.0f
#define SLOW_CM        60.0f
#define CRUISE_PCT     45.0f

static const char *TAG = "filter_speed";

static ecl_drv8833_t s_bridge;
static ecl_drv8833_hbridge_ctx_t s_left_ctx;
static ecl_drv8833_hbridge_ctx_t s_right_ctx;
static ecl_hbridge_t s_left_hbridge;
static ecl_hbridge_t s_right_hbridge;
static ecl_motor_control_t s_left_motor;
static ecl_motor_control_t s_right_motor;
static ecl_ultrasonic_sensor_t s_front_us;
static ecl_moving_avg_t s_avg;
static ecl_lpf_t s_lpf;

static int8_t speed_from_distance(float distance_cm)
{
    if (distance_cm <= STOP_CM) {
        return 0;
    }
    if (distance_cm >= SLOW_CM) {
        return (int8_t)CRUISE_PCT;
    }

    float ratio = (distance_cm - STOP_CM) / (SLOW_CM - STOP_CM);
    return (int8_t)(ratio * CRUISE_PCT);
}

static void init_drive(void)
{
    ecl_drv8833_config_t bridge_cfg =
        ecl_driver_drv8833_default_config(PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2);

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

void app_main(void)
{
    init_drive();

    ecl_ultrasonic_sensor_config_t us_cfg =
        ecl_sensor_ultrasonic_sensor_default_config(PIN_US_TRIG, PIN_US_ECHO);
    ESP_ERROR_CHECK(ecl_sensor_ultrasonic_sensor_init(&s_front_us, &us_cfg));

    ecl_algo_filter_moving_avg_init(&s_avg, 5);
    ecl_algo_filter_lpf_init(&s_lpf, 0.35f, SLOW_CM);

    while (true) {
        float raw_cm = SLOW_CM;
        esp_err_t err = ecl_sensor_ultrasonic_sensor_measure_distance_cm(&s_front_us, &raw_cm);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Distance read failed: %s", esp_err_to_name(err));
            raw_cm = STOP_CM;
        }

        float avg_cm = ecl_algo_filter_moving_avg_update(&s_avg, raw_cm);
        float filtered_cm = ecl_algo_filter_lpf_update(&s_lpf, avg_cm);
        int8_t speed = speed_from_distance(filtered_cm);

        ESP_ERROR_CHECK(ecl_motor_control_set_speed_pct(&s_left_motor, speed));
        ESP_ERROR_CHECK(ecl_motor_control_set_speed_pct(&s_right_motor, speed));

        ESP_LOGI(TAG, "distance raw/avg/lpf=%.1f/%.1f/%.1f cm speed=%d%%",
                 raw_cm, avg_cm, filtered_cm, speed);

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}
