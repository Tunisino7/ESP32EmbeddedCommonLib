/*
 * Motion profile example
 *
 * Ramps a differential-drive robot between target linear velocities and sends
 * the smoothed m/s command to both motors.
 */

#include "ESP32EmbeddedCommonLib/algo/motion_profile.h"
#include "ESP32EmbeddedCommonLib/driver/drv8833.h"
#include "ESP32EmbeddedCommonLib/motor/motor_control.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_AIN1       GPIO_NUM_25
#define PIN_AIN2       GPIO_NUM_26
#define PIN_BIN1       GPIO_NUM_27
#define PIN_BIN2       GPIO_NUM_14

#define LOOP_MS        20U
#define CRUISE_MS      2500U

static const char *TAG = "motion_profile";

static ecl_drv8833_t s_bridge;
static ecl_drv8833_hbridge_ctx_t s_left_ctx;
static ecl_drv8833_hbridge_ctx_t s_right_ctx;
static ecl_hbridge_t s_left_hbridge;
static ecl_hbridge_t s_right_hbridge;
static ecl_motor_control_t s_left_motor;
static ecl_motor_control_t s_right_motor;
static ecl_motion_profile_t s_profile;

static void init_drive(void)
{
    ecl_drv8833_config_t bridge_cfg =
        ecl_drv8833_default_config(PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2);

    ESP_ERROR_CHECK(ecl_drv8833_init(&s_bridge, &bridge_cfg));
    ESP_ERROR_CHECK(ecl_drv8833_bind_hbridge(
        &s_bridge, ECL_DRV8833_CHANNEL_A, &s_left_ctx, &s_left_hbridge));
    ESP_ERROR_CHECK(ecl_drv8833_bind_hbridge(
        &s_bridge, ECL_DRV8833_CHANNEL_B, &s_right_ctx, &s_right_hbridge));

    const ecl_motor_control_config_t motor_cfg = {
        .rpm_max = 300.0f,
        .wheel_radius_m = 0.0325f,
    };
    ESP_ERROR_CHECK(ecl_motor_control_init(&s_left_motor, &s_left_hbridge, &motor_cfg));
    ESP_ERROR_CHECK(ecl_motor_control_init(&s_right_motor, &s_right_hbridge, &motor_cfg));
}

static void run_target(float target_ms)
{
    ecl_algo_motion_profile_set_target(&s_profile, target_ms);

    for (uint32_t elapsed = 0; elapsed < CRUISE_MS; elapsed += LOOP_MS) {
        float velocity_ms =
            ecl_algo_motion_profile_update(&s_profile, (float)LOOP_MS / 1000.0f);

        ESP_ERROR_CHECK(ecl_motor_control_set_speed_ms(&s_left_motor, velocity_ms));
        ESP_ERROR_CHECK(ecl_motor_control_set_speed_ms(&s_right_motor, velocity_ms));

        ESP_LOGI(TAG, "target=%.2f m/s command=%.2f m/s settled=%d",
                 target_ms, velocity_ms, ecl_algo_motion_profile_is_settled(&s_profile));

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}

void app_main(void)
{
    init_drive();
    ecl_algo_motion_profile_init(&s_profile, 0.35f, 0.50f, 0.0f);

    while (true) {
        run_target(0.35f);
        run_target(0.0f);
        run_target(-0.20f);
        run_target(0.0f);
    }
}
