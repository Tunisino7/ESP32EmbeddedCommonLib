/*
 * Odometry example
 *
 * Drives two encoder motors forward and updates differential-drive odometry
 * from the encoder delta ticks at a fixed interval.
 */

#include "ESP32EmbeddedCommonLib/algo/odometry.h"
#include "ESP32EmbeddedCommonLib/motor/dc_motor_encoder.h"

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_LEFT_IN1    GPIO_NUM_25
#define PIN_LEFT_IN2    GPIO_NUM_26
#define PIN_RIGHT_IN1   GPIO_NUM_27
#define PIN_RIGHT_IN2   GPIO_NUM_14

#define PIN_LEFT_ENC_A  GPIO_NUM_32
#define PIN_LEFT_ENC_B  GPIO_NUM_33
#define PIN_RIGHT_ENC_A GPIO_NUM_34
#define PIN_RIGHT_ENC_B GPIO_NUM_35

#define GEAR_RATIO      100U
#define WHEEL_RADIUS_M  0.0325f
#define WHEEL_BASE_M    0.145f
#define LOOP_MS         100U

static const char *TAG = "odometry";

static ecl_dc_motor_encoder_t s_left_motor;
static ecl_dc_motor_encoder_t s_right_motor;
static ecl_odometry_t s_odom;
static int64_t s_prev_left_pulses;
static int64_t s_prev_right_pulses;

static uint32_t ticks_per_output_rev(const ecl_dc_motor_encoder_t *motor)
{
    uint32_t decode_multiplier =
        (motor->config.pin_enc_b == GPIO_NUM_NC) ? 2U : 4U;
    return (uint32_t)motor->config.pulses_per_rev
        * decode_multiplier
        * (uint32_t)motor->config.gear_ratio;
}

static void init_motors(void)
{
    ecl_dc_motor_encoder_config_t left_cfg =
        ecl_dc_motor_encoder_default_config(
            PIN_LEFT_IN1, PIN_LEFT_IN2, PIN_LEFT_IN1,
            PIN_LEFT_ENC_A, PIN_LEFT_ENC_B);
    left_cfg.motor.ledc_timer = LEDC_TIMER_2;
    left_cfg.motor.ledc_channel = LEDC_CHANNEL_0;
    left_cfg.gear_ratio = GEAR_RATIO;

    ecl_dc_motor_encoder_config_t right_cfg =
        ecl_dc_motor_encoder_default_config(
            PIN_RIGHT_IN1, PIN_RIGHT_IN2, PIN_RIGHT_IN1,
            PIN_RIGHT_ENC_A, PIN_RIGHT_ENC_B);
    right_cfg.motor.ledc_timer = LEDC_TIMER_2;
    right_cfg.motor.ledc_channel = LEDC_CHANNEL_2;
    right_cfg.gear_ratio = GEAR_RATIO;

    ESP_ERROR_CHECK(ecl_dc_motor_encoder_init(&s_left_motor, &left_cfg));
    ESP_ERROR_CHECK(ecl_dc_motor_encoder_init(&s_right_motor, &right_cfg));
}

void app_main(void)
{
    init_motors();

    const ecl_odometry_config_t odom_cfg = {
        .wheel_radius_m = WHEEL_RADIUS_M,
        .wheel_base_m = WHEEL_BASE_M,
    };
    ecl_algo_odometry_init(&s_odom, &odom_cfg);

    ESP_ERROR_CHECK(ecl_dc_motor_encoder_get_pulses(&s_left_motor, &s_prev_left_pulses));
    ESP_ERROR_CHECK(ecl_dc_motor_encoder_get_pulses(&s_right_motor, &s_prev_right_pulses));

    ESP_ERROR_CHECK(ecl_dc_motor_encoder_set_speed(&s_left_motor, 35));
    ESP_ERROR_CHECK(ecl_dc_motor_encoder_set_speed(&s_right_motor, 35));

    while (true) {
        int64_t left_pulses = 0;
        int64_t right_pulses = 0;
        ESP_ERROR_CHECK(ecl_dc_motor_encoder_get_pulses(&s_left_motor, &left_pulses));
        ESP_ERROR_CHECK(ecl_dc_motor_encoder_get_pulses(&s_right_motor, &right_pulses));

        int32_t delta_left = (int32_t)(left_pulses - s_prev_left_pulses);
        int32_t delta_right = (int32_t)(right_pulses - s_prev_right_pulses);
        s_prev_left_pulses = left_pulses;
        s_prev_right_pulses = right_pulses;

        ecl_algo_odometry_update(
            &s_odom, delta_left, delta_right, ticks_per_output_rev(&s_left_motor));

        ecl_pose_t pose = {0};
        ecl_algo_odometry_get_pose(&s_odom, &pose);

        ESP_LOGI(TAG, "delta L/R=%ld/%ld pose x=%.3f y=%.3f theta=%.3f",
                 (long)delta_left, (long)delta_right,
                 pose.x_m, pose.y_m, pose.theta_rad);

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}
