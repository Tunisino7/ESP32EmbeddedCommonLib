/*
 * Maze solver example
 *
 * Reads left, front, and right ultrasonic sensors as wall detectors, asks the
 * wall-following maze solver for the next turn, and drives a differential
 * platform with a DRV8833.
 */

#include "ESP32EmbeddedCommonLib/algo/maze_solver.h"
#include "ESP32EmbeddedCommonLib/driver/drv8833.h"
#include "ESP32EmbeddedCommonLib/motor/motor_control.h"
#include "ESP32EmbeddedCommonLib/sensor/ultrasonic_sensor.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_AIN1          GPIO_NUM_25
#define PIN_AIN2          GPIO_NUM_26
#define PIN_BIN1          GPIO_NUM_27
#define PIN_BIN2          GPIO_NUM_14
#define PIN_NSLEEP        GPIO_NUM_12
#define PIN_NFAULT        GPIO_NUM_13

#define PIN_US_LEFT_TRIG  GPIO_NUM_4
#define PIN_US_LEFT_ECHO  GPIO_NUM_16
#define PIN_US_FRONT_TRIG GPIO_NUM_5
#define PIN_US_FRONT_ECHO GPIO_NUM_18
#define PIN_US_RIGHT_TRIG GPIO_NUM_19
#define PIN_US_RIGHT_ECHO GPIO_NUM_21

#define WALL_CM           18.0f
#define DRIVE_SPEED_PCT   35
#define TURN_SPEED_PCT    32
#define CELL_DRIVE_MS     450U
#define TURN_90_MS        360U
#define TURN_180_MS       (2U * TURN_90_MS)

static const char *TAG = "maze";

static ecl_drv8833_t s_bridge;
static ecl_drv8833_hbridge_ctx_t s_left_ctx;
static ecl_drv8833_hbridge_ctx_t s_right_ctx;
static ecl_hbridge_t s_left_hbridge;
static ecl_hbridge_t s_right_hbridge;
static ecl_motor_control_t s_left_motor;
static ecl_motor_control_t s_right_motor;
static ecl_ultrasonic_sensor_t s_left_us;
static ecl_ultrasonic_sensor_t s_front_us;
static ecl_ultrasonic_sensor_t s_right_us;
static ecl_maze_solver_t s_solver;

static void set_drive(int8_t left, int8_t right)
{
    ESP_ERROR_CHECK(ecl_motor_control_set_speed_pct(&s_left_motor, left));
    ESP_ERROR_CHECK(ecl_motor_control_set_speed_pct(&s_right_motor, right));
}

static void stop_drive(void)
{
    ESP_ERROR_CHECK(ecl_motor_control_stop(&s_left_motor));
    ESP_ERROR_CHECK(ecl_motor_control_stop(&s_right_motor));
}

static bool is_wall_detected(ecl_ultrasonic_sensor_t *sensor)
{
    float cm = 0.0f;
    esp_err_t err = ecl_ultrasonic_sensor_measure_distance_cm(sensor, &cm);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Ultrasonic read failed: %s", esp_err_to_name(err));
        return true;
    }
    return cm < WALL_CM;
}

static void execute_turn(ecl_maze_turn_t turn)
{
    switch (turn) {
        case ECL_MAZE_TURN_LEFT:
            set_drive(-TURN_SPEED_PCT, TURN_SPEED_PCT);
            vTaskDelay(pdMS_TO_TICKS(TURN_90_MS));
            break;
        case ECL_MAZE_TURN_RIGHT:
            set_drive(TURN_SPEED_PCT, -TURN_SPEED_PCT);
            vTaskDelay(pdMS_TO_TICKS(TURN_90_MS));
            break;
        case ECL_MAZE_TURN_AROUND:
            set_drive(TURN_SPEED_PCT, -TURN_SPEED_PCT);
            vTaskDelay(pdMS_TO_TICKS(TURN_180_MS));
            break;
        case ECL_MAZE_TURN_NONE:
        default:
            break;
    }
    stop_drive();
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void drive_one_cell(void)
{
    set_drive(DRIVE_SPEED_PCT, DRIVE_SPEED_PCT);
    vTaskDelay(pdMS_TO_TICKS(CELL_DRIVE_MS));
    stop_drive();
}

static void init_drive(void)
{
    ecl_drv8833_config_t bridge_cfg =
        ecl_drv8833_default_config(PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2);
    bridge_cfg.pin_nsleep = PIN_NSLEEP;
    bridge_cfg.pin_nfault = PIN_NFAULT;

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

static void init_ultrasonic(void)
{
    ecl_ultrasonic_sensor_config_t left_cfg =
        ecl_ultrasonic_sensor_default_config(PIN_US_LEFT_TRIG, PIN_US_LEFT_ECHO);
    ecl_ultrasonic_sensor_config_t front_cfg =
        ecl_ultrasonic_sensor_default_config(PIN_US_FRONT_TRIG, PIN_US_FRONT_ECHO);
    ecl_ultrasonic_sensor_config_t right_cfg =
        ecl_ultrasonic_sensor_default_config(PIN_US_RIGHT_TRIG, PIN_US_RIGHT_ECHO);

    ESP_ERROR_CHECK(ecl_ultrasonic_sensor_init(&s_left_us, &left_cfg));
    ESP_ERROR_CHECK(ecl_ultrasonic_sensor_init(&s_front_us, &front_cfg));
    ESP_ERROR_CHECK(ecl_ultrasonic_sensor_init(&s_right_us, &right_cfg));
}

void app_main(void)
{
    init_drive();
    init_ultrasonic();
    ecl_algo_maze_solver_init(&s_solver, ECL_MAZE_FOLLOW_LEFT, ECL_MAZE_NORTH);

    while (true) {
        bool wall_left = is_wall_detected(&s_left_us);
        bool wall_front = is_wall_detected(&s_front_us);
        bool wall_right = is_wall_detected(&s_right_us);

        ecl_maze_turn_t turn =
            ecl_algo_maze_solver_next_turn(&s_solver, wall_left, wall_front, wall_right);

        ESP_LOGI(TAG, "walls L/F/R=%d/%d/%d turn=%d heading=%d",
                 wall_left, wall_front, wall_right, turn,
                 ecl_algo_maze_solver_get_heading(&s_solver));

        execute_turn(turn);
        ecl_algo_maze_solver_apply_turn(&s_solver, turn);
        ESP_LOGI(TAG, "new heading=%d", ecl_algo_maze_solver_get_heading(&s_solver));

        drive_one_cell();
    }
}
