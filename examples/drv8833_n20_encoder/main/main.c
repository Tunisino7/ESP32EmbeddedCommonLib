/*
 * drv8833_n20_encoder — DRV8833 H-bridge driving two N20 DC motors
 *                       with quadrature Hall-effect encoders.
 *
 * Wiring (adjust GPIO numbers to match your board):
 *
 *   DRV8833 pin  │  ESP32 GPIO  │  Description
 *   ─────────────┼──────────────┼──────────────────────────────
 *   AIN1         │  GPIO 25     │  Motor A direction bit 1
 *   AIN2         │  GPIO 26     │  Motor A direction bit 2
 *   BIN1         │  GPIO 27     │  Motor B direction bit 1
 *   BIN2         │  GPIO 14     │  Motor B direction bit 2
 *   nSLEEP       │  GPIO 12     │  Chip enable (HIGH = awake)
 *   nFAULT       │  GPIO 13     │  Fault indicator (LOW = fault)
 *
 *   Motor A encoder:
 *   ENC_A        │  GPIO 32     │  Channel A (both edges counted)
 *   ENC_B        │  GPIO 33     │  Channel B (quadrature 4× mode)
 *
 *   Motor B encoder:
 *   ENC_A        │  GPIO 34     │  Channel A
 *   ENC_B        │  GPIO 35     │  Channel B
 *
 * N20 motor specs used in this example:
 *   PPR (pre-gearbox): 7
 *   Gear ratio:       100  → change MOTOR_GEAR_RATIO below to match your variant
 *
 * The example runs a simple sequence inside a FreeRTOS task:
 *   1. Accelerate motor A to +60 %, motor B to +60 %.
 *   2. Print RPM and pulse count every 100 ms for 2 seconds.
 *   3. Reverse both motors to -40 %.
 *   4. Print again for 2 seconds.
 *   5. Stop both motors.
 *   6. Print final pulse counts and loop.
 */

#include "ESP32EmbeddedCommonLib/driver/drv8833.h"
#include "ESP32EmbeddedCommonLib/motor/dc_motor_encoder.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ── GPIO assignments ────────────────────────────────────────────────────── */
#define PIN_AIN1      GPIO_NUM_25
#define PIN_AIN2      GPIO_NUM_26
#define PIN_BIN1      GPIO_NUM_27
#define PIN_BIN2      GPIO_NUM_14
#define PIN_NSLEEP    GPIO_NUM_12
#define PIN_NFAULT    GPIO_NUM_13

#define PIN_ENC_A_CH_A  GPIO_NUM_32
#define PIN_ENC_B_CH_A  GPIO_NUM_33   /* set to GPIO_NUM_NC for 2× mode */
#define PIN_ENC_A_CH_B  GPIO_NUM_34
#define PIN_ENC_B_CH_B  GPIO_NUM_35   /* set to GPIO_NUM_NC for 2× mode */

/* ── Motor parameters ────────────────────────────────────────────────────── */
#define MOTOR_GEAR_RATIO  100U        /* change to 15, 25, 50, 150, 300 … */

/* ── Task parameters ─────────────────────────────────────────────────────── */
#define MOTOR_TASK_STACK_SIZE  4096U
#define MOTOR_TASK_PRIORITY    5U
#define RPM_PRINT_PERIOD_MS    100U

static const char *TAG = "drv8833_n20_encoder";

/* ── Static driver/motor instances ──────────────────────────────────────── */
static ecl_drv8833_t              s_bridge;
static ecl_dc_motor_encoder_t     s_motor_a;
static ecl_dc_motor_encoder_t     s_motor_b;

/* ── Helper: log RPM + pulses for both motors ────────────────────────────── */
static void log_motors(void)
{
    float   rpm_a = 0.0f, rpm_b = 0.0f;
    int64_t pulses_a = 0,  pulses_b = 0;

    ecl_dc_motor_encoder_get_rpm(&s_motor_a, &rpm_a);
    ecl_dc_motor_encoder_get_rpm(&s_motor_b, &rpm_b);
    ecl_dc_motor_encoder_get_pulses(&s_motor_a, &pulses_a);
    ecl_dc_motor_encoder_get_pulses(&s_motor_b, &pulses_b);

    ESP_LOGI(TAG,
             "A → RPM: %6.1f  pulses: %lld  |  B → RPM: %6.1f  pulses: %lld",
             rpm_a, pulses_a, rpm_b, pulses_b);
}

/* ── FreeRTOS motor demo task ─────────────────────────────────────────────── */
static void motor_task(void *arg)
{
    (void)arg;

    /* ── 1. Forward at 60 % for 2 s ──────────────────────────────────────── */
    ESP_LOGI(TAG, "Forward 60%%");
    ESP_ERROR_CHECK(ecl_dc_motor_encoder_set_speed(&s_motor_a,  60));
    ESP_ERROR_CHECK(ecl_dc_motor_encoder_set_speed(&s_motor_b,  60));

    for (int i = 0; i < (2000 / RPM_PRINT_PERIOD_MS); i++) {
        vTaskDelay(pdMS_TO_TICKS(RPM_PRINT_PERIOD_MS));
        log_motors();
    }

    /* ── 2. Reverse at −40 % for 2 s ─────────────────────────────────────── */
    ESP_LOGI(TAG, "Reverse -40%%");
    ESP_ERROR_CHECK(ecl_dc_motor_encoder_set_speed(&s_motor_a, -40));
    ESP_ERROR_CHECK(ecl_dc_motor_encoder_set_speed(&s_motor_b, -40));

    for (int i = 0; i < (2000 / RPM_PRINT_PERIOD_MS); i++) {
        vTaskDelay(pdMS_TO_TICKS(RPM_PRINT_PERIOD_MS));
        log_motors();
    }

    /* ── 3. Stop ──────────────────────────────────────────────────────────── */
    ESP_LOGI(TAG, "Stop");
    ESP_ERROR_CHECK(ecl_dc_motor_encoder_stop(&s_motor_a));
    ESP_ERROR_CHECK(ecl_dc_motor_encoder_stop(&s_motor_b));

    int64_t pulses_a = 0, pulses_b = 0;
    ecl_dc_motor_encoder_get_pulses(&s_motor_a, &pulses_a);
    ecl_dc_motor_encoder_get_pulses(&s_motor_b, &pulses_b);
    ESP_LOGI(TAG, "Final pulses — A: %lld  B: %lld", pulses_a, pulses_b);

    /* Check for DRV8833 fault */
    bool fault = false;
    if (ecl_drv8833_is_fault(&s_bridge, &fault) == ESP_OK && fault) {
        ESP_LOGE(TAG, "DRV8833 nFAULT asserted — overcurrent or overtemperature!");
    }

    /* Reset counters and loop back */
    ESP_ERROR_CHECK(ecl_dc_motor_encoder_reset_count(&s_motor_a));
    ESP_ERROR_CHECK(ecl_dc_motor_encoder_reset_count(&s_motor_b));

    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Restart the task for a continuous loop */
    xTaskCreate(motor_task, "motor_task", MOTOR_TASK_STACK_SIZE,
                NULL, MOTOR_TASK_PRIORITY, NULL);
    vTaskDelete(NULL);
}

/* ── app_main: one-time initialisation ─────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "DRV8833 + N20 encoder example starting");

    /* ── Initialise DRV8833 bridge ─────────────────────────────────────────── */
    ecl_drv8833_config_t bridge_cfg =
        ecl_drv8833_default_config(PIN_AIN1, PIN_AIN2,
                                             PIN_BIN1, PIN_BIN2);
    bridge_cfg.pin_nsleep    = PIN_NSLEEP;
    bridge_cfg.pin_nfault    = PIN_NFAULT;
    bridge_cfg.slow_decay    = false;   /* fast decay: coast on stop */

    ESP_ERROR_CHECK(ecl_drv8833_init(&s_bridge, &bridge_cfg));
    ESP_LOGI(TAG, "DRV8833 initialised");

    /*
     * ── Initialise motor A (DRV8833 channel A) ─────────────────────────────
     *
     * The dc_motor_encoder driver manages its own LEDC resources independently
     * from the DRV8833 driver.  We wire the same physical INx GPIOs here.
     * LEDC_TIMER_1 / channels 0-1 are used for motor A.
     */
    ecl_dc_motor_encoder_config_t motor_a_cfg =
        ecl_dc_motor_encoder_default_config(
            PIN_AIN1, PIN_AIN2,
            bridge_cfg.channel[ECL_DRV8833_CHANNEL_A].pin_in1,  /* same pin, unused — see note below */
            PIN_ENC_A_CH_A, PIN_ENC_B_CH_A);

    /*
     * NOTE: The DRV8833 driver controls the GPIO direction/duty via LEDC
     * internally.  We use dc_motor_encoder for its encoder (PCNT) and speed
     * control (LEDC) on the same physical pins.  Do NOT call
     * ecl_drv8833_set_speed() and
     * ecl_dc_motor_encoder_set_speed() on the same channel at the
     * same time — use the encoder driver exclusively when encoders are present.
     *
     * Override LEDC resources so they don't collide with the drv8833 defaults:
     *   DRV8833 default: TIMER_1, CH_0..CH_3
     *   Motor A encoder: TIMER_2, CH_0..CH_1
     *   Motor B encoder: TIMER_2, CH_2..CH_3
     */
    motor_a_cfg.motor.pin_in1            = PIN_AIN1;
    motor_a_cfg.motor.pin_in2            = PIN_AIN2;
    motor_a_cfg.motor.ledc_timer         = LEDC_TIMER_2;
    motor_a_cfg.motor.ledc_channel       = LEDC_CHANNEL_0;
    motor_a_cfg.gear_ratio               = MOTOR_GEAR_RATIO;

    ESP_ERROR_CHECK(ecl_dc_motor_encoder_init(&s_motor_a, &motor_a_cfg));
    ESP_LOGI(TAG, "Motor A initialised (gear ratio %u)", MOTOR_GEAR_RATIO);

    /* ── Initialise motor B (DRV8833 channel B) ─────────────────────────────── */
    ecl_dc_motor_encoder_config_t motor_b_cfg =
        ecl_dc_motor_encoder_default_config(
            PIN_BIN1, PIN_BIN2,
            GPIO_NUM_NC,            /* pwm pin overridden below */
            PIN_ENC_A_CH_B, PIN_ENC_B_CH_B);

    motor_b_cfg.motor.pin_in1            = PIN_BIN1;
    motor_b_cfg.motor.pin_in2            = PIN_BIN2;
    motor_b_cfg.motor.pin_pwm            = PIN_BIN1;  /* DRV8833 has no EN; speed via INx */
    motor_b_cfg.motor.ledc_timer         = LEDC_TIMER_2;
    motor_b_cfg.motor.ledc_channel       = LEDC_CHANNEL_2;
    motor_b_cfg.gear_ratio               = MOTOR_GEAR_RATIO;

    ESP_ERROR_CHECK(ecl_dc_motor_encoder_init(&s_motor_b, &motor_b_cfg));
    ESP_LOGI(TAG, "Motor B initialised (gear ratio %u)", MOTOR_GEAR_RATIO);

    /* ── Start demo task ─────────────────────────────────────────────────────── */
    xTaskCreate(motor_task, "motor_task", MOTOR_TASK_STACK_SIZE,
                NULL, MOTOR_TASK_PRIORITY, NULL);
}
