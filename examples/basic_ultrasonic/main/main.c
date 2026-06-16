#include "ESP32EmbeddedCommonLib/ultrasonic_sensor.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
    static const char *TAG = "basic_ultrasonic";

    esp32_common_ultrasonic_sensor_config_t config =
        esp32_common_ultrasonic_sensor_default_config(GPIO_NUM_5, GPIO_NUM_18);

    esp32_common_ultrasonic_sensor_t sensor = {0};
    ESP_ERROR_CHECK(esp32_common_ultrasonic_sensor_init(&sensor, &config));

    while (true) {
        float distance_cm = 0.0f;
        esp_err_t err = esp32_common_ultrasonic_sensor_measure_distance_cm(&sensor, &distance_cm);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Distance: %.2f cm", distance_cm);
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Ultrasonic measurement timed out");
        } else {
            ESP_LOGE(TAG, "Ultrasonic measurement failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
