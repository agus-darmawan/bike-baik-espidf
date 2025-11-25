#include "sensor_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "=== Bike-Baik ESP-IDF Sensor System ===");
    ESP_LOGI(TAG, "Initializing all sensors...");
    
    sensor_manager_init();
    sensor_manager_start_tasks();

    ESP_LOGI(TAG, "System started, reading sensors...");
    ESP_LOGI(TAG, "");

    while (1) {
        sensor_data_t d = sensor_manager_get_data();
        
        ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
        ESP_LOGI(TAG, "║      BIKE-BAIK SENSOR READINGS       ║");
        ESP_LOGI(TAG, "╠════════════════════════════════════════╣");
        ESP_LOGI(TAG, "║ Temperature  : %6.2f °C             ║", d.temperature_c);
        ESP_LOGI(TAG, "║ Wheel Count  : %6u pulses          ║", d.wheel_count);
        ESP_LOGI(TAG, "║ RPM          : %6.1f rpm             ║", d.rpm);
        ESP_LOGI(TAG, "╠════════════════════════════════════════╣");
        ESP_LOGI(TAG, "║ Accel X      : %7.3f m/s²          ║", d.accel_x);
        ESP_LOGI(TAG, "║ Accel Y      : %7.3f m/s²          ║", d.accel_y);
        ESP_LOGI(TAG, "║ Accel Z      : %7.3f m/s²          ║", d.accel_z);
        ESP_LOGI(TAG, "║ Gyro X       : %7.3f rad/s         ║", d.gyro_x);
        ESP_LOGI(TAG, "║ Gyro Y       : %7.3f rad/s         ║", d.gyro_y);
        ESP_LOGI(TAG, "║ Gyro Z       : %7.3f rad/s         ║", d.gyro_z);
        ESP_LOGI(TAG, "╠════════════════════════════════════════╣");
        ESP_LOGI(TAG, "║ Roll         : %7.2f°              ║", d.roll);
        ESP_LOGI(TAG, "║ Pitch        : %7.2f°              ║", d.pitch);
        ESP_LOGI(TAG, "║ Yaw          : %7.2f°              ║", d.yaw);
        ESP_LOGI(TAG, "╠════════════════════════════════════════╣");
        ESP_LOGI(TAG, "║ Digital In   : %s                   ║", d.digital_input_state ? "HIGH" : "LOW ");
        ESP_LOGI(TAG, "║ Input Changes: %6u times           ║", d.digital_change_count);
        ESP_LOGI(TAG, "║ Battery Volt : %6.2f V              ║", d.battery_voltage);
        ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
        ESP_LOGI(TAG, "");
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}