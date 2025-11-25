#include "sensor_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    sensor_manager_init();
    sensor_manager_start_tasks();

    while (1) {
        sensor_data_t d = sensor_manager_get_data();
        ESP_LOGI(TAG, "Temp: %.2f C | Count: %u | RPM: %.2f", d.temperature_c, d.wheel_count, d.rpm);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
