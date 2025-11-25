#include "sensor_tasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max6675.h"
#include "sensor_manager.h"
#include "esp_log.h"

static const char *TAG = "sensor_tasks";

static void max_task(void *arg)
{
    while (1) {
        float t = max6675_read_temperature();
        if (t > -500.0f) {
            sensor_manager_update(t, 0, -1.0f);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void sensor_tasks_start(void)
{
    xTaskCreate(max_task, "max_task", 4096, NULL, 5, NULL);
}
