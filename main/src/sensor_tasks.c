#include "sensor_tasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max6675.h"
#include "prox_sensor.h"
#include "sensor_manager.h"
#include "esp_log.h"

static const char *TAG = "sensor_tasks";

/* proximity task: compute delta each second -> RPM */
static void prox_task(void *arg)
{
    uint32_t last = proximity_get_count();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t now = proximity_get_count();
        uint32_t delta = now - last;
        last = now;
        float rpm = (delta / 1.0f) * 60.0f; /* pulses/sec -> RPM */
        sensor_manager_update(-1000.0f, now, rpm);
    }
}

/* max6675 task: read temperature periodically */
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
    xTaskCreate(prox_task, "prox_task", 2048, NULL, 10, NULL);
    xTaskCreate(max_task,  "max_task",  4096, NULL, 5, NULL);
}
