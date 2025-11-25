#include "sensor_manager.h"
#include "sensor_tasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "max6675.h"
#include "config.h"

static const char *TAG = "sensor_manager";

static SemaphoreHandle_t spi_sem = NULL;
static SemaphoreHandle_t data_mutex = NULL;

static sensor_data_t g_data = {
    .temperature_c = -1000.0f,
    .wheel_count = 0,
    .rpm = 0.0f
};

void sensor_manager_init(void)
{
    ESP_LOGI(TAG, "sensor_manager init");

    spi_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(spi_sem);

    data_mutex = xSemaphoreCreateMutex();

    esp_err_t r = max6675_init(
        MAX6675_SPI_HOST,
        MAX6675_CLK_PIN,
        MAX6675_MISO_PIN,
        MAX6675_CS_PIN,
        MAX6675_SPI_HZ,
        spi_sem
    );

    if (r != ESP_OK) {
        ESP_LOGW(TAG, "max6675_init failed (%d)", r);
    }
}

void sensor_manager_start_tasks(void)
{
    sensor_tasks_start();
}

void sensor_manager_update(float temp_or_nan, uint32_t count, float rpm)
{
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(20))) {
        if (temp_or_nan > -500.0f) g_data.temperature_c = temp_or_nan;
        xSemaphoreGive(data_mutex);
    }
}

sensor_data_t sensor_manager_get_data(void)
{
    sensor_data_t out = g_data;
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(20))) {
        out = g_data;
        xSemaphoreGive(data_mutex);
    }
    return out;
}
