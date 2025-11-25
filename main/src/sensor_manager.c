#include "sensor_manager.h"
#include "sensor_tasks.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "max6675.h"
#include "prox_sensor.h"
#include "mpu6050.h"
#include "digital_input.h"
#include "voltage_sensor.h"

static const char *TAG = "sensor_manager";

static SemaphoreHandle_t spi_sem = NULL;
static SemaphoreHandle_t data_mutex = NULL;

static sensor_data_t g_data = {
    .temperature_c = -1000.0f,
    .wheel_count = 0,
    .rpm = 0.0f,
    .accel_x = 0.0f,
    .accel_y = 0.0f,
    .accel_z = 0.0f,
    .gyro_x = 0.0f,
    .gyro_y = 0.0f,
    .gyro_z = 0.0f,
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = 0.0f,
    .digital_input_state = false,
    .digital_change_count = 0,
    .battery_voltage = 0.0f
};

void sensor_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing sensor manager");

    // Create semaphores
    spi_sem = xSemaphoreCreateBinary();
    configASSERT(spi_sem);
    xSemaphoreGive(spi_sem);

    data_mutex = xSemaphoreCreateMutex();
    configASSERT(data_mutex);

    // Initialize MAX6675
    esp_err_t r = max6675_init(
        MAX6675_SPI_HOST,
        MAX6675_CLK_PIN,
        MAX6675_MISO_PIN,
        MAX6675_CS_PIN,
        MAX6675_SPI_HZ,
        spi_sem
    );
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "MAX6675 init failed (%d)", r);
    } else {
        ESP_LOGI(TAG, "MAX6675 initialized");
    }

    // Initialize proximity sensor
    proximity_config_t prox_cfg = {
        .pin = PROX_PIN,
        .trigger = PROX_TRIGGER,
        .debounce_us = PROX_DEBOUNCE_US
    };
    r = proximity_init(&prox_cfg);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "Proximity sensor init failed (%d)", r);
    } else {
        ESP_LOGI(TAG, "Proximity sensor initialized");
    }

    // Initialize MPU6050
    r = mpu6050_init(MPU6050_I2C_PORT, MPU6050_SDA_PIN, MPU6050_SCL_PIN, MPU6050_ADDR);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 init failed (%d)", r);
    } else {
        ESP_LOGI(TAG, "MPU6050 initialized");
        
        // Optional: calibrate gyro at startup
        mpu6050_gyro_calib_t calib;
        ESP_LOGI(TAG, "Calibrating gyro... keep device still");
        mpu6050_calibrate_gyro(&calib, 100, 10);
    }

    // Initialize digital input
    digital_input_config_t din_cfg = {
        .pin = DIGITAL_INPUT_PIN,
        .pull_mode = DIGITAL_INPUT_PULL,
        .debounce_ms = DIGITAL_DEBOUNCE_MS
    };
    r = digital_input_init(&din_cfg);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "Digital input init failed (%d)", r);
    } else {
        ESP_LOGI(TAG, "Digital input initialized");
    }

    // Initialize voltage sensor
    voltage_sensor_config_t vsens_cfg = {
        .channel = VOLTAGE_ADC_CHAN,
        .unit = VOLTAGE_ADC_UNIT,
        .attenuation = VOLTAGE_ADC_ATTEN,
        .r1 = VOLTAGE_R1,
        .r2 = VOLTAGE_R2,
        .samples = VOLTAGE_SAMPLES
    };
    r = voltage_sensor_init(&vsens_cfg);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "Voltage sensor init failed (%d)", r);
    } else {
        ESP_LOGI(TAG, "Voltage sensor initialized");
    }

    ESP_LOGI(TAG, "Sensor manager initialization complete");
}

void sensor_manager_start_tasks(void)
{
    sensor_tasks_start();
}

void sensor_manager_update_temp(float temp_c)
{
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(20))) {
        g_data.temperature_c = temp_c;
        xSemaphoreGive(data_mutex);
    }
}

void sensor_manager_update_prox(uint32_t count, float rpm)
{
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(20))) {
        g_data.wheel_count = count;
        g_data.rpm = rpm;
        xSemaphoreGive(data_mutex);
    }
}

void sensor_manager_update_imu(float ax, float ay, float az, float gx, float gy, float gz)
{
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(20))) {
        g_data.accel_x = ax;
        g_data.accel_y = ay;
        g_data.accel_z = az;
        g_data.gyro_x = gx;
        g_data.gyro_y = gy;
        g_data.gyro_z = gz;
        xSemaphoreGive(data_mutex);
    }
}

void sensor_manager_update_orientation(float roll, float pitch, float yaw)
{
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(20))) {
        g_data.roll = roll;
        g_data.pitch = pitch;
        g_data.yaw = yaw;
        xSemaphoreGive(data_mutex);
    }
}

void sensor_manager_update_digital(bool state, uint32_t changes)
{
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(20))) {
        g_data.digital_input_state = state;
        g_data.digital_change_count = changes;
        xSemaphoreGive(data_mutex);
    }
}

void sensor_manager_update_voltage(float volts)
{
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(20))) {
        g_data.battery_voltage = volts;
        xSemaphoreGive(data_mutex);
    }
}

sensor_data_t sensor_manager_get_data(void)
{
    sensor_data_t out;
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(20))) {
        out = g_data;
        xSemaphoreGive(data_mutex);
    } else {
        out = g_data;
    }
    return out;
}