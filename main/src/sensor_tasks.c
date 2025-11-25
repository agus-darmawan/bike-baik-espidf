#include "sensor_tasks.h"
#include "sensor_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "max6675.h"
#include "prox_sensor.h"
#include "mpu6050.h"
#include "orientation.h"
#include "digital_input.h"
#include "voltage_sensor.h"

static const char *TAG = "sensor_tasks";

/**
 * @brief Task to read MAX6675 temperature sensor
 */
static void temp_task(void *arg)
{
    ESP_LOGI(TAG, "Temperature task started");
    while (1) {
        float t = max6675_read_temperature();
        if (t > -500.0f) {
            sensor_manager_update_temp(t);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief Task to compute RPM from proximity sensor
 */
static void prox_task(void *arg)
{
    ESP_LOGI(TAG, "Proximity task started");
    uint32_t last = proximity_get_count();
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t now = proximity_get_count();
        uint32_t delta = now - last;
        last = now;
        
        // Convert pulses/sec to RPM (1 pulse = 1 revolution)
        float rpm = (delta / 1.0f) * 60.0f;
        sensor_manager_update_prox(now, rpm);
    }
}

/**
 * @brief Task to read MPU6050 IMU data
 */
static void imu_task(void *arg)
{
    ESP_LOGI(TAG, "IMU task started");
    mpu6050_data_t data;
    
    // Initialize Mahony filter (can also use Madgwick)
    mahony_filter_t filter;
    mahony_init(&filter, 2.0f, 0.0f); // kp=2.0, ki=0.0 (no drift correction)
    
    // OR use Madgwick:
    // madgwick_filter_t filter;
    // madgwick_init(&filter, 0.3f); // beta=0.3
    
    float dt = 0.05f; // 50ms = 20Hz
    float R[3][3]; // Rotation matrix
    float roll, pitch, yaw;
    
    while (1) {
        if (mpu6050_read(&data) == ESP_OK) {
            // Update raw sensor readings
            sensor_manager_update_imu(
                data.ax, data.ay, data.az,
                data.gx, data.gy, data.gz
            );
            
            // Update orientation filter
            mahony_update(&filter, 
                         data.gx, data.gy, data.gz,  // gyro in rad/s
                         data.ax, data.ay, data.az,  // accel in m/s^2
                         dt, R);
            
            // OR use Madgwick:
            // madgwick_update(&filter, 
            //                data.gx, data.gy, data.gz,
            //                data.ax, data.ay, data.az,
            //                dt, R);
            
            // Get Roll, Pitch, Yaw from rotation matrix
            mahony_get_rpy(R, &roll, &pitch, &yaw);
            // OR: madgwick_get_rpy(R, &roll, &pitch, &yaw);
            
            // Update orientation
            sensor_manager_update_orientation(roll, pitch, yaw);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz update rate
    }
}

/**
 * @brief Task to read digital input state
 */
static void digital_input_task(void *arg)
{
    ESP_LOGI(TAG, "Digital input task started");
    
    while (1) {
        bool state = digital_input_read();
        uint32_t changes = digital_input_get_change_count();
        sensor_manager_update_digital(state, changes);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Task to read voltage sensor
 */
static void voltage_task(void *arg)
{
    ESP_LOGI(TAG, "Voltage task started");
    
    while (1) {
        float voltage = voltage_sensor_read();
        if (voltage >= 0.0f) {
            sensor_manager_update_voltage(voltage);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1Hz update rate
    }
}

void sensor_tasks_start(void)
{
    ESP_LOGI(TAG, "Starting sensor tasks");
    
    xTaskCreate(temp_task, "temp_task", 4096, NULL, 5, NULL);
    xTaskCreate(prox_task, "prox_task", 2048, NULL, 10, NULL);
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 8, NULL);
    xTaskCreate(digital_input_task, "din_task", 2048, NULL, 6, NULL);
    xTaskCreate(voltage_task, "voltage_task", 3072, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "All sensor tasks created");
}