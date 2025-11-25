#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Aggregated sensor data structure
 */
typedef struct {
    /* MAX6675 Temperature */
    float temperature_c;
    
    /* Proximity sensor */
    uint32_t wheel_count;
    float rpm;
    
    /* MPU6050 IMU */
    float accel_x, accel_y, accel_z;  /* m/s^2 */
    float gyro_x, gyro_y, gyro_z;     /* rad/s */
    
    /* MPU6050 Orientation (from Mahony/Madgwick filter) */
    float roll, pitch, yaw;           /* degrees */
    
    /* Digital input */
    bool digital_input_state;
    uint32_t digital_change_count;
    
    /* Voltage sensor */
    float battery_voltage;
} sensor_data_t;

/**
 * @brief Initialize all sensors and RTOS primitives.
 * Call once at startup before starting tasks.
 */
void sensor_manager_init(void);

/**
 * @brief Start all sensor reading tasks.
 * Creates FreeRTOS tasks for periodic sensor updates.
 */
void sensor_manager_start_tasks(void);

/**
 * @brief Get thread-safe copy of current sensor data.
 * 
 * @return Current sensor data snapshot
 */
sensor_data_t sensor_manager_get_data(void);

/**
 * @brief Internal update function used by sensor tasks.
 * Not intended for external use.
 */
void sensor_manager_update_temp(float temp_c);
void sensor_manager_update_prox(uint32_t count, float rpm);
void sensor_manager_update_imu(float ax, float ay, float az, float gx, float gy, float gz);
void sensor_manager_update_orientation(float roll, float pitch, float yaw);
void sensor_manager_update_digital(bool state, uint32_t changes);
void sensor_manager_update_voltage(float volts);

#ifdef __cplusplus
}
#endif