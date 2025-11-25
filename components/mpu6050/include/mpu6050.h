#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_DEFAULT_I2C_PORT I2C_NUM_0
#define MPU6050_DEFAULT_SDA_PIN  21
#define MPU6050_DEFAULT_SCL_PIN  22
#define MPU6050_DEFAULT_ADDR     0x68

/**
 * @brief MPU6050 sensor data structure
 */
typedef struct {
    float ax, ay, az;           /**< Filtered accelerometer (m/s^2) */
    float gx, gy, gz;           /**< Filtered gyroscope (rad/s) */
    float raw_ax, raw_ay, raw_az; /**< Raw accelerometer (g) */
    float raw_gx, raw_gy, raw_gz; /**< Raw gyroscope (deg/s) */
} mpu6050_data_t;

/**
 * @brief MPU6050 gyroscope calibration data
 */
typedef struct {
    float offset_x, offset_y, offset_z; /**< Gyro offsets */
    bool calibrated;                     /**< Calibration status */
} mpu6050_gyro_calib_t;

/**
 * @brief Initialize MPU6050 sensor.
 * 
 * @param port I2C port number
 * @param sda_pin SDA GPIO pin
 * @param scl_pin SCL GPIO pin
 * @param addr I2C device address (typically 0x68)
 * @return ESP_OK on success
 */
esp_err_t mpu6050_init(i2c_port_t port, int sda_pin, int scl_pin, uint8_t addr);

/**
 * @brief Read MPU6050 sensor data.
 * 
 * @param out Pointer to data structure to store results
 * @return ESP_OK on success
 */
esp_err_t mpu6050_read(mpu6050_data_t *out);

/**
 * @brief Calibrate gyroscope offsets.
 * 
 * @param calib Pointer to calibration structure
 * @param samples Number of samples to average
 * @param delay_ms Delay between samples in milliseconds
 */
void mpu6050_calibrate_gyro(mpu6050_gyro_calib_t *calib, int samples, int delay_ms);

/**
 * @brief Configure digital filters for sensor readings.
 * 
 * @param lp_alpha Low-pass filter coefficient (0.0 to 1.0)
 * @param movavg_size Moving average window size
 */
void mpu6050_set_filters(float lp_alpha, int movavg_size);

#ifdef __cplusplus
}
#endif