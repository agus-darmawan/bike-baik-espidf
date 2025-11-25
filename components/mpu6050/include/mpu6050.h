#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef float float32_t;

#define MPU6050_DEFAULT_I2C_PORT I2C_NUM_0
#define MPU6050_DEFAULT_SDA_PIN  21
#define MPU6050_DEFAULT_SCL_PIN  22
#define MPU6050_DEFAULT_ADDR     0x68

typedef enum {
    MPU6050_FILTER_NONE,      // No filtering
    MPU6050_FILTER_LOWPASS,   // Low-pass filter only
    MPU6050_FILTER_MOVAVG,    // Moving average only
    MPU6050_FILTER_BOTH       // Both filters applied
} mpu6050_filter_t;

typedef enum {
    MPU6050_ORIENTATION_NONE,      // No orientation calculation
    MPU6050_ORIENTATION_MAHONY,    // Mahony filter
    MPU6050_ORIENTATION_MADGWICK   // Madgwick filter
} mpu6050_orientation_t;

typedef struct {
    float32_t ax, ay, az;              // Filtered acceleration in m/s^2
    float32_t gx, gy, gz;              // Gyro in rad/s (offset-corrected)
    float32_t raw_ax, raw_ay, raw_az;  // Raw accel in g
    float32_t raw_gx, raw_gy, raw_gz;  // Raw gyro in dps
    float32_t roll, pitch, yaw;        // Orientation in degrees
} mpu6050_data_t;

typedef struct {
    float32_t offset_x;
    float32_t offset_y;
    float32_t offset_z;
    bool calibrated;
} mpu6050_gyro_calib_t;

/**
 * Initialize MPU6050 sensor over I2C.
 * @param port I2C port number
 * @param sda_pin SDA GPIO pin
 * @param scl_pin SCL GPIO pin
 * @param addr I2C device address
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t mpu6050_init(i2c_port_t port, int sda_pin, int scl_pin, uint8_t addr);

/**
 * Read accelerometer and gyroscope data from MPU6050.
 * Automatically calculates orientation if enabled.
 * @param out Pointer to data structure
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t mpu6050_read(mpu6050_data_t *out);

/**
 * Calibrate gyroscope offsets.
 * Keep sensor stationary during calibration.
 * @param calib Pointer to calibration structure
 * @param samples Number of samples to average
 * @param delay_ms Delay between samples in milliseconds
 */
void mpu6050_calibrate_gyro(mpu6050_gyro_calib_t *calib, int samples, int delay_ms);

/**
 * Configure accelerometer filter type and parameters.
 * @param filter Filter type selection
 * @param lp_alpha Low-pass filter alpha coefficient (0..1), ignored if not using lowpass
 * @param movavg_size Moving average window size (max 16), ignored if not using movavg
 */
void mpu6050_set_filter(mpu6050_filter_t filter, float32_t lp_alpha, int movavg_size);

/**
 * Configure orientation filter type and parameters.
 * @param orientation Orientation filter selection (NONE, MAHONY, MADGWICK)
 * @param param1 Mahony Kp / Madgwick beta
 * @param param2 Mahony Ki (ignored for Madgwick)
 */
void mpu6050_set_orientation(mpu6050_orientation_t orientation, float32_t param1, float32_t param2);

#ifdef __cplusplus
}
#endif
