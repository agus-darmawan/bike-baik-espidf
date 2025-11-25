#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_DEFAULT_I2C_PORT I2C_NUM_0
#define MPU6050_DEFAULT_SDA_PIN 21
#define MPU6050_DEFAULT_SCL_PIN 22
#define MPU6050_DEFAULT_ADDR    0x68

typedef struct {
    float ax,ay,az;
    float gx,gy,gz;
    float raw_ax,raw_ay,raw_az;
    float raw_gx,raw_gy,raw_gz;
} mpu6050_data_t;

typedef struct {
    float offset_x,offset_y,offset_z;
    bool calibrated;
} mpu6050_gyro_calib_t;

/**
 * Initialize MPU6050.
 * @param port I2C port
 * @param sda_pin SDA pin
 * @param scl_pin SCL pin
 * @param addr I2C address
 * @return ESP_OK on success
 */
esp_err_t mpu6050_init(i2c_port_t port,int sda_pin,int scl_pin,uint8_t addr);

/**
 * Read MPU6050 data.
 * @param out Pointer to data struct
 * @return ESP_OK on success
 */
esp_err_t mpu6050_read(mpu6050_data_t *out);

/**
 * Calibrate gyro offsets.
 * @param calib Pointer to calibration struct
 * @param samples Number of samples
 * @param delay_ms Delay between samples (ms)
 */
void mpu6050_calibrate_gyro(mpu6050_gyro_calib_t *calib,int samples,int delay_ms);

/**
 * Set filters for MPU6050 readings.
 * @param lp_alpha Low-pass alpha (0..1)
 * @param movavg_size Moving average buffer size
 */
void mpu6050_set_filters(float lp_alpha,int movavg_size);

#ifdef __cplusplus
}
#endif
