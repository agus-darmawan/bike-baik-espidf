#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "driver/gpio.h"

// MPU6050 I2C Configuration
#define MPU6050_I2C_NUM         I2C_NUM_0
#define MPU6050_I2C_SCL_PIN     GPIO_NUM_22
#define MPU6050_I2C_SDA_PIN     GPIO_NUM_21
#define MPU6050_I2C_FREQ_HZ     100000
#define MPU6050_ADDR            0x68

// MPU6050 Registers
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_ZOUT_H     0x47

// Data structure
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float pitch;  // angle in degrees
    float roll;   // angle in degrees
} mpu6050_data_t;

// Function prototypes
esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read_data(mpu6050_data_t *data);

#endif // MPU6050_H