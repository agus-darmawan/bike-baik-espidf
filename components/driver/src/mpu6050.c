#include "mpu6050.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MPU6050";

/**
 * Initialize MPU6050 sensor over I2C.
 */
esp_err_t mpu6050_init(void) {
    esp_err_t ret;
    
    // I2C master configuration
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU6050_I2C_SDA_PIN,
        .scl_io_num = MPU6050_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU6050_I2C_FREQ_HZ,
    };
    
    ret = i2c_param_config(MPU6050_I2C_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters");
        return ret;
    }
    
    ret = i2c_driver_install(MPU6050_I2C_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver");
        return ret;
    }
    
    // Wake up MPU6050 (it starts in sleep mode)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0x00, true);  // Clear sleep bit
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(MPU6050_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

/**
 * Read raw data from MPU6050.
 */
static esp_err_t mpu6050_read_raw(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(MPU6050_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * Read and process MPU6050 data.
 */
esp_err_t mpu6050_read_data(mpu6050_data_t *data) {
    uint8_t raw_data[14];
    esp_err_t ret;
    
    // Read all sensor data at once
    ret = mpu6050_read_raw(MPU6050_ACCEL_XOUT_H, raw_data, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050 data");
        return ret;
    }
    
    // Convert raw data to meaningful values
    int16_t accel_x_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    int16_t accel_y_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    int16_t accel_z_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    
    int16_t gyro_x_raw = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    int16_t gyro_y_raw = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    int16_t gyro_z_raw = (int16_t)((raw_data[12] << 8) | raw_data[13]);
    
    // Convert to g (gravity) and degrees/sec
    data->accel_x = accel_x_raw / 16384.0;
    data->accel_y = accel_y_raw / 16384.0;
    data->accel_z = accel_z_raw / 16384.0;
    
    data->gyro_x = gyro_x_raw / 131.0;
    data->gyro_y = gyro_y_raw / 131.0;
    data->gyro_z = gyro_z_raw / 131.0;
    
    // Calculate pitch and roll angles
    data->pitch = atan2(data->accel_y, sqrt(data->accel_x * data->accel_x + data->accel_z * data->accel_z)) * 180.0 / M_PI;
    data->roll = atan2(-data->accel_x, data->accel_z) * 180.0 / M_PI;
    
    return ESP_OK;
}