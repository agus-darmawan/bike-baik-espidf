#include "mpu6050.h"
#include "moving_average.h"
#include "orientation.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "MPU6050";

static i2c_port_t i2c_port = I2C_NUM_0;
static uint8_t dev_addr = MPU6050_DEFAULT_ADDR;

// Filter configuration
static mpu6050_filter_t filter_type = MPU6050_FILTER_BOTH;
static float lp_alpha = 0.2f;
static int mav_size = 8;

// Orientation configuration
static mpu6050_orientation_t orientation_type = MPU6050_ORIENTATION_MAHONY;
static float orientation_param1 = 0.5f;  // Mahony Kp / Madgwick beta
static float orientation_param2 = 0.0f;  // Mahony Ki
static float rotation_matrix[3][3];
static uint32_t last_update_time = 0;
static bool orientation_initialized = false;

// Orientation filter instances
static mahony_filter_t mahony_filter;
static madgwick_filter_t madgwick_filter;

// Filter state
static float accel_lp_x = 0.0f, accel_lp_y = 0.0f, accel_lp_z = 0.0f;
static movavg_t mav_x, mav_y, mav_z;
static float mav_buf_x[16], mav_buf_y[16], mav_buf_z[16];

// Gyro calibration
static mpu6050_gyro_calib_t gyro_calib = {0, 0, 0, false};

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    return i2c_master_write_to_device(i2c_port, dev_addr, data, 2, 100 / portTICK_PERIOD_MS);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_write_read_device(i2c_port, dev_addr, &reg, 1, buf, len, 100 / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_init(i2c_port_t port, int sda_pin, int scl_pin, uint8_t addr) {
    i2c_port = port;
    dev_addr = addr;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    
    esp_err_t ret = i2c_param_config(port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C");
        return ret;
    }

    ret = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver");
        return ret;
    }

    // Wake up MPU6050
    ret = write_reg(0x6B, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }

    // Configure gyro: ±250 dps
    write_reg(0x1B, 0x00);

    // Configure accel: ±2g
    write_reg(0x1C, 0x00);

    // Configure DLPF: ~44Hz bandwidth
    write_reg(0x1A, 0x03);

    // Initialize filters
    movavg_init(&mav_x, mav_buf_x, mav_size);
    movavg_init(&mav_y, mav_buf_y, mav_size);
    movavg_init(&mav_z, mav_buf_z, mav_size);

    // Initialize orientation filter (default: Mahony)
    mahony_init(&mahony_filter, orientation_param1, orientation_param2);
    orientation_initialized = true;
    last_update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

void mpu6050_set_filter(mpu6050_filter_t filter, float32_t lp_alpha_in, int movavg_in) {
    filter_type = filter;

    if (filter == MPU6050_FILTER_LOWPASS || filter == MPU6050_FILTER_BOTH) {
        if (lp_alpha_in > 0.0f && lp_alpha_in <= 1.0f) {
            lp_alpha = lp_alpha_in;
        }
    }

    if (filter == MPU6050_FILTER_MOVAVG || filter == MPU6050_FILTER_BOTH) {
        if (movavg_in > 0 && movavg_in <= 16) {
            mav_size = movavg_in;
            movavg_init(&mav_x, mav_buf_x, mav_size);
            movavg_init(&mav_y, mav_buf_y, mav_size);
            movavg_init(&mav_z, mav_buf_z, mav_size);
        }
    }

    ESP_LOGI(TAG, "Accel filter: type=%d, lp_alpha=%.2f, movavg_size=%d", 
             filter_type, lp_alpha, mav_size);
}

void mpu6050_set_orientation(mpu6050_orientation_t orientation, float32_t param1, float32_t param2) {
    orientation_type = orientation;
    orientation_param1 = param1;
    orientation_param2 = param2;

    if (orientation == MPU6050_ORIENTATION_MAHONY) {
        mahony_init(&mahony_filter, param1, param2);
        orientation_initialized = true;
        ESP_LOGI(TAG, "Orientation filter: Mahony (Kp=%.2f, Ki=%.2f)", param1, param2);
    } else if (orientation == MPU6050_ORIENTATION_MADGWICK) {
        madgwick_init(&madgwick_filter, param1);
        orientation_initialized = true;
        ESP_LOGI(TAG, "Orientation filter: Madgwick (beta=%.2f)", param1);
    } else {
        orientation_initialized = false;
        ESP_LOGI(TAG, "Orientation filter: Disabled");
    }

    last_update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void mpu6050_calibrate_gyro(mpu6050_gyro_calib_t *calib, int samples, int delay_ms) {
    if (!calib) return;

    int64_t sumx = 0, sumy = 0, sumz = 0;
    int16_t gx, gy, gz;
    uint8_t buf[14];

    ESP_LOGI(TAG, "Calibrating gyro with %d samples...", samples);

    for (int i = 0; i < samples; i++) {
        if (read_regs(0x3B, buf, 14) != ESP_OK) {
            vTaskDelay(1 / portTICK_PERIOD_MS);
            continue;
        }
        
        gx = (buf[8] << 8) | buf[9];
        gy = (buf[10] << 8) | buf[11];
        gz = (buf[12] << 8) | buf[13];
        
        sumx += gx;
        sumy += gy;
        sumz += gz;
        
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }

    calib->offset_x = (float)sumx / (float)samples;
    calib->offset_y = (float)sumy / (float)samples;
    calib->offset_z = (float)sumz / (float)samples;
    calib->calibrated = true;

    gyro_calib = *calib;

    ESP_LOGI(TAG, "Gyro calibrated - offsets: X=%.2f, Y=%.2f, Z=%.2f", 
             calib->offset_x, calib->offset_y, calib->offset_z);
}

esp_err_t mpu6050_read(mpu6050_data_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;

    uint8_t buf[14];
    esp_err_t ret = read_regs(0x3B, buf, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return ESP_FAIL;
    }

    // Parse raw data
    int16_t ax_raw = (buf[0] << 8) | buf[1];
    int16_t ay_raw = (buf[2] << 8) | buf[3];
    int16_t az_raw = (buf[4] << 8) | buf[5];
    int16_t gx_raw = (buf[8] << 8) | buf[9];
    int16_t gy_raw = (buf[10] << 8) | buf[11];
    int16_t gz_raw = (buf[12] << 8) | buf[13];

    // Store raw values in g
    out->raw_ax = ax_raw / 16384.0f;
    out->raw_ay = ay_raw / 16384.0f;
    out->raw_az = az_raw / 16384.0f;

    // Store raw gyro in dps
    out->raw_gx = gx_raw / 131.0f;
    out->raw_gy = gy_raw / 131.0f;
    out->raw_gz = gz_raw / 131.0f;

    // Apply gyro calibration and convert to rad/s
    const float dps2rad = 3.14159265358979f / 180.0f;
    out->gx = ((float)gx_raw - gyro_calib.offset_x) / 131.0f * dps2rad;
    out->gy = ((float)gy_raw - gyro_calib.offset_y) / 131.0f * dps2rad;
    out->gz = ((float)gz_raw - gyro_calib.offset_z) / 131.0f * dps2rad;

    // Convert accel to m/s^2
    float ax_ms2 = (ax_raw / 16384.0f) * 9.80665f;
    float ay_ms2 = (ay_raw / 16384.0f) * 9.80665f;
    float az_ms2 = (az_raw / 16384.0f) * 9.80665f;

    // Apply selected filter
    switch (filter_type) {
        case MPU6050_FILTER_NONE:
            out->ax = ax_ms2;
            out->ay = ay_ms2;
            out->az = az_ms2;
            break;

        case MPU6050_FILTER_LOWPASS:
            accel_lp_x = lowpass_update(accel_lp_x, ax_ms2, lp_alpha);
            accel_lp_y = lowpass_update(accel_lp_y, ay_ms2, lp_alpha);
            accel_lp_z = lowpass_update(accel_lp_z, az_ms2, lp_alpha);
            out->ax = accel_lp_x;
            out->ay = accel_lp_y;
            out->az = accel_lp_z;
            break;

        case MPU6050_FILTER_MOVAVG:
            out->ax = movavg_update(&mav_x, ax_ms2);
            out->ay = movavg_update(&mav_y, ay_ms2);
            out->az = movavg_update(&mav_z, az_ms2);
            break;

        case MPU6050_FILTER_BOTH:
            ax_ms2 = movavg_update(&mav_x, ax_ms2);
            ay_ms2 = movavg_update(&mav_y, ay_ms2);
            az_ms2 = movavg_update(&mav_z, az_ms2);
            
            accel_lp_x = lowpass_update(accel_lp_x, ax_ms2, lp_alpha);
            accel_lp_y = lowpass_update(accel_lp_y, ay_ms2, lp_alpha);
            accel_lp_z = lowpass_update(accel_lp_z, az_ms2, lp_alpha);
            
            out->ax = accel_lp_x;
            out->ay = accel_lp_y;
            out->az = accel_lp_z;
            break;
    }

    // Calculate orientation if enabled
    if (orientation_initialized && orientation_type != MPU6050_ORIENTATION_NONE) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (now - last_update_time) / 1000.0f;
        last_update_time = now;

        // Limit dt to reasonable range
        if (dt > 0.0f && dt < 1.0f) {
            if (orientation_type == MPU6050_ORIENTATION_MAHONY) {
                mahony_update(&mahony_filter, out->gx, out->gy, out->gz, 
                             out->ax, out->ay, out->az, 
                             dt, rotation_matrix);
                mahony_get_rpy(rotation_matrix, &out->roll, &out->pitch, &out->yaw);
            } else if (orientation_type == MPU6050_ORIENTATION_MADGWICK) {
                madgwick_update(&madgwick_filter, out->gx, out->gy, out->gz, 
                               out->ax, out->ay, out->az, 
                               dt, rotation_matrix);
                madgwick_get_rpy(rotation_matrix, &out->roll, &out->pitch, &out->yaw);
            }
        }
    } else {
        out->roll = 0.0f;
        out->pitch = 0.0f;
        out->yaw = 0.0f;
    }

    return ESP_OK;
}
