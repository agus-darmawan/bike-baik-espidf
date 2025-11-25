#include "mpu6050.h"
#include "moving_average.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "MPU6050";

static i2c_port_t i2c_port = MPU6050_DEFAULT_I2C_PORT;
static uint8_t dev_addr = MPU6050_DEFAULT_ADDR;

// Filter
static float lp_alpha = 0.2f;
static movavg_t ma_ax, ma_ay, ma_az;
static float ma_buf_ax[32], ma_buf_ay[32], ma_buf_az[32]; // max 32

// Gyro calibration
static mpu6050_gyro_calib_t gyro_calib = {0,0,0,false};

// Internal I2C helpers
static esp_err_t write_reg(uint8_t reg, uint8_t val){
    uint8_t buf[2] = {reg,val};
    return i2c_master_write_to_device(i2c_port, dev_addr, buf, 2, 100/portTICK_PERIOD_MS);
}
static esp_err_t read_regs(uint8_t reg,uint8_t *buf,size_t len){
    return i2c_master_write_read_device(i2c_port,dev_addr,&reg,1,buf,len,100/portTICK_PERIOD_MS);
}

esp_err_t mpu6050_init(i2c_port_t port,int sda,int scl,uint8_t addr){
    i2c_port = port; dev_addr = addr;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(i2c_port,&conf);
    i2c_driver_install(i2c_port,conf.mode,0,0,0);

    write_reg(0x6B,0x00); // wake up
    write_reg(0x1B,0x00); // gyro ±250dps
    write_reg(0x1C,0x00); // accel ±2g
    write_reg(0x1A,0x03); // DLPF 44Hz

    // init moving average
    movavg_init(&ma_ax, ma_buf_ax, 8);
    movavg_init(&ma_ay, ma_buf_ay, 8);
    movavg_init(&ma_az, ma_buf_az, 8);

    ESP_LOGI(TAG,"MPU6050 initialized on I2C%d SDA%d SCL%d",port,sda,scl);
    return ESP_OK;
}

void mpu6050_set_filters(float alpha,int mov_size){
    lp_alpha = alpha;
    if(mov_size>32) mov_size=32;
    movavg_init(&ma_ax, ma_buf_ax, mov_size);
    movavg_init(&ma_ay, ma_buf_ay, mov_size);
    movavg_init(&ma_az, ma_buf_az, mov_size);
}

void mpu6050_calibrate_gyro(mpu6050_gyro_calib_t *calib,int samples,int delay_ms){
    if(!calib) return;
    int64_t sumx=0,sumy=0,sumz=0;
    uint8_t buf[14];
    for(int i=0;i<samples;i++){
        if(read_regs(0x3B,buf,14)!=ESP_OK){ vTaskDelay(1/portTICK_PERIOD_MS); continue;}
        int16_t gx=(buf[8]<<8)|buf[9];
        int16_t gy=(buf[10]<<8)|buf[11];
        int16_t gz=(buf[12]<<8)|buf[13];
        sumx+=gx; sumy+=gy; sumz+=gz;
        vTaskDelay(delay_ms/portTICK_PERIOD_MS);
    }
    calib->offset_x=(float)sumx/samples;
    calib->offset_y=(float)sumy/samples;
    calib->offset_z=(float)sumz/samples;
    calib->calibrated=true;
    gyro_calib=*calib;
    ESP_LOGI(TAG,"Gyro calibrated x=%.2f y=%.2f z=%.2f",calib->offset_x,calib->offset_y,calib->offset_z);
}

esp_err_t mpu6050_read(mpu6050_data_t *out){
    if(!out) return ESP_ERR_INVALID_ARG;
    uint8_t buf[14];
    if(read_regs(0x3B,buf,14)!=ESP_OK) return ESP_FAIL;

    int16_t ax_raw = (buf[0]<<8)|buf[1];
    int16_t ay_raw = (buf[2]<<8)|buf[3];
    int16_t az_raw = (buf[4]<<8)|buf[5];
    int16_t gx_raw = (buf[8]<<8)|buf[9];
    int16_t gy_raw = (buf[10]<<8)|buf[11];
    int16_t gz_raw = (buf[12]<<8)|buf[13];

    out->raw_ax=ax_raw/16384.0f;
    out->raw_ay=ay_raw/16384.0f;
    out->raw_az=az_raw/16384.0f;

    out->ax=movavg_update(&ma_ax, out->raw_ax*9.80665f);
    out->ay=movavg_update(&ma_ay, out->raw_ay*9.80665f);
    out->az=movavg_update(&ma_az, out->raw_az*9.80665f);

    out->raw_gx=gx_raw/131.0f - gyro_calib.offset_x/131.0f;
    out->raw_gy=gy_raw/131.0f - gyro_calib.offset_y/131.0f;
    out->raw_gz=gz_raw/131.0f - gyro_calib.offset_z/131.0f;

    out->gx=out->raw_gx*M_PI/180.0f;
    out->gy=out->raw_gy*M_PI/180.0f;
    out->gz=out->raw_gz*M_PI/180.0f;

    return ESP_OK;
}