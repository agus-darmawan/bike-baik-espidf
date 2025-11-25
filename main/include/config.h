#pragma once
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"

/* ===== MAX6675 Temperature Sensor ===== */
#define MAX6675_SPI_HOST   SPI2_HOST
#define MAX6675_CLK_PIN    GPIO_NUM_18
#define MAX6675_MISO_PIN   GPIO_NUM_19
#define MAX6675_CS_PIN     GPIO_NUM_5
#define MAX6675_SPI_HZ     1000000

/* ===== Proximity Opto Sensor ===== */
#define PROX_PIN           GPIO_NUM_35  /* Input-only, needs external pull-up */
#define PROX_TRIGGER       GPIO_INTR_NEGEDGE
#define PROX_DEBOUNCE_US   5000        /* microseconds */

/* ===== MPU6050 IMU Sensor ===== */
#define MPU6050_I2C_PORT   I2C_NUM_0
#define MPU6050_SDA_PIN    GPIO_NUM_21
#define MPU6050_SCL_PIN    GPIO_NUM_22
#define MPU6050_ADDR       0x68

/* ===== Digital Input Sensor ===== */
#define DIGITAL_INPUT_PIN  GPIO_NUM_33
#define DIGITAL_INPUT_PULL GPIO_PULLUP_ONLY
#define DIGITAL_DEBOUNCE_MS 50

/* ===== Voltage Sensor (ADC) ===== */
#define VOLTAGE_ADC_UNIT   ADC_UNIT_1
#define VOLTAGE_ADC_CHAN   ADC_CHANNEL_6  /* GPIO34 */
#define VOLTAGE_ADC_ATTEN  ADC_ATTEN_DB_12 /* 0-3.3V range */
#define VOLTAGE_R1         10000.0f       /* Upper resistor (ohms) */
#define VOLTAGE_R2         10000.0f       /* Lower resistor (ohms) */
#define VOLTAGE_SAMPLES    10             /* Number of samples to average */