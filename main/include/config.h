#pragma once
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/uart.h"
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

/* ===== SIM808 GPS/GPRS Module ===== */
#define SIM808_UART_PORT   UART_NUM_2
#define SIM808_TX_PIN      GPIO_NUM_17
#define SIM808_RX_PIN      GPIO_NUM_16
#define SIM808_PWR_PIN     GPIO_NUM_4      /* Power control pin */
#define SIM808_RST_PIN     -1              /* Reset pin (not used) */
#define SIM808_BAUD_RATE   9600

/* Network Configuration */
#define SIM808_APN         "internet"      /* Change to your APN */
#define SIM808_APN_USER    NULL            /* APN username if required */
#define SIM808_APN_PASS    NULL            /* APN password if required */

/* RabbitMQ Configuration */
#define RABBITMQ_BROKER    "http://your-rabbitmq-server.com:15672"
#define RABBITMQ_USERNAME  "guest"
#define RABBITMQ_PASSWORD  "guest"
#define RABBITMQ_TOPIC     "bike-performance"

/* API Server for Vehicle Parameters */
#define API_SERVER_URL     "http://your-api-server.com/api"

/* ===== Vehicle Parameters ===== */
#define VEHICLE_WHEELBASE_DEFAULT  1.3f    /* Default wheelbase in meters */
#define VEHICLE_WHEEL_CIRCUMFERENCE_DEFAULT 1.8f  /* Default wheel circumference in meters */
#define VEHICLE_MASS_DEFAULT       150.0f  /* Default vehicle + rider mass in kg */

/* ===== Trip Management ===== */
#define BATTERY_VOLTAGE_ENGINE_ON   12.2f  /* Voltage threshold for engine ON */
#define TRIP_UPDATE_INTERVAL_MS     1000   /* Update interval in milliseconds */
