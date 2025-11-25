#pragma once
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize MAX6675.
 * @param host SPI host (e.g., SPI2_HOST)
 * @param clk_pin SCLK gpio
 * @param miso_pin MISO gpio
 * @param cs_pin CS gpio
 * @param hz SPI clock speed
 * @param spi_sem Optional pointer to a binary semaphore to protect shared SPI bus (can be NULL)
 * @return ESP_OK on success
 */
esp_err_t max6675_init(spi_host_device_t host,
                      gpio_num_t clk_pin,
                      gpio_num_t miso_pin,
                      gpio_num_t cs_pin,
                      uint32_t hz,
                      SemaphoreHandle_t spi_sem);

/**
 * Read temperature in Celsius. Returns a negative large value on error.
 * @return Temperature in °C, or negative value on error
 */
float max6675_read_temperature(void);

#ifdef __cplusplus
}
#endif