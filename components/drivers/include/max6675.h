#ifndef MAX6675_H
#define MAX6675_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

// SPI Configuration for MAX6675
#define MAX6675_CLK_PIN     GPIO_NUM_18
#define MAX6675_MISO_PIN    GPIO_NUM_19
#define MAX6675_CS_PIN      GPIO_NUM_5
#define SPI_SPEED_HZ        1000000  // 1 MHz

// Function prototypes
esp_err_t max6675_init(void);
float max6675_read_temperature(void);

#endif // MAX6675_H