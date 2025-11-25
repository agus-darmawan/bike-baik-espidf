#pragma once
#include "driver/gpio.h"
#include "driver/spi_master.h"

/* Hardware mapping (project-specific) */
#define MAX6675_SPI_HOST   SPI2_HOST
#define MAX6675_CLK_PIN    GPIO_NUM_18
#define MAX6675_MISO_PIN   GPIO_NUM_19
#define MAX6675_CS_PIN     GPIO_NUM_5
#define MAX6675_SPI_HZ     1000000

/* Proximity opto (GPIO35 input-only, external pull-up required) */
#define PROX_PIN           GPIO_NUM_35
#define PROX_TRIGGER       GPIO_INTR_NEGEDGE
#define PROX_DEBOUNCE_US   5000 /* microseconds */
