/**
 * @file digital_input.h
 * @author @agus-darmawan
 * @brief This file contains the interface for a digital input sensor module
 * @version 0.1
 * @date 2025-11-25
 * @copyright Copyright (c) 2025
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Digital input configuration structure
 */
typedef struct {
    gpio_num_t pin;              /**< GPIO pin number */
    gpio_pull_mode_t pull_mode;  /**< Pull-up/down configuration */
    uint32_t debounce_ms;        /**< Debounce time in milliseconds */
} digital_input_config_t;

/**
 * @brief Initialize digital input sensor.
 * 
 * @param cfg Pointer to configuration structure
 * @return ESP_OK on success or error code
 */
esp_err_t digital_input_init(const digital_input_config_t *cfg);

/**
 * @brief Read digital input state.
 * 
 * @return true if input is HIGH, false if LOW
 */
bool digital_input_read(void);

/**
 * @brief Get total state change count since boot.
 * 
 * @return Total number of state changes detected
 */
uint32_t digital_input_get_change_count(void);

#ifdef __cplusplus
}
#endif