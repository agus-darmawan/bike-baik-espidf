/**
 * @file prox_sensor.h
 * @author @agus-darmawan
 * @brief This file contains the interface for the proximity opto sensor module
 * @version 0.1
 * @date 2025-11-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#pragma once
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Proximity sensor configuration structure
 */
typedef struct {
    gpio_num_t pin;
    gpio_int_type_t trigger;
    uint32_t debounce_us;
} proximity_config_t;

/**
 * @brief Initialize proximity opto sensor (ISR).
 * 
 * @param cfg pointer to config
 * @return ESP_OK on success
 */
esp_err_t proximity_init(const proximity_config_t *cfg);

/**
 * @brief Return total pulses since boot (one pulse = one wheel revolution).
 * 
 * @return pulse count
 */
uint32_t proximity_get_count(void);

#ifdef __cplusplus
}
#endif