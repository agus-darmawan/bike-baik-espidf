#pragma once
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t pin;
    gpio_int_type_t trigger;
    uint32_t debounce_us;
} proximity_config_t;

/**
 * Initialize proximity opto sensor (ISR).
 * @param cfg pointer to config
 * @return ESP_OK on success
 */
esp_err_t proximity_init(const proximity_config_t *cfg);

/**
 * Return total pulses since boot (one pulse = one wheel revolution).
 * @return pulse count
 */
uint32_t proximity_get_count(void);

#ifdef __cplusplus
}
#endif