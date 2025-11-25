#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Voltage sensor configuration structure
 */
typedef struct {
    adc_channel_t channel;     /**< ADC channel */
    adc_unit_t unit;           /**< ADC unit (ADC_UNIT_1 or ADC_UNIT_2) */
    adc_atten_t attenuation;   /**< ADC attenuation */
    float r1;                  /**< Upper resistor in voltage divider (ohms) */
    float r2;                  /**< Lower resistor in voltage divider (ohms) */
    uint8_t samples;           /**< Number of samples to average */
} voltage_sensor_config_t;

/**
 * @brief Initialize voltage sensor (ADC with voltage divider).
 * 
 * @param cfg Pointer to configuration structure
 * @return ESP_OK on success
 */
esp_err_t voltage_sensor_init(const voltage_sensor_config_t *cfg);

/**
 * @brief Read voltage in volts.
 * 
 * @return Measured voltage in volts, or negative value on error
 */
float voltage_sensor_read(void);

/**
 * @brief Read raw ADC value.
 * 
 * @return Raw ADC reading
 */
int voltage_sensor_read_raw(void);

#ifdef __cplusplus
}
#endif