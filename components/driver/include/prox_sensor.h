#ifndef PROX_SENSOR_H
#define PROX_SENSOR_H

#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>

#define PROX_SENSOR_GPIO      GPIO_NUM_4   // ubah kalau perlu
#define PROX_PULSES_PER_REV   1           // pulsa per 1 putaran
#define PROX_WHEEL_CIRC_M     2.0f        // keliling roda (meter)

typedef struct {
    uint32_t pulses;
    float    dt_sec;
    float    speed_kmh;
} prox_data_t;

esp_err_t prox_init(gpio_num_t gpio_num, gpio_int_type_t intr_type);
void      prox_sample(prox_data_t *out, uint32_t sample_ms);

#endif // PROX_SENSOR_H
