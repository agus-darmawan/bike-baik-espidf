#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float temperature_c;
    uint32_t wheel_count;
    float rpm;
} sensor_data_t;

void sensor_manager_init(void);
void sensor_manager_start_tasks(void);
sensor_data_t sensor_manager_get_data(void);
void sensor_manager_update(float temp_or_nan, uint32_t count, float rpm);

#ifdef __cplusplus
}
#endif
