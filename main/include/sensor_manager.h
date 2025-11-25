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

/* initialize sensors and RTOS primitives (call once) */
void sensor_manager_init(void);

/* start sensor tasks (creates FreeRTOS tasks) */
void sensor_manager_start_tasks(void);

/* safe read (copy) of sensor data */
sensor_data_t sensor_manager_get_data(void);

/* internal update used by tasks (temp_or_nan: < -500 => ignore) */
void sensor_manager_update(float temp_or_nan, uint32_t count, float rpm);

#ifdef __cplusplus
}
#endif
