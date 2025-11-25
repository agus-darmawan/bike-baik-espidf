#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start all sensor reading tasks.
 * Creates FreeRTOS tasks for each sensor type.
 */
void sensor_tasks_start(void);

#ifdef __cplusplus
}
#endif