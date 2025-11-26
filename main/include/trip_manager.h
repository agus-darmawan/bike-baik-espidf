/**
 * @file trip_manager.h
 * @author @agus-darmawan
 * @brief Trip management dengan sensor fusion dan data transmission
 * @version 0.1
 * @date 2025-11-25
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Vehicle parameters structure
 */
typedef struct {
    float wheelbase;              /**< Wheelbase in meters */
    float wheel_circumference;    /**< Wheel circumference in meters */
    float mass;                   /**< Total mass (vehicle + rider) in kg */
} vehicle_params_t;

/**
 * @brief Initialize trip manager
 */
void trip_manager_init(void);

/**
 * @brief Start trip manager task
 */
void trip_manager_start(void);

/**
 * @brief Get current vehicle parameters
 * @param params Pointer to parameters structure
 */
void trip_manager_get_vehicle_params(vehicle_params_t *params);

/**
 * @brief Check if trip is active
 * @return true if trip is active, false otherwise
 */
bool trip_manager_is_trip_active(void);

/**
 * @brief Get current trip ID
 * @return Pointer to trip ID string (NULL if no active trip)
 */
const char* trip_manager_get_current_trip_id(void);

#ifdef __cplusplus
}
#endif
