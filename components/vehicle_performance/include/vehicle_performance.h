/**
 * @file vehicle_performance.h
 * @author @agus-darmawan
 * @brief Vehicle performance tracking and calculation
 * @version 0.1
 * @date 2025-11-25
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Constants ===== */
#define GRAVITY 9.80665f           /**< Gravity constant (m/s²) */
#define A_STANDARD 0.5f            /**< Standard acceleration (m/s²) */
#define K_CONSTANT 0.02f           /**< Oil wear constant */
#define T_STANDARD 80.0f           /**< Standard engine temperature (°C) */

/* ===== Vehicle Performance Data Structure ===== */
typedef struct {
    char order_id[64];             /**< Current rental order ID */
    bool is_tracking;              /**< Tracking status */
    
    /* Cumulative wear metrics (in equivalent kilometers) */
    float s_rear_tire;             /**< Rear tire wear */
    float s_front_tire;            /**< Front tire wear */
    float s_rear_brake_pad;        /**< Rear brake pad wear */
    float s_front_brake_pad;       /**< Front brake pad wear */
    float s_chain_or_cvt;          /**< Chain/CVT wear */
    float s_engine_oil;            /**< Engine oil wear */
    float s_engine;                /**< Engine wear (total distance) */
    float s_air_filter;            /**< Air filter wear */
    
    /* Trip statistics */
    float total_distance_km;       /**< Total distance traveled (km) */
    float average_speed;           /**< Average speed (m/s) */
    float max_speed;               /**< Maximum speed (m/s) */
    uint32_t trip_count;           /**< Number of updates */
    
    /* Current velocity for next iteration */
    float v_start;                 /**< Starting velocity for next update */
} vehicle_performance_t;

/**
 * @brief Initialize performance tracking system
 */
void performance_init(void);

/**
 * @brief Reset all performance counters
 */
void performance_reset(void);

/**
 * @brief Start tracking for a new rental order
 * @param order_id Order identifier string
 */
void performance_start_tracking(const char* order_id);

/**
 * @brief Stop tracking and finalize data
 */
void performance_stop_tracking(void);

/**
 * @brief Update performance data when NOT using brake
 * @param s_real Actual distance traveled (m)
 * @param h Elevation change (m), positive = uphill
 * @param v_end Ending velocity (m/s)
 * @param temp_machine Engine temperature (°C)
 * @param time Time interval (seconds)
 */
void performance_without_brake_update(float s_real, float h, float v_end, 
                                      float temp_machine, int time);

/**
 * @brief Update performance data when using brake
 * @param s_real Actual distance traveled (m)
 * @param h Elevation change (m)
 * @param v_end Ending velocity (m/s)
 * @param temp_machine Engine temperature (°C)
 * @param time Time interval (seconds)
 * @param mass Vehicle + rider mass (kg)
 * @param wheelbase Wheelbase (m)
 */
void performance_with_brake_update(float s_real, float h, float v_end,
                                   float temp_machine, int time,
                                   float mass, float wheelbase);

/**
 * @brief Get current performance data
 * @return Copy of performance data structure
 */
vehicle_performance_t performance_get_data(void);

#ifdef __cplusplus
}
#endif
