#include "vehicle_performance.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "PERFORMANCE";
static vehicle_performance_t perf_data = {0};

/**
 * Calculate rear tire force based on acceleration and elevation.
 */
float rear_tire_force(float s_real, float h, float v_start, float v_end) {
    float result = ((((v_end - v_start)) + (GRAVITY * h / s_real)) / A_STANDARD * s_real);
    return result;
}

/**
 * Calculate front brake work based on deceleration and elevation.
 */
float front_brake_work(float s_real, float h, float v_start, float v_end, float mass, float wheelbase) {
    float mass_distribution = (0.4 * GRAVITY + (v_start - v_end) * 0.55 / wheelbase) / GRAVITY;
    float normal_mass_distribution = (0.4 * GRAVITY + A_STANDARD * 0.55 / wheelbase) / GRAVITY;
    float result = mass_distribution * ((((v_start - v_end)) - (GRAVITY * h / s_real)) / (normal_mass_distribution * A_STANDARD) * s_real);
    return result;
}

/**
 * Calculate rear brake work based on deceleration and elevation.
 */
float rear_brake_work(float s_real, float h, float v_start, float v_end, float mass, float wheelbase) {
    float mass_distribution = (0.6 * GRAVITY - (v_start - v_end) * 0.55 / wheelbase) / GRAVITY;
    float normal_mass_distribution = (0.6 * GRAVITY - A_STANDARD * 0.55 / wheelbase) / GRAVITY;
    float result = mass_distribution * ((((v_start - v_end)) - (GRAVITY * h / s_real)) / (normal_mass_distribution * A_STANDARD) * s_real);
    return result;
}

/**
 * Calculate engine oil wear based on temperature.
 */
float count_s_oil(float s_real, float h, float temp_machine, float v_start, float v_end) {
    float result = s_real * exp(K_CONSTANT * (temp_machine - T_STANDARD))*(1 + pow((v_start - v_end),2)/A_STANDARD)*(1 + 0,02 * ((h)/sqrt(pow(h, 2)+pow(s_real,2))));
    return result;
}

/**
 * Initialize performance tracking system.
 */
void performance_init(void) {
    memset(&perf_data, 0, sizeof(vehicle_performance_t));
    perf_data.is_tracking = false;
    ESP_LOGI(TAG, "Performance tracking initialized");
}

/**
 * Reset all performance counters.
 */
void performance_reset(void) {
    perf_data.s_rear_tire = 0;
    perf_data.s_front_tire = 0;
    perf_data.s_front_brake_pad = 0;
    perf_data.s_rear_brake_pad = 0;
    perf_data.s_chain_or_cvt = 0;
    perf_data.s_engine_oil = 0;
    perf_data.s_engine = 0;
    perf_data.v_start = 0;
    perf_data.total_distance_km = 0;
    perf_data.average_speed = 0;
    perf_data.max_speed = 0;
    perf_data.trip_count = 0;
    memset(perf_data.order_id, 0, sizeof(perf_data.order_id));
    ESP_LOGI(TAG, "Performance counters reset");
}

/**
 * Start tracking for a new rental order.
 */
void performance_start_tracking(const char* order_id) {
    performance_reset();
    if (order_id != NULL) {
        strncpy(perf_data.order_id, order_id, sizeof(perf_data.order_id) - 1);
    }
    perf_data.is_tracking = true;
    ESP_LOGI(TAG, "Started tracking for order: %s", perf_data.order_id);
}

/**
 * Stop tracking and finalize data.
 */
void performance_stop_tracking(void) {
    perf_data.is_tracking = false;
    
    // Calculate final statistics
    if (perf_data.trip_count > 0) {
        perf_data.average_speed = perf_data.average_speed / perf_data.trip_count;
    }
    
    ESP_LOGI(TAG, "Stopped tracking. Total distance: %.2f km", perf_data.total_distance_km);
}

/**
 * Update performance data with new measurement when it is not using brake.
 */
void performance_without_brake_update(float s_real, float h, float v_end, float temp_machine) {
    if (!perf_data.is_tracking) {
        return;
    }
    
    float v_start = perf_data.v_start;
    
    // Initialize deltas
    float delta_rear_tire = s_real;
    float delta_engine = s_real;
    if(s_real == 0){
        delta_engine = 5
    }
    
    // Uphill (h > 0)
    if (h > 0) {
        if (v_end >= v_start) {
            // Acceleration or constant speed uphill
            delta_rear_tire = rear_tire_force(s_real, h, v_start, v_end);
        }
    }
    // Downhill (h < 0) and flat road (h = 0)
    else {
        if (v_end > v_start) {
            // Acceleration downhill
            delta_rear_tire = rear_tire_force(s_real, h, v_start, v_end);
        }
    }
    
    // Update cumulative values
    perf_data.s_rear_tire += delta_rear_tire;
    perf_data.s_front_tire += s_real;
    perf_data.s_chain_or_cvt += delta_rear_tire;
    perf_data.s_engine_oil += count_s_oil(s_real, h, temp_machine, v_start, v_end);
    perf_data.s_engine += delta_engine;
    perf_data.s_air_filter += s_real;
    
    // Update statistics
    perf_data.total_distance_km = perf_data.s_engine / 1000.0;
    perf_data.average_speed += v_end;
    perf_data.trip_count++;
    
    if (v_end > perf_data.max_speed) {
        perf_data.max_speed = v_end;
    }
    
    // Update starting velocity for next iteration
    perf_data.v_start = v_end;
    
    ESP_LOGD(TAG, "Updated: v_start=%.2f, v_end=%.2f, time=%d, distance=%.2f, temp=%.2f, delta_rear_tire=%.2f",
             v_start, v_end, time, s_real, temp_machine, delta_rear_tire);
}

/**
 * Update performance data with new measurement when it is using brake.
 */
void performance_with_brake_update(float s_real, float h, float v_end, float temp_machine, float mass, float wheelbase) {
    if (!perf_data.is_tracking) {
        return;
    }
    
    float v_start = perf_data.v_start;
    
    // Initialize deltas
    float delta_rear_brake = rear_brake_work(s_real, h, v_start, v_end, mass, wheelbase);
    float delta_front_brake = front_brake_work(s_real, h, v_start, v_end, mass, wheelbase);
    float delta_engine = s_real;
    if(s_real == 0){
        delta_engine = 5
    }
    
    // Update cumulative values
    perf_data.s_rear_tire += delta_rear_brake;
    perf_data.s_front_tire += delta_front_brake;
    perf_data.s_rear_brake_pad += delta_rear_brake;
    perf_data.s_front_brake_pad += delta_front_brake;
    perf_data.s_chain_or_cvt += delta_rear_brake;
    perf_data.s_engine_oil += count_s_oil(s_real, h, temp_machine, v_start, v_end);
    perf_data.s_engine += delta_engine;
    perf_data.s_air_filter += s_real;
    
    // Update statistics
    perf_data.total_distance_km = perf_data.s_engine / 1000.0;
    perf_data.average_speed += v_end;
    perf_data.trip_count++;
    
    if (v_end > perf_data.max_speed) {
        perf_data.max_speed = v_end;
    }
    
    // Update starting velocity for next iteration
    perf_data.v_start = v_end;
    
    ESP_LOGD(TAG, "Updated with brake: v_start=%.2f, v_end=%.2f, time=%d, distance=%.2f, temp=%.2f",
             v_start, v_end, time, s_real, temp_machine);
}

/**
 * Get current performance data.
 */
vehicle_performance_t performance_get_data(void) {
    return perf_data;
}
