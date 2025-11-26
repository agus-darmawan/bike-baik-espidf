/**
 * @file alert_manager.h
 * @author @agus-darmawan
 * @brief Vehicle alert detection and management (crash, fall, bump)
 * @version 0.1
 * @date 2025-11-26
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Alert types
 */
typedef enum {
    ALERT_TYPE_CRASH,    /**< High impact collision */
    ALERT_TYPE_FALL,     /**< Vehicle tipped over */
    ALERT_TYPE_BUMP      /**< Large road bump/pothole */
} alert_type_t;

/**
 * @brief Alert data structure
 */
typedef struct {
    alert_type_t type;       /**< Alert type */
    char serial_number[32];  /**< Vehicle serial number */
    float latitude;          /**< GPS latitude */
    float longitude;         /**< GPS longitude */
    uint64_t timestamp;      /**< Unix timestamp (ms) */
    
    /* Sensor data at alert moment */
    float accel_x, accel_y, accel_z;  /**< Acceleration (m/s²) */
    float roll, pitch, yaw;            /**< Orientation (degrees) */
    float speed;                       /**< Speed (m/s) */
} alert_data_t;

/**
 * @brief Alert callback function type
 */
typedef void (*alert_callback_t)(const alert_data_t *alert, void *user_data);

/**
 * @brief Initialize alert manager
 * @param serial_number Vehicle serial number
 */
void alert_manager_init(const char *serial_number);

/**
 * @brief Register alert callback
 * @param callback Callback function to call when alert is detected
 * @param user_data User data passed to callback
 */
void alert_manager_register_callback(alert_callback_t callback, void *user_data);

/**
 * @brief Update alert detection with new sensor data
 * @param accel_x Acceleration X (m/s²)
 * @param accel_y Acceleration Y (m/s²)
 * @param accel_z Acceleration Z (m/s²)
 * @param roll Roll angle (degrees)
 * @param pitch Pitch angle (degrees)
 * @param yaw Yaw angle (degrees)
 * @param speed Current speed (m/s)
 * @param latitude GPS latitude
 * @param longitude GPS longitude
 */
void alert_manager_update(float accel_x, float accel_y, float accel_z,
                         float roll, float pitch, float yaw,
                         float speed, float latitude, float longitude);

/**
 * @brief Get alert type as string
 * @param type Alert type
 * @return Alert type string
 */
const char* alert_type_to_string(alert_type_t type);

/**
 * @brief Get MQTT topic for alert type
 * @param type Alert type
 * @return MQTT topic string
 */
const char* alert_type_to_topic(alert_type_t type);

#ifdef __cplusplus
}
#endif