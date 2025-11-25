/**
 * @file kalman_filter.h
 * @author @agus-darmawan
 * @brief Extended Kalman Filter for sensor fusion (GPS + IMU + Odometry)
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
 * @brief Vehicle state structure
 */
typedef struct {
    float x;              /**< Position X (meters) */
    float y;              /**< Position Y (meters) */
    float vx;             /**< Velocity X (m/s) */
    float vy;             /**< Velocity Y (m/s) */
    float heading;        /**< Heading angle (radians) */
    float yaw_rate;       /**< Yaw rate (rad/s) */
} vehicle_state_t;

/**
 * @brief GPS measurement
 */
typedef struct {
    float latitude;       /**< Latitude (degrees) */
    float longitude;      /**< Longitude (degrees) */
    float altitude;       /**< Altitude (meters) */
    float speed;          /**< Speed (m/s) */
    float course;         /**< Course (degrees) */
    bool valid;           /**< Measurement valid flag */
} gps_measurement_t;

/**
 * @brief IMU measurement
 */
typedef struct {
    float ax;             /**< Acceleration X (m/s²) */
    float ay;             /**< Acceleration Y (m/s²) */
    float az;             /**< Acceleration Z (m/s²) */
    float gx;             /**< Gyro X (rad/s) */
    float gy;             /**< Gyro Y (rad/s) */
    float gz;             /**< Gyro Z (rad/s) */
    float roll;           /**< Roll angle (degrees) */
    float pitch;          /**< Pitch angle (degrees) */
    float yaw;            /**< Yaw angle (degrees) */
} imu_measurement_t;

/**
 * @brief Odometry measurement
 */
typedef struct {
    float distance;       /**< Distance traveled (meters) */
    float speed;          /**< Speed from wheel (m/s) */
    bool valid;           /**< Measurement valid flag */
} odometry_measurement_t;

/**
 * @brief Kalman filter configuration
 */
typedef struct {
    float process_noise_pos;      /**< Process noise for position */
    float process_noise_vel;      /**< Process noise for velocity */
    float process_noise_heading;  /**< Process noise for heading */
    
    float measurement_noise_gps;  /**< Measurement noise for GPS */
    float measurement_noise_imu;  /**< Measurement noise for IMU */
    float measurement_noise_odo;  /**< Measurement noise for odometry */
} kalman_config_t;

/**
 * @brief Initialize Kalman filter
 * @param config Pointer to configuration structure
 */
void kalman_init(const kalman_config_t *config);

/**
 * @brief Predict step (time update)
 * @param dt Time delta in seconds
 */
void kalman_predict(float dt);

/**
 * @brief Update with GPS measurement
 * @param gps GPS measurement
 */
void kalman_update_gps(const gps_measurement_t *gps);

/**
 * @brief Update with IMU measurement
 * @param imu IMU measurement
 */
void kalman_update_imu(const imu_measurement_t *imu);

/**
 * @brief Update with odometry measurement
 * @param odo Odometry measurement
 */
void kalman_update_odometry(const odometry_measurement_t *odo);

/**
 * @brief Get current vehicle state estimate
 * @param state Pointer to state structure to fill
 */
void kalman_get_state(vehicle_state_t *state);

/**
 * @brief Convert GPS to local XY coordinates
 * @param lat Latitude (degrees)
 * @param lon Longitude (degrees)
 * @param x Output X coordinate (meters)
 * @param y Output Y coordinate (meters)
 */
void gps_to_xy(float lat, float lon, float *x, float *y);

/**
 * @brief Set GPS origin for coordinate conversion
 * @param lat Origin latitude (degrees)
 * @param lon Origin longitude (degrees)
 */
void set_gps_origin(float lat, float lon);

#ifdef __cplusplus
}
#endif
