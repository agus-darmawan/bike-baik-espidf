#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef float float32_t;

/**
 * @brief Mahony filter structure for orientation estimation
 */
typedef struct {
    float32_t q0, q1, q2, q3;                  /**< Quaternion */
    float32_t kp, ki;                          /**< Proportional and integral gains */
    float32_t integralFBx, integralFBy, integralFBz; /**< Integral feedback */
} mahony_filter_t;

/**
 * @brief Madgwick filter structure for orientation estimation
 */
typedef struct {
    float32_t q0, q1, q2, q3;  /**< Quaternion */
    float32_t beta;            /**< Filter gain */
} madgwick_filter_t;

/**
 * @brief Initialize Mahony filter for orientation estimation.
 * 
 * @param filter Pointer to Mahony filter structure
 * @param kp Proportional gain (typical: 2.0)
 * @param ki Integral gain (typical: 0.0 for no drift correction, 0.01 with)
 */
void mahony_init(mahony_filter_t *filter, float32_t kp, float32_t ki);

/**
 * @brief Update Mahony filter using gyroscope and accelerometer data.
 * 
 * @param filter Pointer to Mahony filter structure
 * @param gx Gyroscope x in rad/s
 * @param gy Gyroscope y in rad/s
 * @param gz Gyroscope z in rad/s
 * @param ax Accelerometer x in m/s^2
 * @param ay Accelerometer y in m/s^2
 * @param az Accelerometer z in m/s^2
 * @param dt Time step in seconds
 * @param Rbn Output rotation matrix (3x3)
 */
void mahony_update(mahony_filter_t *filter, float32_t gx, float32_t gy, float32_t gz, 
                   float32_t ax, float32_t ay, float32_t az, float32_t dt, float32_t Rbn[3][3]);

/**
 * @brief Get Roll, Pitch, Yaw angles from rotation matrix.
 * 
 * @param R 3x3 rotation matrix from filter
 * @param roll Output roll angle in degrees
 * @param pitch Output pitch angle in degrees
 * @param yaw Output yaw angle in degrees
 */
void mahony_get_rpy(float32_t R[3][3], float32_t *roll, float32_t *pitch, float32_t *yaw);

/**
 * @brief Initialize Madgwick filter for orientation estimation.
 * 
 * @param filter Pointer to Madgwick filter structure
 * @param beta Filter gain (typical: 0.1 - 0.6, lower = more stable but slower)
 */
void madgwick_init(madgwick_filter_t *filter, float32_t beta);

/**
 * @brief Update Madgwick filter using gyroscope and accelerometer data.
 * 
 * @param filter Pointer to Madgwick filter structure
 * @param gx Gyroscope x in rad/s
 * @param gy Gyroscope y in rad/s
 * @param gz Gyroscope z in rad/s
 * @param ax Accelerometer x in m/s^2
 * @param ay Accelerometer y in m/s^2
 * @param az Accelerometer z in m/s^2
 * @param dt Time step in seconds
 * @param Rbn Output rotation matrix (3x3)
 */
void madgwick_update(madgwick_filter_t *filter, float32_t gx, float32_t gy, float32_t gz, 
                     float32_t ax, float32_t ay, float32_t az, float32_t dt, float32_t Rbn[3][3]);

/**
 * @brief Get Roll, Pitch, Yaw angles from rotation matrix.
 * 
 * @param R 3x3 rotation matrix from filter
 * @param roll Output roll angle in degrees
 * @param pitch Output pitch angle in degrees
 * @param yaw Output yaw angle in degrees
 */
void madgwick_get_rpy(float32_t R[3][3], float32_t *roll, float32_t *pitch, float32_t *yaw);

#ifdef __cplusplus
}
#endif