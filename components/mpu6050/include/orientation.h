#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef float float32_t;

typedef struct {
    float32_t q0, q1, q2, q3;
    float32_t kp, ki;
    float32_t integralFBx, integralFBy, integralFBz;
} mahony_filter_t;

typedef struct {
    float32_t q0, q1, q2, q3;
    float32_t beta;
} madgwick_filter_t;

/**
 * Initialize Mahony filter for orientation estimation.
 * @param filter Pointer to Mahony filter structure
 * @param kp Proportional gain
 * @param ki Integral gain
 */
void mahony_init(mahony_filter_t *filter, float32_t kp, float32_t ki);

/**
 * Update Mahony filter using gyroscope and accelerometer data.
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
 * Get Roll, Pitch, Yaw angles from rotation matrix.
 * @param R 3x3 rotation matrix from filter
 * @param roll Output roll angle in degrees
 * @param pitch Output pitch angle in degrees
 * @param yaw Output yaw angle in degrees
 */
void mahony_get_rpy(float32_t R[3][3], float32_t *roll, float32_t *pitch, float32_t *yaw);

/**
 * Initialize Madgwick filter for orientation estimation.
 * @param filter Pointer to Madgwick filter structure
 * @param beta Filter gain
 */
void madgwick_init(madgwick_filter_t *filter, float32_t beta);

/**
 * Update Madgwick filter using gyroscope and accelerometer data.
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
 * Get Roll, Pitch, Yaw angles from rotation matrix.
 * @param R 3x3 rotation matrix from filter
 * @param roll Output roll angle in degrees
 * @param pitch Output pitch angle in degrees
 * @param yaw Output yaw angle in degrees
 */
void madgwick_get_rpy(float32_t R[3][3], float32_t *roll, float32_t *pitch, float32_t *yaw);

#ifdef __cplusplus
}
#endif
