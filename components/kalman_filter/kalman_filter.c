#include "kalman_filter.h"
#include <math.h>
#include <string.h>

#define STATE_DIM 6    // [x, y, vx, vy, heading, yaw_rate]
#define PI 3.14159265358979323846

// GPS origin for coordinate conversion
static struct {
    float lat0;
    float lon0;
    bool initialized;
} gps_origin = {0};

// Kalman filter state
static struct {
    float x[STATE_DIM];           // State vector
    float P[STATE_DIM][STATE_DIM]; // Covariance matrix
    
    // Configuration
    float Q_pos;
    float Q_vel;
    float Q_heading;
    float R_gps;
    float R_imu;
    float R_odo;
    
    bool initialized;
} kf = {0};

/* ===== GPS Coordinate Conversion ===== */

void set_gps_origin(float lat, float lon)
{
    gps_origin.lat0 = lat;
    gps_origin.lon0 = lon;
    gps_origin.initialized = true;
}

void gps_to_xy(float lat, float lon, float *x, float *y)
{
    if (!gps_origin.initialized) {
        set_gps_origin(lat, lon);
        *x = 0.0f;
        *y = 0.0f;
        return;
    }
    
    // Simple equirectangular projection
    // Good for small distances (<10km)
    const float R_earth = 6371000.0f; // Earth radius in meters
    
    float lat_rad = lat * PI / 180.0f;
    float lon_rad = lon * PI / 180.0f;
    float lat0_rad = gps_origin.lat0 * PI / 180.0f;
    float lon0_rad = gps_origin.lon0 * PI / 180.0f;
    
    *x = (lon_rad - lon0_rad) * cos((lat_rad + lat0_rad) / 2.0f) * R_earth;
    *y = (lat_rad - lat0_rad) * R_earth;
}

/* ===== Matrix Operations (Simplified) ===== */

static void matrix_identity(float mat[STATE_DIM][STATE_DIM])
{
    memset(mat, 0, sizeof(float) * STATE_DIM * STATE_DIM);
    for (int i = 0; i < STATE_DIM; i++) {
        mat[i][i] = 1.0f;
    }
}

/* ===== Kalman Filter Implementation ===== */

void kalman_init(const kalman_config_t *config)
{
    memset(&kf, 0, sizeof(kf));
    
    // Set configuration
    kf.Q_pos = config ? config->process_noise_pos : 0.1f;
    kf.Q_vel = config ? config->process_noise_vel : 0.1f;
    kf.Q_heading = config ? config->process_noise_heading : 0.01f;
    kf.R_gps = config ? config->measurement_noise_gps : 5.0f;
    kf.R_imu = config ? config->measurement_noise_imu : 0.1f;
    kf.R_odo = config ? config->measurement_noise_odo : 0.5f;
    
    // Initialize covariance matrix
    matrix_identity(kf.P);
    for (int i = 0; i < STATE_DIM; i++) {
        kf.P[i][i] = 10.0f; // Initial uncertainty
    }
    
    kf.initialized = true;
}

void kalman_predict(float dt)
{
    if (!kf.initialized) return;
    
    // State prediction using simple motion model
    // x(k+1) = x(k) + vx*dt*cos(heading) - vy*dt*sin(heading)
    // y(k+1) = y(k) + vx*dt*sin(heading) + vy*dt*cos(heading)
    // vx(k+1) = vx(k)
    // vy(k+1) = vy(k)
    // heading(k+1) = heading(k) + yaw_rate*dt
    // yaw_rate(k+1) = yaw_rate(k)
    
    float heading = kf.x[4];
    float vx = kf.x[2];
    float vy = kf.x[3];
    float yaw_rate = kf.x[5];
    
    // Predict new state
    float x_new[STATE_DIM];
    x_new[0] = kf.x[0] + (vx * cosf(heading) - vy * sinf(heading)) * dt;
    x_new[1] = kf.x[1] + (vx * sinf(heading) + vy * cosf(heading)) * dt;
    x_new[2] = vx;
    x_new[3] = vy;
    x_new[4] = heading + yaw_rate * dt;
    x_new[5] = yaw_rate;
    
    // Normalize heading to [-pi, pi]
    while (x_new[4] > PI) x_new[4] -= 2.0f * PI;
    while (x_new[4] < -PI) x_new[4] += 2.0f * PI;
    
    memcpy(kf.x, x_new, sizeof(kf.x));
    
    // Predict covariance (simplified - add process noise to diagonal)
    kf.P[0][0] += kf.Q_pos * dt;
    kf.P[1][1] += kf.Q_pos * dt;
    kf.P[2][2] += kf.Q_vel * dt;
    kf.P[3][3] += kf.Q_vel * dt;
    kf.P[4][4] += kf.Q_heading * dt;
    kf.P[5][5] += kf.Q_heading * dt;
}

void kalman_update_gps(const gps_measurement_t *gps)
{
    if (!kf.initialized || !gps || !gps->valid) return;
    
    // Convert GPS to XY
    float x_meas, y_meas;
    gps_to_xy(gps->latitude, gps->longitude, &x_meas, &y_meas);
    
    // Measurement update (simplified Kalman gain calculation)
    float innovation_x = x_meas - kf.x[0];
    float innovation_y = y_meas - kf.x[1];
    
    // Kalman gain (simplified)
    float S_x = kf.P[0][0] + kf.R_gps;
    float S_y = kf.P[1][1] + kf.R_gps;
    float K_x = kf.P[0][0] / S_x;
    float K_y = kf.P[1][1] / S_y;
    
    // Update state
    kf.x[0] += K_x * innovation_x;
    kf.x[1] += K_y * innovation_y;
    
    // Update velocity from GPS speed and course
    if (gps->speed > 0.1f) {
        float course_rad = gps->course * PI / 180.0f;
        float vx_gps = gps->speed * cosf(course_rad);
        float vy_gps = gps->speed * sinf(course_rad);
        
        // Blend with current velocity estimate
        kf.x[2] = 0.7f * kf.x[2] + 0.3f * vx_gps;
        kf.x[3] = 0.7f * kf.x[3] + 0.3f * vy_gps;
    }
    
    // Update covariance (simplified)
    kf.P[0][0] *= (1.0f - K_x);
    kf.P[1][1] *= (1.0f - K_y);
}

void kalman_update_imu(const imu_measurement_t *imu)
{
    if (!kf.initialized || !imu) return;
    
    // Update heading from IMU yaw
    float yaw_rad = imu->yaw * PI / 180.0f;
    float innovation = yaw_rad - kf.x[4];
    
    // Normalize innovation
    while (innovation > PI) innovation -= 2.0f * PI;
    while (innovation < -PI) innovation += 2.0f * PI;
    
    // Kalman gain (simplified)
    float S = kf.P[4][4] + kf.R_imu;
    float K = kf.P[4][4] / S;
    
    // Update state
    kf.x[4] += K * innovation;
    
    // Update yaw rate from gyro Z
    kf.x[5] = 0.8f * kf.x[5] + 0.2f * imu->gz;
    
    // Update covariance
    kf.P[4][4] *= (1.0f - K);
}

void kalman_update_odometry(const odometry_measurement_t *odo)
{
    if (!kf.initialized || !odo || !odo->valid) return;
    
    // Update velocity magnitude from odometry
    float v_current = sqrtf(kf.x[2] * kf.x[2] + kf.x[3] * kf.x[3]);
    float v_odo = odo->speed;
    
    if (v_current > 0.01f) {
        // Scale current velocity to match odometry
        float scale = v_odo / v_current;
        kf.x[2] *= scale;
        kf.x[3] *= scale;
    } else if (v_odo > 0.01f) {
        // Use heading to set velocity direction
        kf.x[2] = v_odo * cosf(kf.x[4]);
        kf.x[3] = v_odo * sinf(kf.x[4]);
    }
}

void kalman_get_state(vehicle_state_t *state)
{
    if (!state) return;
    
    if (kf.initialized) {
        state->x = kf.x[0];
        state->y = kf.x[1];
        state->vx = kf.x[2];
        state->vy = kf.x[3];
        state->heading = kf.x[4];
        state->yaw_rate = kf.x[5];
    } else {
        memset(state, 0, sizeof(vehicle_state_t));
    }
}
