#include "orientation.h"
#include <math.h>

void mahony_init(mahony_filter_t *filter, float32_t kp, float32_t ki) {
    if (!filter) return;
    
    filter->kp = kp;
    filter->ki = ki;
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;
    filter->integralFBx = 0.0f;
    filter->integralFBy = 0.0f;
    filter->integralFBz = 0.0f;
}

void mahony_update(mahony_filter_t *filter, float32_t gx, float32_t gy, float32_t gz, 
                   float32_t ax, float32_t ay, float32_t az, float32_t dt, float32_t Rbn[3][3]) {
    if (!filter) return;
    
    float32_t q0 = filter->q0, q1 = filter->q1, q2 = filter->q2, q3 = filter->q3;
    
    if (!(ax == 0 && ay == 0 && az == 0)) {
        float recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        float halfvx = q1*q3 - q0*q2;
        float halfvy = q0*q1 + q2*q3;
        float halfvz = q0*q0 - 0.5f + q3*q3;

        float halfex = ay*halfvz - az*halfvy;
        float halfey = az*halfvx - ax*halfvz;
        float halfez = ax*halfvy - ay*halfvx;

        if (filter->ki > 0) {
            filter->integralFBx += filter->ki * halfex * dt;
            filter->integralFBy += filter->ki * halfey * dt;
            filter->integralFBz += filter->ki * halfez * dt;
            gx += filter->integralFBx;
            gy += filter->integralFBy;
            gz += filter->integralFBz;
        } else {
            filter->integralFBx = 0.0f;
            filter->integralFBy = 0.0f;
            filter->integralFBz = 0.0f;
        }

        gx += filter->kp * halfex;
        gy += filter->kp * halfey;
        gz += filter->kp * halfez;
    }

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    
    float qa = q0, qb = q1, qc = q2;
    q0 += -qb*gx - qc*gy - q3*gz;
    q1 += qa*gx + qc*gz - q3*gy;
    q2 += qa*gy - qb*gz + q3*gx;
    q3 += qa*gz + qb*gy - qc*gx;

    float recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    filter->q0 = q0 * recipNorm;
    filter->q1 = q1 * recipNorm;
    filter->q2 = q2 * recipNorm;
    filter->q3 = q3 * recipNorm;

    // Compute rotation matrix
    q0 = filter->q0;
    q1 = filter->q1;
    q2 = filter->q2;
    q3 = filter->q3;
    
    Rbn[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    Rbn[0][1] = 2*(q1*q2 - q0*q3);
    Rbn[0][2] = 2*(q1*q3 + q0*q2);
    Rbn[1][0] = 2*(q1*q2 + q0*q3);
    Rbn[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    Rbn[1][2] = 2*(q2*q3 - q0*q1);
    Rbn[2][0] = 2*(q1*q3 - q0*q2);
    Rbn[2][1] = 2*(q2*q3 + q0*q1);
    Rbn[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
}

void mahony_get_rpy(float32_t R[3][3], float32_t *roll, float32_t *pitch, float32_t *yaw) {
    *roll = atan2f(R[2][1], R[2][2]) * 57.29578f;
    *pitch = -asinf(fmaxf(-1.0f, fminf(1.0f, R[2][0]))) * 57.29578f;
    *yaw = atan2f(R[1][0], R[0][0]) * 57.29578f;
}