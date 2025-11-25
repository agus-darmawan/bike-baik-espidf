#include "orientation.h"
#include <math.h>

void madgwick_init(madgwick_filter_t *filter, float32_t beta) {
    if (!filter) return;
    
    filter->beta = beta;
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;
}

void madgwick_update(madgwick_filter_t *filter, float32_t gx, float32_t gy, float32_t gz, 
                     float32_t ax, float32_t ay, float32_t az, float32_t dt, float32_t Rbn[3][3]) {
    if (!filter) return;
    
    float32_t q0 = filter->q0, q1 = filter->q1, q2 = filter->q2, q3 = filter->q3;
    
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        float recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        float s0 = 0, s1 = 0, s2 = 0, s3 = 0;
        s0 = 2*q0*q2*q2 + 2*q2*ax + 2*q0*q1*q1 - 2*q1*ay;
        s1 = 2*q1*q3*q3 - 2*q3*ax + 4*q0*q0*q1 - 2*q0*ay;
        s2 = 4*q0*q0*q2 + 2*q0*ax - 4*q2 + 2*q2*az;
        s3 = 4*q1*q1*q3 - 2*q1*ax;

        float norm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;

        float qDot1 = 0.5f*(-q1*gx - q2*gy - q3*gz) - filter->beta*s0;
        float qDot2 = 0.5f*( q0*gx + q2*gz - q3*gy) - filter->beta*s1;
        float qDot3 = 0.5f*( q0*gy - q1*gz + q3*gx) - filter->beta*s2;
        float qDot4 = 0.5f*( q0*gz + q1*gy - q2*gx) - filter->beta*s3;

        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;
    } else {
        float qDot1 = 0.5f*(-q1*gx - q2*gy - q3*gz);
        float qDot2 = 0.5f*( q0*gx + q2*gz - q3*gy);
        float qDot3 = 0.5f*( q0*gy - q1*gz + q3*gx);
        float qDot4 = 0.5f*( q0*gz + q1*gy - q2*gx);
        
        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;
    }

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

void madgwick_get_rpy(float32_t R[3][3], float32_t *roll, float32_t *pitch, float32_t *yaw) {
    *roll = atan2f(R[2][1], R[2][2]) * 57.29578f;
    *pitch = -asinf(fmaxf(-1.0f, fminf(1.0f, R[2][0]))) * 57.29578f;
    *yaw = atan2f(R[1][0], R[0][0]) * 57.29578f;
}