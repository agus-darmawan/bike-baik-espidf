#include "mahony.h"
#include <math.h>
static float q0=1, q1=0, q2=0, q3=0;
static float Ki = 0.0f, Kp = 0.5f;
void mahony_init(float kp, float ki){ Kp = kp; Ki = ki; q0=1; q1=q2=q3=0; }
// simplified Mahony that outputs Rbn (rotation body->nav)
void mahony_update(float gx,float gy,float gz,float ax,float ay,float az,float dt,float Rbn[3][3]) {
    // For brevity: use small-angle integration with normalization (not full Mahony).
    // Integrate gyro to update quaternion
    float gx_r = gx * 0.5f * dt;
    float gy_r = gy * 0.5f * dt;
    float gz_r = gz * 0.5f * dt;
    float nq0 = q0 - q1*gx_r - q2*gy_r - q3*gz_r;
    float nq1 = q1 + q0*gx_r + q2*gz_r - q3*gy_r;
    float nq2 = q2 + q0*gy_r - q1*gz_r + q3*gx_r;
    float nq3 = q3 + q0*gz_r + q1*gy_r - q2*gx_r;
    // normalize
    float norm = sqrtf(nq0*nq0 + nq1*nq1 + nq2*nq2 + nq3*nq3);
    if(norm>1e-6f){ q0=nq0/norm; q1=nq1/norm; q2=nq2/norm; q3=nq3/norm; }
    // build Rbn (body -> nav) rotation matrix
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
