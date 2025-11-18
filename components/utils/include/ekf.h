#pragma once
#include <stdint.h>
#define EKF_STATE_DIM 9
typedef struct {
    float x[EKF_STATE_DIM];
    float P[EKF_STATE_DIM][EKF_STATE_DIM];
} ekf_t;
void ekf_init(ekf_t *ekf, double lat0, double lon0, double alt0);
void ekf_predict(ekf_t *ekf, float ax, float ay, float az, float dt, float Rbn[3][3]);
void ekf_update_gps_pos(ekf_t *ekf, float pn, float pe, float pd, float Rgps);
void ekf_update_gps_vel(ekf_t *ekf, float vn, float ve, float vd, float Rvel);
void ekf_update_speed_sensor(ekf_t *ekf, float speed_forward, float Rspeed, float Rbn[3][3]);
