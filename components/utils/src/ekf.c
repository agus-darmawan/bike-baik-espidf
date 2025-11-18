#include "ekf.h"
#include <math.h>
#include <string.h>
static const float g_ned[3] = {0.0f, 0.0f, 9.80665f};
void ekf_init(ekf_t *ekf, double lat0, double lon0, double alt0) {
    memset(ekf, 0, sizeof(ekf_t));
    // initialize covariances
    for(int i=0;i<EKF_STATE_DIM;i++) for(int j=0;j<EKF_STATE_DIM;j++) ekf->P[i][j]=0.0f;
    ekf->P[0][0]=ekf->P[1][1]=ekf->P[2][2]=25.0f;
    ekf->P[3][3]=ekf->P[4][4]=ekf->P[5][5]=1.0f;
    ekf->P[6][6]=ekf->P[7][7]=ekf->P[8][8]=0.1f;
}
void ekf_predict(ekf_t *ekf, float ax, float ay, float az, float dt, float Rbn[3][3]) {
    float p[3], v[3], ba[3];
    for(int i=0;i<3;i++){ p[i]=ekf->x[i]; v[i]=ekf->x[3+i]; ba[i]=ekf->x[6+i]; }
    float a_body[3] = {ax - ba[0], ay - ba[1], az - ba[2]};
    float a_nav[3];
    for(int i=0;i<3;i++) a_nav[i] = Rbn[i][0]*a_body[0] + Rbn[i][1]*a_body[1] + Rbn[i][2]*a_body[2];
    for(int i=0;i<3;i++){
        v[i] += (a_nav[i] + g_ned[i]) * dt;
        p[i] += v[i] * dt + 0.5f * (a_nav[i] + g_ned[i]) * dt * dt;
    }
    for(int i=0;i<3;i++){ ekf->x[i]=p[i]; ekf->x[3+i]=v[i]; }
    // small Q injection
    const float q_pos = 1e-3f, q_vel = 1e-2f, q_ba = 1e-6f;
    for(int i=0;i<3;i++) ekf->P[i][i] += q_pos;
    for(int i=3;i<6;i++) ekf->P[i][i] += q_vel;
    for(int i=6;i<9;i++) ekf->P[i][i] += q_ba;
}
void ekf_update_gps_pos(ekf_t *ekf, float pn, float pe, float pd, float Rgps) {
    float y[3];
    y[0]=pn-ekf->x[0]; y[1]=pe-ekf->x[1]; y[2]=pd-ekf->x[2];
    float Sdiag[3];
    for(int i=0;i<3;i++) Sdiag[i] = ekf->P[i][i] + Rgps;
    // approximate K = P(:,pos)/Sdiag
    float dx[EKF_STATE_DIM]; memset(dx,0,sizeof(dx));
    for(int r=0;r<EKF_STATE_DIM;r++){
        for(int c=0;c<3;c++) dx[r] += ekf->P[r][c] * (y[c] / Sdiag[c]);
    }
    for(int i=0;i<EKF_STATE_DIM;i++) ekf->x[i] += dx[i];
    for(int i=0;i<3;i++) ekf->P[i][i] *= 0.1f;
}
void ekf_update_gps_vel(ekf_t *ekf, float vn, float ve, float vd, float Rvel) {
    float y[3] = { vn - ekf->x[3], ve - ekf->x[4], vd - ekf->x[5] };
    for(int i=0;i<3;i++) ekf->x[3+i] += 0.5f * y[i];
    for(int i=3;i<6;i++) ekf->P[i][i] *= 0.2f;
}
void ekf_update_speed_sensor(ekf_t *ekf, float speed_forward, float Rspeed, float Rbn[3][3]) {
    float vnav[3] = { ekf->x[3], ekf->x[4], ekf->x[5] };
    float Rnb[3][3];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) Rnb[i][j]=Rbn[j][i];
    float vbody_x = Rnb[0][0]*vnav[0] + Rnb[0][1]*vnav[1] + Rnb[0][2]*vnav[2];
    float y = speed_forward - vbody_x;
    float gain = 0.5f;
    float col0[3] = { Rbn[0][0], Rbn[1][0], Rbn[2][0] };
    for(int i=0;i<3;i++) ekf->x[3+i] += gain * col0[i] * y;
    for(int i=3;i<6;i++) ekf->P[i][i] *= 0.9f;
}
