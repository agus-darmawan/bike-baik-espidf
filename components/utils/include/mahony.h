#pragma once
void mahony_init(float kp, float ki);
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt, float Rbn[3][3]);
