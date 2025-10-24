#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize filter with sample rate (Hz) and beta gain (typ 0.05..0.2)
void madgwick_init(float sample_hz, float beta);

// Update with IMU data (no magnetometer)
// gx,gy,gz in rad/s, ax,ay,az unitless, normalized to 1g magnitude preferred
void madgwick_update_imu(float gx, float gy, float gz,
                         float ax, float ay, float az);

// Get quaternion (w,x,y,z)
void madgwick_get_quat(float *qw, float *qx, float *qy, float *qz);

// Get Euler angles in degrees
void madgwick_get_euler_deg(float *roll, float *pitch, float *yaw);

// Optionally tune beta (0.03..0.2 typical): higher trusts accel more
void madgwick_set_beta(float beta);

#ifdef __cplusplus
}
#endif
