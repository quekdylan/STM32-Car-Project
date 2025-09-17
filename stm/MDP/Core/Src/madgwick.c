#include "madgwick.h"
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Tunables
static float s_beta = 0.1f;   // algorithm gain
static float s_dt   = 0.01f;  // sample period (s)

// Quaternion state (w, x, y, z)
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

static inline float inv_sqrt(float x){
    return 1.0f / sqrtf(x);
}

void madgwick_init(float sample_hz, float beta)
{
    if (sample_hz > 0.0f) s_dt = 1.0f / sample_hz;
    if (beta > 0.0f) s_beta = beta;
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
}

void madgwick_update_imu(float gx, float gy, float gz,
                         float ax, float ay, float az)
{
    // Normalize accelerometer
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm <= 1e-6f) {
        // Invalid accel; integrate gyro only
        float qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
        float qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
        float qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
        float qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);
        q0 += qDot1 * s_dt; q1 += qDot2 * s_dt; q2 += qDot3 * s_dt; q3 += qDot4 * s_dt;
    } else {
        ax /= norm; ay /= norm; az /= norm;

        // Auxiliary variables to avoid repeated operations
        float _2q0 = 2.0f*q0, _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
        float _4q0 = 4.0f*q0, _4q1 = 4.0f*q1, _4q2 = 4.0f*q2;
        float _8q1 = 8.0f*q1, _8q2 = 8.0f*q2;
        float q0q0 = q0*q0; float q1q1 = q1*q1; float q2q2 = q2*q2; float q3q3 = q3*q3;

        // Gradient decent algorithm corrective step
        float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        float s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        float s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        float s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;
        norm = inv_sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= norm; s1 *= norm; s2 *= norm; s3 *= norm;

        // Compute rate of change of quaternion
        float qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz) - s_beta*s0;
        float qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy) - s_beta*s1;
        float qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx) - s_beta*s2;
        float qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx) - s_beta*s3;

        // Integrate to yield quaternion
        q0 += qDot1 * s_dt; q1 += qDot2 * s_dt; q2 += qDot3 * s_dt; q3 += qDot4 * s_dt;
    }

    // Normalize quaternion
    float n = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= n; q1 *= n; q2 *= n; q3 *= n;
}

void madgwick_get_quat(float *qw, float *qx, float *qy, float *qz)
{
    if (qw) *qw = q0; if (qx) *qx = q1; if (qy) *qy = q2; if (qz) *qz = q3;
}

void madgwick_get_euler_deg(float *roll, float *pitch, float *yaw)
{
    // ZYX convention (yaw around Z)
    float sinr_cosp = 2.0f * (q0*q1 + q2*q3);
    float cosr_cosp = 1.0f - 2.0f * (q1*q1 + q2*q2);
    float r = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (q0*q2 - q3*q1);
    float p = (fabsf(sinp) >= 1.0f) ? copysignf(M_PI/2.0f, sinp) : asinf(sinp);

    float siny_cosp = 2.0f * (q0*q3 + q1*q2);
    float cosy_cosp = 1.0f - 2.0f * (q2*q2 + q3*q3);
    float y = atan2f(siny_cosp, cosy_cosp);

    if (roll)  *roll  = r * 57.2957795f;
    if (pitch) *pitch = p * 57.2957795f;
    if (yaw)   *yaw   = y * 57.2957795f;
}

void madgwick_set_beta(float beta)
{
    if (beta > 0.0f && beta < 1.0f) s_beta = beta;
}
