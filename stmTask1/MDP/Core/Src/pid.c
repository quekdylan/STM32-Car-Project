#include "pid.h"

#define CLAMP_ABS(val, max_abs) \
    do {                         \
        if ((val) > (max_abs)) { \
            (val) = (max_abs);   \
        } else if ((val) < -(max_abs)) { \
            (val) = -(max_abs);  \
        }                        \
    } while (0)

void PID_init_with_dt(pid_type_def *pid,
                      const fp32 PID[3],
                      fp32 max_out,
                      fp32 max_iout,
                      fp32 max_dout,
                      fp32 dT)
{
    if (!pid || !PID) return;

    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];

    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->max_dout = max_dout;
    pid->dT = dT;

    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->set = pid->fdb = 0.0f;
}

fp32 PID_calc_with_dt(pid_type_def *pid, fp32 input, fp32 target)
{
    if (!pid) return 0.0f;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = target;
    pid->fdb = input;
    pid->error[0] = target - input;

    // Proportional
    pid->Pout = pid->Kp * pid->error[0];

    // Integral (scaled by dt)
    pid->Iout += pid->Ki * pid->error[0] * pid->dT;
    CLAMP_ABS(pid->Iout, pid->max_iout);

    // Derivative based on error difference (scaled by dt)
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = pid->error[0] - pid->error[1];
    pid->Dout = pid->Kd * (pid->Dbuf[0] / (pid->dT > 0.0f ? pid->dT : 1.0f));
    CLAMP_ABS(pid->Dout, pid->max_dout);

    // Sum and clamp
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    CLAMP_ABS(pid->out, pid->max_out);

    return pid->out;
}

void PID_clear(pid_type_def *pid)
{
    if (!pid) return;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

void PID_SPEED_1(pid_type_def *pid, float angleNow, float *correction, int8_t dir,
    uint16_t *newDutyL, uint16_t *newDutyR)
{
// “Turn/trim” mode: target angle = 0 (straight), lower base, more authority
float error = 0.0f - angleNow;
*correction = pid->Kp * error;

// Lower base (~37% of 799 ≈ 300)
*newDutyL = 490 + (int16_t)(*correction * dir);
*newDutyR = 490 - (int16_t)(*correction * dir);

// Clamp to valid PWM range [0..799]
if (*newDutyL > 799) *newDutyL = 799;
if ((int)*newDutyL < 0) *newDutyL = 0;
if (*newDutyR > 799) *newDutyR = 799;
if ((int)*newDutyR < 0) *newDutyR = 0;
}

void PID_SPEED_2(pid_type_def *pid, float angleNow, float *correction, int8_t dir,
    uint16_t *newDutyL, uint16_t *newDutyR) 
{
// Fast mode: target angle = 0 (straight)
float error = 0.0f - angleNow;
*correction = pid->Kp * error;

// Apply correction around higher base duty (~65% of 799 ≈ 520)
*newDutyL = 520 + (int16_t)(*correction * dir);
*newDutyR = 520 - (int16_t)(*correction * dir);

// Clamp values to valid PWM range
if (*newDutyL > 799) *newDutyL = 799;
if (*newDutyL < 0)   *newDutyL = 0;
if (*newDutyR > 799) *newDutyR = 799;
if (*newDutyR < 0)   *newDutyR = 0;
}

void PID_SPEED_T(pid_type_def *pid, float angleNow, float *correction, int8_t dir,
    uint16_t *newDutyL, uint16_t *newDutyR)
{
// “Turn/trim” mode: target angle = 0 (straight), lower base, more authority
float error = 0.0f - angleNow;
*correction = pid->Kp * error;

// Lower base (~37% of 799 ≈ 300)
*newDutyL = 300 + (int16_t)(*correction * dir);
*newDutyR = 300 - (int16_t)(*correction * dir);

// Clamp to valid PWM range [0..799]
if (*newDutyL > 799) *newDutyL = 799;
if ((int)*newDutyL < 0) *newDutyL = 0;
if (*newDutyR > 799) *newDutyR = 799;
if ((int)*newDutyR < 0) *newDutyR = 0;
}