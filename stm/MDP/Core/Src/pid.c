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

