#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef float fp32;

typedef struct
{
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 max_out;   // absolute limit for total output
    fp32 max_iout;  // absolute limit for integral term
    fp32 max_dout;  // absolute limit for derivative term

    fp32 dT;        // sample time in seconds

    fp32 error[3];
    fp32 Dbuf[3];
} pid_type_def;

void PID_init_with_dt(pid_type_def *pid,
                      const fp32 PID[3],
                      fp32 max_out,
                      fp32 max_iout,
                      fp32 max_dout,
                      fp32 dT);

fp32 PID_calc_with_dt(pid_type_def *pid, fp32 input, fp32 target);

void PID_clear(pid_type_def *pid);

#endif // PID_H

