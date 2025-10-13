#include "main.h"        // HAL types
#include "control.h"     // control APIs
#include "motor.h"       // motor_set_speeds, encoders
#include "pid.h"         // PID implementation

// TIM5 handle comes from main.c
extern TIM_HandleTypeDef htim5;

// ----- Control configuration -----
// PID runs at 100 Hz (dT = 0.01 s)
#define CONTROL_DT_S           (0.01f)

// Output is PWM percent in [-100, 100]
#define PID_MAX_OUT            (100.0f)
#define PID_MAX_IOUT           (40.0f)   // clamp integral to avoid windup
#define PID_MAX_DOUT           (40.0f)

// Initial guess for speed loop gains (percent per tick units)
// Tune on your hardware (see notes in control.h)
#define PID_KP                 (10.0f)
#define PID_KI                 (0.003f)
#define PID_KD                 (0.01f)

static pid_type_def pid_left;
static pid_type_def pid_right;

// Targets expressed as encoder ticks per control period (10 ms)
static volatile int32_t target_left_ticks_dt = 0;
static volatile int32_t target_right_ticks_dt = 0;
static volatile int32_t meas_left_ticks_dt = 0;
static volatile int32_t meas_right_ticks_dt = 0;
static volatile int8_t last_cmd_left = 0;
static volatile int8_t last_cmd_right = 0;

// Last encoder counts for speed measurement
static int32_t last_left_counts = 0;
static int32_t last_right_counts = 0;
static uint8_t first_sample = 1;

// Foreground scheduling flag: set by TIM5 ISR, consumed in main loop/task
static volatile uint8_t control_due = 0;

uint8_t control_is_due(void) { return control_due; }
void control_clear_due(void) { control_due = 0; }

void control_init(void)
{
    const fp32 gains[3] = { PID_KP, PID_KI, PID_KD };
    PID_init_with_dt(&pid_left, gains, PID_MAX_OUT, PID_MAX_IOUT, PID_MAX_DOUT, CONTROL_DT_S);
    PID_init_with_dt(&pid_right, gains, PID_MAX_OUT, PID_MAX_IOUT, PID_MAX_DOUT, CONTROL_DT_S);

    // Initialize encoder baselines
    last_left_counts = motor_get_left_encoder_counts();
    last_right_counts = motor_get_right_encoder_counts();
    first_sample = 1;

    // Start TIM5 at 100 Hz in interrupt mode
    HAL_TIM_Base_Start_IT(&htim5);
}

void control_set_target_ticks_per_dt(int32_t left_ticks, int32_t right_ticks)
{
    // If direction flips (sign change), clear integrators to avoid fighting previous windup
    if ((left_ticks == 0 || target_left_ticks_dt == 0) ? 0 : ((left_ticks > 0) != (target_left_ticks_dt > 0))) {
        PID_clear(&pid_left);
    }
    if ((right_ticks == 0 || target_right_ticks_dt == 0) ? 0 : ((right_ticks > 0) != (target_right_ticks_dt > 0))) {
        PID_clear(&pid_right);
    }

    // If transitioning to zero target from non-zero, clear integrators to prevent residual drive
    if (left_ticks == 0 && target_left_ticks_dt != 0) {
        PID_clear(&pid_left);
    }
    if (right_ticks == 0 && target_right_ticks_dt != 0) {
        PID_clear(&pid_right);
    }
    target_left_ticks_dt = left_ticks;
    target_right_ticks_dt = right_ticks;
}

void control_sync_encoders(void)
{
    // When another module resets encoder counters, realign our baselines
    // and skip one sample to avoid a large spurious delta fighting the PID.
    last_left_counts = motor_get_left_encoder_counts();
    last_right_counts = motor_get_right_encoder_counts();
    first_sample = 1;
}

// Runs at 100 Hz from TIM5 ISR
void control_step(void)
{
    // Measure delta counts every 10 ms
    int32_t cur_left = motor_get_left_encoder_counts();
    int32_t cur_right = motor_get_right_encoder_counts();

    int32_t dl = cur_left - last_left_counts;
    int32_t dr = cur_right - last_right_counts;

    last_left_counts = cur_left;
    last_right_counts = cur_right;

    if (first_sample) {
        // Skip control on first sample (no valid speed yet)
        first_sample = 0;
        return;
    }

    // Latch measured values for diagnostics/UI
    meas_left_ticks_dt = dl;
    meas_right_ticks_dt = dr;

    // If targets are zero, unconditionally stop and clear integrators to avoid creep
    if (target_left_ticks_dt == 0 && target_right_ticks_dt == 0) {
        PID_clear(&pid_left);
        PID_clear(&pid_right);
        last_cmd_left = 0;
        last_cmd_right = 0;
        motor_stop();
        return;
    }

    // Compute PID outputs (PWM percent)
    float out_l = PID_calc_with_dt(&pid_left, (float)dl, (float)target_left_ticks_dt);
    float out_r = PID_calc_with_dt(&pid_right, (float)dr, (float)target_right_ticks_dt);

    // Apply a small deadband so tiny outputs don't map to 54% PWM minimum
    int8_t cmd_l = (int8_t)(out_l);
    int8_t cmd_r = (int8_t)(out_r);
    const int8_t pwm_deadband = 5; // percent
    if (cmd_l < pwm_deadband && cmd_l > -pwm_deadband) cmd_l = 0;
    if (cmd_r < pwm_deadband && cmd_r > -pwm_deadband) cmd_r = 0;
    last_cmd_left = cmd_l;
    last_cmd_right = cmd_r;
    motor_set_speeds(cmd_l, cmd_r);
}

// HAL weak callback override: dispatch TIM5 @ 100 Hz
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5) {
        control_due = 1;   // signal foreground to run control_step() at 100 Hz
    }
}

void control_get_target_and_measured(int32_t *t_left,
                                     int32_t *t_right,
                                     int32_t *m_left,
                                     int32_t *m_right)
{
    if (t_left)  *t_left  = target_left_ticks_dt;
    if (t_right) *t_right = target_right_ticks_dt;
    if (m_left)  *m_left  = meas_left_ticks_dt;
    if (m_right) *m_right = meas_right_ticks_dt;
}

void control_get_outputs(int8_t *out_left, int8_t *out_right)
{
    if (out_left)  *out_left = last_cmd_left;
    if (out_right) *out_right = last_cmd_right;
}
