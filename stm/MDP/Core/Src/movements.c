// movements.c
#include "movements.h"
#include "motor.h"
#include "imu.h"
#include "pid.h"

// ===== Your existing helpers we reuse =====
extern void PID_SPEED_1(pid_type_def *pid, float angleNow, float *correction,
                        int8_t dir, uint16_t *newDutyL, uint16_t *newDutyR);
extern void PID_SPEED_2(pid_type_def *pid, float angleNow, float *correction,
                        int8_t dir, uint16_t *newDutyL, uint16_t *newDutyR);

// --- Config you said: 6.6 cm wheels, 330 PPR quadrature x2 (rising/rising) = 660 counts/rev ---
static const float WHEEL_CIRC_CM  = 3.1415926f * 6.6f;         // ~20.735 cm
static const float COUNTS_PER_REV = 660.0f;                    // your setting
static const float TICKS_PER_CM   = COUNTS_PER_REV / WHEEL_CIRC_CM; // ~31.8

// --- Motion state ---
typedef struct {
  uint8_t   active;
  int8_t    dir;                 // +1 forward, -1 backward
  speed_mode_t mode;             // SPEED_MODE_1 or _2
  int32_t   target_ticks;        // distance goal in encoder ticks (left-based)
  int32_t   start_left;
  float     angle_deg;           // integrated yaw (deg)
  // old-PID-style things:
  pid_type_def pidSlow, pidFast;
  uint8_t   pid_inited;
} move_state_t;

static move_state_t ms = {0};

// utility: clamp a value
static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void move_start_straight(float distance_cm, speed_mode_t mode)
{
  ms.active = 1;
  ms.mode   = mode;
  ms.dir    = (distance_cm >= 0.0f) ? 1 : -1;
  ms.angle_deg = 0.0f;

  // distance target (use left encoder only, per your request)
  motor_reset_encoders();
  ms.start_left   = motor_get_left_encoder_counts();
  ms.target_ticks = (int32_t)( (float)ms.dir * (fabsf(distance_cm) * TICKS_PER_CM) );

  // init old P-only “pid” containers once
  if (!ms.pid_inited) {
    float slow[3] = {2.5f, 0.0f, 0.0f}; // Kp, Ki, Kd (only Kp used)
    float fast[3] = {1.5f, 0.0f, 0.0f}; // Kp
    PID_init_with_dt(&ms.pidSlow, slow, 30.0f, 10.0f, 5.0f, 0.01f);
    PID_init_with_dt(&ms.pidFast, fast, 30.0f, 10.0f, 5.0f, 0.01f);
    ms.pid_inited = 1;
  }
}

void move_abort(void)
{
  ms.active = 0;
  motor_stop();
}

uint8_t move_is_active(void)
{
  return ms.active;
}

// Call this exactly every 10 ms (100 Hz)
void move_tick_100Hz(void)
{
  if (!ms.active) return;

  // 1) Update yaw by integrating gyro at 100 Hz
  imu_update_yaw_100Hz();               // updates internal yaw rate
  // For “friend style” angleNow, add the 10 ms contribution from instant rate (°/s):
  float gz_dps;
  Gyro_Read_Z(NULL, &gz_dps);           // uses internal handle; returns °/s (bias-removed, deadbanded)
  ms.angle_deg += gz_dps * 0.01f;       // integrate

  // 2) Check distance using ONLY LEFT encoder (as you asked)
  int32_t left_now = motor_get_left_encoder_counts();
  int32_t left_delta = left_now - ms.start_left;

  // stop when reached (allow both forward/backward by sign)
  if ((ms.dir > 0 && left_delta >= ms.target_ticks) ||
      (ms.dir < 0 && left_delta <= ms.target_ticks))
  {
    motor_stop();
    ms.active = 0;
    return;
  }

  // 3) Compute wheel PWM using your old functions (P-only)
  float correction = 0.0f;
  uint16_t newDutyL = 0, newDutyR = 0;

  if (ms.mode == SPEED_MODE_1) {
    PID_SPEED_1(&ms.pidSlow, ms.angle_deg, &correction, ms.dir, &newDutyL, &newDutyR);
  } else {
    PID_SPEED_2(&ms.pidFast, ms.angle_deg, &correction, ms.dir, &newDutyL, &newDutyR);
  }

  // 4) Convert those “friend-style” servo-like PWM (e.g., 1000..2200) into your motor_set_speeds()
  // Your friend’s functions clamp between ~[600..2200] or [1000..2200]. Map linearly to ±100%.
  // Example: treat 1000 as “0%”, 2200 as “+120%”. You used (newDuty - 1000)/10 in your test.
  // Let’s keep that simple mapping but clamp to [-100, 100]:

  int16_t l_pct = (int16_t)((int32_t)newDutyL - 1000) / 10; // ≈0..120
  int16_t r_pct = (int16_t)((int32_t)newDutyR - 1000) / 10; // ≈0..120

  if (l_pct > 100) l_pct = 100;
  if (r_pct > 100) r_pct = 100;
  if (l_pct < -100) l_pct = -100;
  if (r_pct < -100) r_pct = -100;

  motor_set_speeds((int8_t)l_pct, (int8_t)r_pct);
}