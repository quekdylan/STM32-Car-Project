// movements.c
#include "movements.h"
#include "motor.h"
#include "imu.h"
#include "control.h" // Assumes this provides control_set_target_ticks_per_dt()
#include "servo.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdlib.h> // For abs()
#include <stdint.h>

// =================================================================================
// T U N A B L E   P A R A M E T E R S
// =================================================================================

// --- Motion Profile Configuration (Fixed distances in centimeters) ---
// You can change these at runtime via move_set_profile_distances_cm().
static float g_accel_dist_cm = 5.0f; // accelerate over first 5 cm
static float g_decel_dist_cm = 12.0f; // decelerate over last 12 cm

// --- Speed Configuration ---
// Speeds are defined in "encoder ticks per control period (10ms)".
// You must tune these for your specific robot.
#define V_MAX_TICKS_PER_DT (35.0f) // Max speed during cruise phase (e.g., 30 ticks / 10ms)
#define V_MIN_TICKS_PER_DT (3.0f)  // Minimum speed to overcome static friction and ensure smooth start/stop

// --- Yaw Correction ---
// Proportional gain for heading correction.
// A small value gently corrects the heading. Too large a value will cause oscillation.
// Units: (ticks/10ms) per degree of error.
#define YAW_KP_TICKS_PER_DEG (6.0f)  // Increased from 3.0f for better straight-line correction

// --- Turn Configuration ---
// Speed for ackermann turns (encoder ticks / 10ms per wheel after scaling).
#define TURN_BASE_TICKS_PER_DT   (25)     // base speed before inner/outer scaling
#define TURN_YAW_TARGET_DEG      (90.0f)  // desired turn angle
#define TURN_YAW_TOLERANCE_DEG   (0.2f)   // acceptable tolerance to stop
#define TURN_YAW_SLOW_BAND_DEG   (10.0f)  // start slowing when within this band
#define TURN_MIN_TICKS_PER_DT    (3)      // minimum per-wheel speed while turning
#define STEER_SETTLE_MS_PER_UNIT (2.0f)   // estimated ms per servo command unit of travel

// duration (in 100 Hz ticks) to hold/transition minimum speed at turn start to allow time for front wheels to turn
static uint16_t g_turn_spinup_ticks = 100;

// Ackermann geometry (cm) and steering calibration
#define WHEELBASE_CM             (14.5f)
#define TRACK_WIDTH_CM           (16.5f)
// Estimated max physical front steering angle (deg) at Servo_WriteAngle ±100.
#define STEER_MAX_DEG            (30.0f)

// Steering angles for Ackermann (Servo_WriteAngle: -100=full left, +100=full right)
#define STEER_ANGLE_MAG          (100.0f)  // magnitude for turning

// =================================================================================
// M O D U L E   S T A T E
// =================================================================================

// --- Physical Constants ---
static const float WHEEL_CIRC_CM  = 3.1415926f * 6.6f;
static const float COUNTS_PER_REV = 1560.0f;
static const float TICKS_PER_CM   = COUNTS_PER_REV / WHEEL_CIRC_CM;

// --- Motion State Machine ---
typedef enum {
  MOVE_IDLE = 0,
  MOVE_STRAIGHT,
  MOVE_TURN
} move_mode_e;

typedef struct {
  uint8_t   active;           // Is a movement in progress?
  move_mode_e mode;           // Straight or Turn
  int8_t    dir;              // Straight: +1 forward, -1 backward.

  // Straight move
  int32_t   total_ticks_abs;  // The target distance in absolute encoder ticks.
  int32_t   accel_end_ticks;
  int32_t   decel_start_ticks;

  // Turn move
  int8_t    turn_sign;        // +1 = left (CCW), -1 = right (CW)
  int8_t    drive_sign;       // +1 = forward, -1 = backward (during turn)
  float     yaw_target_deg;   // e.g., +/-90
  float     yaw_accum_deg;    // integrated yaw since turn start (unwrapped)
  float     prev_yaw_deg;     // previous wrapped yaw sample
  float     steer_cmd;        // servo command written in range [-100..+100]
  uint16_t  spinup_ticks_left;// remaining ticks to hold TURN_MIN_TICKS_PER_DT at turn start
  uint16_t  spinup_total_ticks;// configured spin-up duration (ticks)
} move_state_t;

static move_state_t ms = {0};

static uint32_t compute_servo_settle_ms(float target_cmd)
{
  float delta = fabsf(target_cmd - ms.steer_cmd);
  float settle_ms = delta * STEER_SETTLE_MS_PER_UNIT;
  return (uint32_t)lroundf(settle_ms);
}
extern Servo global_steer; // provided by main.c

// =================================================================================
// P U B L I C   A P I
// =================================================================================

uint8_t move_is_active(void) {
  return ms.active;
}

void move_abort(void) {
  if (!ms.active) return;
  ms.active = 0;
  ms.mode = MOVE_IDLE;
  control_set_target_ticks_per_dt(0, 0);
  motor_stop();
}

void move_start_straight(float distance_cm) {
  // 1. Reset hardware and state
  motor_reset_encoders();
  control_sync_encoders();
  imu_zero_yaw(); // Ensure we start with a target heading of 0 degrees
  ms.active = 0;
  ms.mode = MOVE_STRAIGHT;
  ms.dir = (distance_cm >= 0.0f) ? 1 : -1;

  // Center steering for straight driving with proportional settle wait
  Servo_Center(&global_steer);
  uint32_t settle_ms = compute_servo_settle_ms(0.0f);
  if (settle_ms > 0U) {
    osDelay(settle_ms);
  }
  ms.steer_cmd = 0.0f;

  // 2. Calculate target distances in encoder ticks
  ms.total_ticks_abs = (int32_t)lroundf(fabsf(distance_cm) * TICKS_PER_CM);
  if (ms.total_ticks_abs < 10) { // Avoid trivial movements
    return;
  }

  // 3. Define the motion profile key points using fixed distances (in cm)
  float accel_cm = (g_accel_dist_cm < 0.0f) ? 0.0f : g_accel_dist_cm;
  float decel_cm = (g_decel_dist_cm < 0.0f) ? 0.0f : g_decel_dist_cm;
  int32_t accel_ticks = (int32_t)lroundf(accel_cm * TICKS_PER_CM);
  int32_t decel_ticks = (int32_t)lroundf(decel_cm * TICKS_PER_CM);
  if (accel_ticks < 0) accel_ticks = 0;
  if (decel_ticks < 0) decel_ticks = 0;

  // If total distance is too short, use a symmetric triangle (accel then decel)
  if (ms.total_ticks_abs <= (accel_ticks + decel_ticks)) {
    ms.accel_end_ticks   = ms.total_ticks_abs / 2;
    ms.decel_start_ticks = ms.accel_end_ticks;
  } else {
    ms.accel_end_ticks   = accel_ticks;
    ms.decel_start_ticks = ms.total_ticks_abs - decel_ticks;
  }

  // 4. Set initial speed target (zero, the tick function will handle the rest)
  control_set_target_ticks_per_dt(0, 0);
  ms.active = 1;
}

void move_turn(char dir_char, float angle_deg)
{
  // Prepare state
  motor_reset_encoders();   // keeps odometry deltas clean
  control_sync_encoders();
  imu_zero_yaw();           // measure delta from current heading
  ms.mode = MOVE_TURN;
  ms.active = 0;

  // Determine turn side and drive direction
  if (dir_char == 'L') {
    ms.turn_sign = +1; // left
    ms.drive_sign = +1; // forward
  } else if (dir_char == 'R') {
    ms.turn_sign = -1; // right
    ms.drive_sign = +1; // forward
  } else if (dir_char == 'l') {
    ms.turn_sign = +1; // left yaw target
    ms.drive_sign = -1; // backward motion
  } else { // default to 'r'
    ms.turn_sign = -1; // right yaw target
    ms.drive_sign = -1; // backward motion
  }

  float requested_angle = fabsf(angle_deg);
  if (requested_angle <= 0.01f) {
    requested_angle = TURN_YAW_TARGET_DEG;
  }

  // Yaw target sign flips when reversing: steering left in reverse yields rightward yaw
  float yaw_target = requested_angle * (float)ms.turn_sign;
  if (ms.drive_sign < 0) {
    yaw_target = -yaw_target;
  }
  ms.yaw_target_deg = yaw_target;

  // Steering command: Servo negative = left, positive = right; consistent even when reversing
  float steer_angle = (ms.turn_sign > 0) ? -STEER_ANGLE_MAG : +STEER_ANGLE_MAG;

  // Ensure rear wheels are stationary before moving the steering linkage
  control_set_target_ticks_per_dt(0, 0);
  motor_stop();
  Servo_WriteAngle(&global_steer, steer_angle);
  uint32_t settle_ms = compute_servo_settle_ms(steer_angle);
  if (settle_ms > 0U) {
    osDelay(settle_ms);
  }
  ms.steer_cmd = steer_angle;
  ms.yaw_accum_deg = 0.0f;
  ms.prev_yaw_deg = 0.0f;

  // Use the configured spin-up duration before ramping to steady turn speed
  ms.spinup_total_ticks = g_turn_spinup_ticks;
  ms.spinup_ticks_left = ms.spinup_total_ticks;
  control_set_target_ticks_per_dt(0, 0);

  // Mark motion active only after steering is settled
  ms.active = 1;

  // Initial wheel targets: both wheels same sign (Ackermann forward/backward arc)
  control_set_target_ticks_per_dt(TURN_MIN_TICKS_PER_DT * ms.drive_sign,
                                  TURN_MIN_TICKS_PER_DT * ms.drive_sign);
}



// Call this function at exactly 100 Hz (every 10ms)
void move_tick_100Hz(void) {
  // Always integrate yaw so UI sees live orientation even when idle
  imu_update_yaw_100Hz();
  if (!ms.active) return; // no active motion plan; nothing else to do

  float current_yaw = imu_get_yaw();

  // Branch by mode
  if (ms.mode == MOVE_TURN) {
    // Unwrap yaw delta so accumulated heading can exceed ±180°
    float yaw_delta = current_yaw - ms.prev_yaw_deg;
    if (yaw_delta > 180.0f) {
      yaw_delta -= 360.0f;
    } else if (yaw_delta < -180.0f) {
      yaw_delta += 360.0f;
    }
    ms.yaw_accum_deg += yaw_delta;
    ms.prev_yaw_deg = current_yaw;

  // Compute remaining angle to target
  float err = ms.yaw_target_deg - ms.yaw_accum_deg; // degrees remaining
  float target_abs = fabsf(ms.yaw_target_deg);
  float accum_abs = fabsf(ms.yaw_accum_deg);

  // If within tolerance or we have crossed/exceeded the desired yaw, stop
  if ((fabsf(err) <= TURN_YAW_TOLERANCE_DEG) ||
    (target_abs > 0.0f && accum_abs >= target_abs)) {
      ms.active = 0;
      ms.mode = MOVE_IDLE;
      control_set_target_ticks_per_dt(0, 0);
      motor_brake_ms(50);
      motor_stop();
      return;
    }

    // Scale per-wheel ticks as we approach target for smooth stop
    int32_t base;
    float aerr = fabsf(err);
    
    if (ms.spinup_ticks_left > 0U) {
      uint16_t total = ms.spinup_total_ticks;
      if (total == 0U) {
        base = TURN_BASE_TICKS_PER_DT;
        ms.spinup_ticks_left = 0U;
      } else {
        uint16_t completed = (uint16_t)(total - ms.spinup_ticks_left);
        float progress = ((float)completed + 1.0f) / (float)total;
        if (progress > 1.0f) progress = 1.0f;
        float ramp = (float)TURN_MIN_TICKS_PER_DT + ((float)TURN_BASE_TICKS_PER_DT - (float)TURN_MIN_TICKS_PER_DT) * progress;
        if (ramp < (float)TURN_MIN_TICKS_PER_DT) ramp = (float)TURN_MIN_TICKS_PER_DT;
        if (ramp > (float)TURN_BASE_TICKS_PER_DT) ramp = (float)TURN_BASE_TICKS_PER_DT;
        base = (int32_t)lroundf(ramp);
        ms.spinup_ticks_left--;
      }
    } else {
      base = TURN_BASE_TICKS_PER_DT;
      if (aerr < TURN_YAW_SLOW_BAND_DEG) {
        float scale = aerr / TURN_YAW_SLOW_BAND_DEG; // 1 -> 0 as we approach
        int32_t slow = (int32_t)lroundf((float)TURN_BASE_TICKS_PER_DT * scale);
        if (slow < TURN_MIN_TICKS_PER_DT) slow = TURN_MIN_TICKS_PER_DT;
        base = slow;
      }
    }

    // Ackermann differential on rear wheels:
    // Use bicycle model: R = L / tan(delta). Scale rear left/right by (R ± T/2)/R.
    float steer_mag = fabsf(ms.steer_cmd); // 0..100
    int32_t left, right;
    if (steer_mag < 1e-3f) {
      // Essentially straight
      left  = base * ms.drive_sign;
      right = base * ms.drive_sign;
    } else {
      float delta_deg = (steer_mag / 100.0f) * STEER_MAX_DEG;
      float delta_rad = delta_deg * (3.14159265358979323846f / 180.0f);
      float tan_delta = tanf(delta_rad);
      if (fabsf(tan_delta) < 1e-4f) {
        left  = base * ms.drive_sign;
        right = base * ms.drive_sign;
      } else {
        float R = WHEELBASE_CM / tan_delta;   // turn radius of rear axle centerline
        float rfac = TRACK_WIDTH_CM / (2.0f * fabsf(R)); // >= 0
        rfac *= 1.5f; // stronger bias for tighter turns
        // Clamp to avoid negative inner factor at extreme angles
        if (rfac > 0.9f) rfac = 0.9f;
        float inner = (1.0f - rfac);
        float outer = (1.0f + rfac);
        // For left turn, left is inner; for right turn, right is inner
        float lscale = (ms.turn_sign > 0) ? inner : outer;
        float rscale = (ms.turn_sign > 0) ? outer : inner;
        int32_t lcmd = (int32_t)lroundf((float)base * lscale);
        int32_t rcmd = (int32_t)lroundf((float)base * rscale);
        // Enforce minimum per-wheel speed while turning
        if (lcmd < TURN_MIN_TICKS_PER_DT) lcmd = TURN_MIN_TICKS_PER_DT;
        if (rcmd < TURN_MIN_TICKS_PER_DT) rcmd = TURN_MIN_TICKS_PER_DT;
        left  = lcmd * ms.drive_sign;
        right = rcmd * ms.drive_sign;
      }
    }
    control_set_target_ticks_per_dt(left, right);
    return;
  }

  // --- Straight mode ---
  ms.prev_yaw_deg = current_yaw;
  // Use the average of both encoders for a robust distance measurement
  int32_t left_ticks = motor_get_left_encoder_counts();
  int32_t right_ticks = motor_get_right_encoder_counts();
  int32_t distance_travelled_ticks = (abs(left_ticks) + abs(right_ticks)) / 2;

  // 2. Check for completion
  if (distance_travelled_ticks >= ms.total_ticks_abs) {
    ms.active = 0;
    control_set_target_ticks_per_dt(0, 0);
    motor_brake_ms(80); // Apply a short active brake for a crisp stop
    motor_stop();       // Ensure H-bridge is off after braking
    return;
  }

  // 3. Determine target base speed based on position in the trapezoidal profile
  float target_base_speed_ticks;

  if (distance_travelled_ticks < ms.accel_end_ticks) {
    // --- ACCELERATION PHASE ---
    float progress = (float)distance_travelled_ticks / (float)ms.accel_end_ticks;
    target_base_speed_ticks = V_MIN_TICKS_PER_DT + (V_MAX_TICKS_PER_DT - V_MIN_TICKS_PER_DT) * progress;
  }
  else if (distance_travelled_ticks > ms.decel_start_ticks) {
    // --- DECELERATION PHASE ---
    float progress = (float)(ms.total_ticks_abs - distance_travelled_ticks) / (float)(ms.total_ticks_abs - ms.decel_start_ticks);
    target_base_speed_ticks = V_MIN_TICKS_PER_DT + (V_MAX_TICKS_PER_DT - V_MIN_TICKS_PER_DT) * progress;
  }
  else {
    // --- CRUISE PHASE ---
    target_base_speed_ticks = V_MAX_TICKS_PER_DT;
  }

  // Ensure speed never drops below minimum while moving
  if (target_base_speed_ticks < V_MIN_TICKS_PER_DT) {
    target_base_speed_ticks = V_MIN_TICKS_PER_DT;
  }

  // 4) Respect remaining distance so we don't command more ticks than available
  int32_t remaining_ticks = ms.total_ticks_abs - distance_travelled_ticks;
  if (remaining_ticks < 0) remaining_ticks = 0;
  int32_t base_ticks = (int32_t)lroundf(target_base_speed_ticks);
  if (base_ticks > remaining_ticks) base_ticks = remaining_ticks;
  if (base_ticks < 0) base_ticks = 0;
  if (base_ticks == 0 && remaining_ticks > 0) base_ticks = 1; // ensure progress

  // 5) Calculate yaw correction. Keep forward behavior and make reverse match it
  // by flipping the bias sign when driving backward.
  int32_t yaw_bias = (int32_t)lroundf(-current_yaw * YAW_KP_TICKS_PER_DEG);
  if (ms.dir < 0) yaw_bias = -yaw_bias;
  if (yaw_bias > base_ticks/2) yaw_bias = base_ticks/2;
  if (yaw_bias < -(base_ticks/2)) yaw_bias = -(base_ticks/2);
  
  // 6) Combine base and yaw correction
  int32_t left_target_ticks  = base_ticks - yaw_bias;
  int32_t right_target_ticks = base_ticks + yaw_bias;

  // 6. Set the targets for the low-level PID speed controllers
  control_set_target_ticks_per_dt(left_target_ticks * ms.dir, right_target_ticks * ms.dir);
}

// Optional: runtime configuration of accel/decel distances (in centimeters)
void move_set_profile_distances_cm(float accel_cm, float decel_cm)
{
  if (accel_cm < 0.0f) accel_cm = 0.0f;
  if (decel_cm < 0.0f) decel_cm = 0.0f;
  g_accel_dist_cm = accel_cm;
  g_decel_dist_cm = decel_cm;
}

void move_get_profile_distances_cm(float *accel_cm, float *decel_cm)
{
  if (accel_cm) *accel_cm = g_accel_dist_cm;
  if (decel_cm) *decel_cm = g_decel_dist_cm;
}

void move_set_turn_spinup_ticks(uint16_t ticks_100Hz)
{
  g_turn_spinup_ticks = ticks_100Hz;
}

uint16_t move_get_turn_spinup_ticks(void)
{
  return g_turn_spinup_ticks;
}
