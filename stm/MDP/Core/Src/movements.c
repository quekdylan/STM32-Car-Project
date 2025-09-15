// movements.c
#include "movements.h"
#include "motor.h"
#include "imu.h"
#include "control.h" // Assumes this provides control_set_target_ticks_per_dt()
#include "servo.h"
#include <math.h>
#include <stdlib.h> // For abs()

// =================================================================================
// T U N A B L E   P A R A M E T E R S
// =================================================================================

// --- Motion Profile Configuration (Fixed distances in centimeters) ---
// You can change these at runtime via move_set_profile_distances_cm().
static float g_accel_dist_cm = 10.0f; // accelerate over first 10 cm
static float g_decel_dist_cm = 10.0f; // decelerate over last 10 cm

// --- Speed Configuration ---
// Speeds are defined in "encoder ticks per control period (10ms)".
// You must tune these for your specific robot.
#define V_MAX_TICKS_PER_DT (35.0f) // Max speed during cruise phase (e.g., 35 ticks / 10ms)
#define V_MIN_TICKS_PER_DT (5.0f)  // Minimum speed to overcome static friction and ensure smooth start/stop

// --- Yaw Correction ---
// Proportional gain for heading correction.
// A small value gently corrects the heading. Too large a value will cause oscillation.
// Units: (ticks/10ms) per degree of error.
#define YAW_KP_TICKS_PER_DEG (0.25f)

// --- Turn Configuration ---
// Speed for in-place turns (encoder ticks / 10ms per wheel; opposite signs).
#define TURN_BASE_TICKS_PER_DT   (10)     // per wheel (both wheels same magnitude)
#define TURN_YAW_TARGET_DEG      (90.0f)  // desired turn angle
#define TURN_YAW_TOLERANCE_DEG   (2.0f)   // acceptable tolerance to stop
#define TURN_YAW_SLOW_BAND_DEG   (15.0f)  // start slowing when within this band
#define TURN_MIN_TICKS_PER_DT    (3)      // minimum per-wheel speed while turning

// Steering angles for Ackermann (Servo_WriteAngle: -100=full left, +100=full right)
#define STEER_ANGLE_MAG          (60.0f)  // magnitude for turning

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
} move_state_t;

static move_state_t ms = {0};
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
  imu_zero_yaw(); // Ensure we start with a target heading of 0 degrees
  ms.active = 1;
  ms.mode = MOVE_STRAIGHT;
  ms.dir = (distance_cm >= 0.0f) ? 1 : -1;

  // Center steering for straight driving
  Servo_Center(&global_steer);

  // 2. Calculate target distances in encoder ticks
  ms.total_ticks_abs = (int32_t)lroundf(fabsf(distance_cm) * TICKS_PER_CM);
  if (ms.total_ticks_abs < 10) { // Avoid trivial movements
    ms.active = 0;
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
}

void move_start_turn(char dir_char)
{
  // Prepare state
  motor_reset_encoders();   // not strictly needed for turn, but keeps odom clean
  imu_zero_yaw();           // measure delta from current heading
  ms.active = 1;
  ms.mode = MOVE_TURN;
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
  } else { // 'r'
    ms.turn_sign = -1; // right yaw target
    ms.drive_sign = -1; // backward motion
  }
  ms.yaw_target_deg = TURN_YAW_TARGET_DEG * (float)ms.turn_sign;

  // Set steering angle for Ackermann turn
  float steer_angle = (ms.turn_sign > 0) ? -STEER_ANGLE_MAG : +STEER_ANGLE_MAG; // negative=left
  // When reversing, to achieve the same yaw direction, steering must be opposite
  if (ms.drive_sign < 0) steer_angle = -steer_angle;
  Servo_WriteAngle(&global_steer, steer_angle);

  // Initial wheel targets: both wheels same sign (Ackermann forward/backward arc)
  control_set_target_ticks_per_dt(TURN_BASE_TICKS_PER_DT * ms.drive_sign,
                                  TURN_BASE_TICKS_PER_DT * ms.drive_sign);
}


// Call this function at exactly 100 Hz (every 10ms)
void move_tick_100Hz(void) {
  if (!ms.active) return;

  // 1. Update IMU yaw at 100 Hz for both modes
  imu_update_yaw_100Hz();
  float current_yaw = imu_get_yaw();

  // Branch by mode
  if (ms.mode == MOVE_TURN) {
    // Compute remaining angle to target
    float err = ms.yaw_target_deg - current_yaw; // degrees

    // If within tolerance, stop
    if (fabsf(err) <= TURN_YAW_TOLERANCE_DEG) {
      ms.active = 0;
      ms.mode = MOVE_IDLE;
      control_set_target_ticks_per_dt(0, 0);
      // Center steering on completion
      Servo_Center(&global_steer);
      motor_brake_ms(50);
      motor_stop();
      return;
    }

    // Scale per-wheel ticks as we approach target for smooth stop
    int32_t base = TURN_BASE_TICKS_PER_DT;
    float aerr = fabsf(err);
    if (aerr < TURN_YAW_SLOW_BAND_DEG) {
      float scale = aerr / TURN_YAW_SLOW_BAND_DEG; // 1 -> 0 as we approach
      int32_t slow = (int32_t)lroundf((float)TURN_BASE_TICKS_PER_DT * scale);
      if (slow < TURN_MIN_TICKS_PER_DT) slow = TURN_MIN_TICKS_PER_DT;
      base = slow;
    }

    int32_t left  = base * ms.drive_sign;
    int32_t right = base * ms.drive_sign;
    control_set_target_ticks_per_dt(left, right);
    return;
  }

  // --- Straight mode ---
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

  // 5) Calculate yaw correction, clamp so both sides remain non-negative
  int32_t yaw_bias = (int32_t)lroundf(current_yaw * YAW_KP_TICKS_PER_DEG);
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
