// movements.c
#include "movements.h"
#include "motor.h"
#include "imu.h"
#include "control.h" // Assumes this provides control_set_target_ticks_per_dt()
#include "servo.h"
#include "main.h"
#include "sensor.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdlib.h> // For abs()
#include <stdint.h>
#include <stdio.h>

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart3;

// =================================================================================
// T U N A B L E   P A R A M E T E R S
// =================================================================================

// --- Motion Profile Configuration (Fixed distances in centimeters) ---
// You can change these at runtime via move_set_profile_distances_cm().
static float g_accel_dist_cm = 10.0f; // accelerate over first 5 cm
static float g_decel_dist_cm = 12.0f; // decelerate over last 12 cm

// --- Speed Configuration ---
// Speeds are defined in "encoder ticks per control period (10ms)".
// You must tune these for your specific robot.
#define V_MAX_TICKS_PER_DT (62.0f) // Max speed during cruise phase (e.g., 30 ticks / 10ms) outdoor-50
#define V_MIN_TICKS_PER_DT (4.0f)  // Minimum speed to overcome static friction and ensure smooth start/stop

// --- Yaw Correction ---
// Proportional gain for heading correction.
// A small value gently corrects the heading. Too large a value will cause oscillation.
// Units: (ticks/10ms) per degree of error.
#define YAW_KP_TICKS_PER_DEG (3.7f)  // Increased from 3.0f for better straight-line correction

#define YAW_FILTER_WINDOW_SIZE  (10U)

// --- Turn Configuration ---
// Speed for ackermann turns (encoder ticks / 10ms per wheel after scaling).
#define TURN_BASE_TICKS_PER_DT   (30)     // base speed before inner/outer scaling   outdoor-27
#define TURN_YAW_TARGET_DEG      (90.0f)  // default turn angle when none provided
#define TURN_YAW_TOLERANCE_DEG   (0.1f)   // acceptable tolerance to stop
#define TURN_YAW_SLOW_BAND_DEG   (8.0f)  // start slowing when within this band
#define TURN_MIN_TICKS_PER_DT    (4)      // minimum per-wheel speed while turning
#define STEER_SETTLE_MS_PER_UNIT (2.0f)   // estimated ms per servo command unit of travel

// duration (in 100 Hz ticks) to hold/transition minimum speed at turn start to allow time for front wheels to turn
static uint16_t g_turn_spinup_ticks = 100;

// --- Arc Maneuver Configuration ---
#define ARC_BASE_TICKS_PER_DT         (27)     // nominal encoder ticks/10ms during arc   outdoor-27
#define ARC_SERVO_MAG_DEG             (91.0f)  // outer segment steering angle
#define ARC_MIDDLE_SERVO_MAG_DEG      (91.0f)  // middle segment steering angle
#define ARC_OUTER_RATIO               (1.79f)   // speed multiplier for outer wheel
#define ARC_YAW_TARGET_DEG            (47.0f)  // yaw goal per arc leg
#define ARC_YAW_TOLERANCE_DEG         (0.1f)   // yaw tolerance before switching segments
#define ARC_SLOW_BAND_DEG             (10.0f)   // yaw error band for slowing down
#define ARC_STRAIGHTEN_DELAY_TICKS    (15U)    // hold straight after final segment
#define ARC_SEGMENT_MIN_HOLD_TICKS    (35U)    // keep minimum speed at segment start

// --- Drive until obstacle configuration ---
#define DRIVE_OBS_TICKS_PER_DT             (37)    // nominal speed while approaching obstacle (slower to reduce overshoot)
#define DRIVE_OBS_DEFAULT_THRESHOLD_CM     (30.0f) // fallback stop distance
#define DRIVE_OBS_DEFAULT_MAX_DIST_CM      (160.0f) // max travel before abort
#define DRIVE_OBS_SLOW_DISTANCE_CM         (20.0f) // begin slowing within this range (earlier decel)
#define DRIVE_OBS_STOP_TOLERANCE_CM        (0.4f)  // acceptable distance error (tighter)
#define DRIVE_OBS_STOP_HYST_CM             (1.0f)  // extra deadband so we don't re-arm instantly
#define DRIVE_OBS_STOP_CONFIRM_TICKS       (3U)    // how many control samples to confirm no motion (stricter)
#define DRIVE_OBS_STOP_EPS_CM              (0.2f)  // "basically not moving" threshold (stricter)
#define DRIVE_OBS_BRAKE_MS                 (150U)   // brief active-brake pulse (longer)
#define DRIVE_OBS_POST_HOLD_MS             (150U)   // extra hold after confirming stop to prevent creep
#define DRIVE_OBS_SAMPLE_INTERVAL_TICKS    (4U)    // ultrasonic sampling interval
#define DRIVE_OBS_TRIGGER_DELAY_MS         (40U)   // wait after trigger before read

// --- IR Sensor Configuration ---
#define IR_DISTANCE_THRESHOLD_CM           (40.0f) // stop once IR-reported distance exceeds this
#define IR_MAX_TRAVEL_CM                   (200.0f) // safety cap in centimeters
#define IR_MAX_DISTANCE_CM                 (80.0f)  // clamp converted readings
#define IR_SAMPLE_COUNT                    (10U)    // average these many samples per read
#define IR_POLL_DELAY_MS                   (10U)   // wait between IR polls for quicker response
#define IR_MAX_TIME_MS                     (8000U) // safety cap by time when not tracking distance
#define IR_BELOW_FIXED_THRESHOLD_CM        (30.0f) // fixed threshold for move_until_ir_below
#define IR_ADC_POLL_TIMEOUT_MS             (20U)   // ADC conversion timeout
// Use a longer ADC sample time for stable IR readings (higher source impedance)
#define IR_ADC_SAMPLE_TIME                 ADC_SAMPLETIME_144CYCLES
#define IR_ADC_CHANNEL_LEFT                ADC_CHANNEL_2
#define IR_ADC_CHANNEL_RIGHT               ADC_CHANNEL_3
#define IR_DRIVE_TICKS_PER_DT              (27)     // nominal speed while hunting IR edge

// --- Post-turn steering stabilization ---
#define SERVO_POST_TURN_RIGHT_US         (1600U) // apply slight right bias after finishing a left turn
#define SERVO_POST_TURN_LEFT_US          (1450U) // apply slight left bias after finishing a right turn
#define SERVO_POST_TURN_DELAY_TICKS      (5U)    // wait ~50 ms at 100 Hz before re-centering

// Turn profiles allow quick switching between steering envelopes/differentials.
#define TURN_DEFAULT_PROFILE               MOVE_TURN_PROFILE_PRIMARY

// 30cm turn radius
#define TURN_PROFILE_PRIMARY_STEER_MAG      (98.9f)
#define TURN_PROFILE_PRIMARY_OUTER_RATIO    (1.7f)

//20cm turn radius
#define TURN_PROFILE_SECONDARY_STEER_MAG    (100.0f)
#define TURN_PROFILE_SECONDARY_OUTER_RATIO  (5.0f)

// Steering angles for Ackermann (Servo_WriteAngle: -100=full left, +100=full right)
// are derived from the selected turn profile.

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
  MOVE_TURN,
  MOVE_ARC
} move_mode_e;

extern Servo global_steer; // provided by main.c

typedef struct {
  move_arc_side_e side;
  uint8_t segment_index;
  float yaw_targets[3];
  float servo_cmds[3];
  uint16_t straighten_delay_ticks;
  uint16_t segment_hold_ticks;
} move_arc_state_t;

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
  uint8_t   servo_recentering_phase; // 0 = none, 1 = waiting to center after overshoot
  uint16_t  servo_recentering_counter; // countdown ticks for post-turn centering
  move_arc_state_t arc;       // state for arc maneuver
} move_state_t;

static move_state_t ms = {0};

typedef struct {
  float    samples[YAW_FILTER_WINDOW_SIZE];
  float    sum;
  uint8_t  count;
  uint8_t  index;
} yaw_filter_t;

static yaw_filter_t g_yaw_filter = {0};

static move_turn_profile_e g_turn_profile = TURN_DEFAULT_PROFILE;

static void yaw_filter_reset(float seed)
{
  g_yaw_filter.sum = 0.0f;
  g_yaw_filter.count = 0U;
  g_yaw_filter.index = 0U;
  for (uint8_t i = 0U; i < YAW_FILTER_WINDOW_SIZE; ++i) {
    g_yaw_filter.samples[i] = seed;
  }
}

static float yaw_filter_apply(float sample)
{
  if (YAW_FILTER_WINDOW_SIZE == 0U) {
    return sample;
  }

  if (g_yaw_filter.count < YAW_FILTER_WINDOW_SIZE) {
    g_yaw_filter.samples[g_yaw_filter.index] = sample;
    g_yaw_filter.sum += sample;
    g_yaw_filter.index = (uint8_t)((g_yaw_filter.index + 1U) % YAW_FILTER_WINDOW_SIZE);
    g_yaw_filter.count++;
  } else {
    float old_sample = g_yaw_filter.samples[g_yaw_filter.index];
    g_yaw_filter.samples[g_yaw_filter.index] = sample;
    g_yaw_filter.sum += sample - old_sample;
    g_yaw_filter.index = (uint8_t)((g_yaw_filter.index + 1U) % YAW_FILTER_WINDOW_SIZE);
  }

  if (g_yaw_filter.count == 0U) {
    return sample;
  }

  return g_yaw_filter.sum / (float)g_yaw_filter.count;
}

static float turn_profile_get_steer_mag(void)
{
  return (g_turn_profile == MOVE_TURN_PROFILE_PRIMARY)
             ? TURN_PROFILE_PRIMARY_STEER_MAG
             : TURN_PROFILE_SECONDARY_STEER_MAG;
}

static float turn_profile_get_outer_ratio(void)
{
  return (g_turn_profile == MOVE_TURN_PROFILE_PRIMARY)
             ? TURN_PROFILE_PRIMARY_OUTER_RATIO
             : TURN_PROFILE_SECONDARY_OUTER_RATIO;
}

static float median_of_three(float a, float b, float c)
{
  if (a > b) {
    float tmp = a; a = b; b = tmp;
  }
  if (b > c) {
    float tmp = b; b = c; c = tmp;
  }
  if (a > b) {
    float tmp = a; a = b; b = tmp;
  }
  return b;
}

static float average_window(const float *values, uint8_t count)
{
  if (values == NULL || count == 0U) {
    return 0.0f;
  }
  float sum = 0.0f;
  for (uint8_t i = 0U; i < count; ++i) {
    sum += values[i];
  }
  return sum / (float)count;
}

static uint32_t compute_servo_settle_ms(float target_cmd)
{
  float delta = fabsf(target_cmd - ms.steer_cmd);
  float settle_ms = delta * STEER_SETTLE_MS_PER_UNIT;
  return (uint32_t)lroundf(settle_ms);
}

static uint16_t ir_read_average_raw(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = IR_ADC_SAMPLE_TIME;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    return 0U;
  }

  uint32_t accumulator = 0U;
  uint8_t acquired = 0U;

  for (uint8_t i = 0U; i < IR_SAMPLE_COUNT; ++i) {
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
      break;
    }
    if (HAL_ADC_PollForConversion(&hadc1, IR_ADC_POLL_TIMEOUT_MS) == HAL_OK) {
      accumulator += HAL_ADC_GetValue(&hadc1);
      acquired++;
    }
    HAL_ADC_Stop(&hadc1);
  }

  if (acquired == 0U) {
    return 0U;
  }

  return (uint16_t)(accumulator / acquired);
}

static float ir_raw_to_distance_cm(uint16_t raw)
{
  if (raw == 0U) {
    return IR_MAX_DISTANCE_CM;
  }

  // Convert 12-bit ADC reading to an approximate 10-bit value for common Sharp GP2Y0A21YK curve fits.
  float raw10 = ((float)raw) * (1023.0f / 4095.0f);
  if (raw10 < 21.0f) {
    raw10 = 21.0f;
  }

  float distance = 4800.0f / (raw10 - 20.0f);
  if (distance > IR_MAX_DISTANCE_CM) {
    distance = IR_MAX_DISTANCE_CM;
  }
  if (distance < 0.0f) {
    distance = 0.0f;
  }
  return distance;
}

static float ir_sample_distance_cm(uint32_t channel)
{
  uint16_t raw = ir_read_average_raw(channel);
  if (raw == 0U) {
    return IR_MAX_DISTANCE_CM;
  }
  return ir_raw_to_distance_cm(raw);
}

static void schedule_post_turn_servo_recentering(void)
{
  uint16_t pulse_us = (ms.turn_sign > 0) ? SERVO_POST_TURN_RIGHT_US : SERVO_POST_TURN_LEFT_US;
  Servo_WriteUS(&global_steer, pulse_us);
  ms.servo_recentering_phase = 1U;
  ms.servo_recentering_counter = SERVO_POST_TURN_DELAY_TICKS;
}

// =================================================================================
// P U B L I C   A P I
// =================================================================================

uint8_t move_is_active(void) {
  return ms.active;
}

void move_set_turn_profile(move_turn_profile_e profile)
{
  if (profile != MOVE_TURN_PROFILE_PRIMARY && profile != MOVE_TURN_PROFILE_SECONDARY) {
    return;
  }
  g_turn_profile = profile;
}

move_turn_profile_e move_get_turn_profile(void)
{
  return g_turn_profile;
}

void move_abort(void) {
  if (!ms.active) return;
  ms.active = 0;
  ms.mode = MOVE_IDLE;
  control_set_target_ticks_per_dt(0, 0);
  motor_stop();
  ms.servo_recentering_phase = 0U;
  ms.servo_recentering_counter = 0U;
  ms.steer_cmd = 0.0f;
  Servo_Center(&global_steer);
  ms.arc.segment_hold_ticks = 0U;
}

void move_start_straight(float distance_cm) {
  // 1. Reset hardware and state
  motor_reset_encoders();
  control_sync_encoders();
  imu_zero_yaw(); // Ensure we start with a target heading of 0 degrees
  yaw_filter_reset(0.0f);
  ms.active = 0;
  ms.mode = MOVE_STRAIGHT;
  ms.dir = (distance_cm >= 0.0f) ? 1 : -1;
  ms.servo_recentering_phase = 0U;
  ms.servo_recentering_counter = 0U;

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
  yaw_filter_reset(0.0f);
  ms.mode = MOVE_TURN;
  ms.active = 0;
  ms.servo_recentering_phase = 0U;
  ms.servo_recentering_counter = 0U;

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
  float profile_steer_mag = turn_profile_get_steer_mag();


  float steer_angle = (ms.turn_sign > 0) ? -profile_steer_mag : +profile_steer_mag;

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

void move_start_arc(move_arc_side_e side)
{
  if (side != MOVE_ARC_SIDE_LEFT && side != MOVE_ARC_SIDE_RIGHT) {
    side = MOVE_ARC_SIDE_LEFT;
  }

  motor_reset_encoders();
  control_sync_encoders();
  imu_zero_yaw();
  yaw_filter_reset(0.0f);

  ms.active = 0U;
  ms.mode = MOVE_ARC;
  ms.dir = 1;
  ms.servo_recentering_phase = 0U;
  ms.servo_recentering_counter = 0U;

  Servo_Center(&global_steer);
  uint32_t settle_ms = compute_servo_settle_ms(0.0f);
  if (settle_ms > 0U) {
    osDelay(settle_ms);
  }
  ms.steer_cmd = 0.0f;

  ms.arc.side = side;
  ms.arc.segment_index = 0U;
  ms.arc.straighten_delay_ticks = 0U;
  ms.arc.segment_hold_ticks = ARC_SEGMENT_MIN_HOLD_TICKS;

  float first_target = (side == MOVE_ARC_SIDE_LEFT) ? ARC_YAW_TARGET_DEG : -ARC_YAW_TARGET_DEG;
  float first_servo = (side == MOVE_ARC_SIDE_LEFT) ? -ARC_SERVO_MAG_DEG : ARC_SERVO_MAG_DEG;

  ms.arc.yaw_targets[0] = first_target;
  ms.arc.yaw_targets[1] = -first_target;
  ms.arc.yaw_targets[2] = 0.0f;

  ms.arc.servo_cmds[0] = first_servo;
  ms.arc.servo_cmds[1] = (side == MOVE_ARC_SIDE_LEFT)
                             ? ARC_MIDDLE_SERVO_MAG_DEG
                             : -ARC_MIDDLE_SERVO_MAG_DEG;
  ms.arc.servo_cmds[2] = first_servo;

  ms.prev_yaw_deg = 0.0f;
  ms.yaw_accum_deg = 0.0f;

  ms.steer_cmd = ms.arc.servo_cmds[0];
  Servo_WriteAngle(&global_steer, ms.steer_cmd);

  control_set_target_ticks_per_dt(ARC_BASE_TICKS_PER_DT, ARC_BASE_TICKS_PER_DT);
  ms.active = 1U;
}

float move_drive_until_obstacle(float stop_threshold_cm, void (*service_hook)(void))
{
  float target_threshold_cm = (stop_threshold_cm > 0.0f) ? stop_threshold_cm : DRIVE_OBS_DEFAULT_THRESHOLD_CM;

  move_abort();
  Servo_Center(&global_steer);
  osDelay(20);
  if (service_hook) {
    service_hook();
  }

  motor_reset_encoders();
  control_sync_encoders();
  imu_zero_yaw();
  yaw_filter_reset(0.0f);

  int32_t prev_left = motor_get_left_encoder_counts();
  int32_t prev_right = motor_get_right_encoder_counts();
  // Accumulates signed travel: positive for forward, negative for reverse
  float travelled_cm = 0.0f;
  uint8_t sample_counter = 0U;
  int32_t dir_sign = 0;
  float distance_window[3] = {0.0f, 0.0f, 0.0f};
  uint8_t distance_count = 0U;
  uint8_t distance_index = 0U;
  uint8_t drive_stopped = 0U;
  uint8_t reverse_threshold_adjusted = 0U;

  control_set_target_ticks_per_dt(0, 0);

  while (1) {
    osDelay(10);
    if (service_hook) {
      service_hook();
    }

    int32_t left_now = motor_get_left_encoder_counts();
    int32_t right_now = motor_get_right_encoder_counts();
    int32_t dl = left_now - prev_left;
    int32_t dr = right_now - prev_right;
    prev_left = left_now;
    prev_right = right_now;

    float step_cm = ( (fabsf((float)dl) + fabsf((float)dr)) * 0.5f ) / TICKS_PER_CM;
    // Apply sign once direction is determined; before that we are stationary
    if (dir_sign != 0) {
      travelled_cm += ((float)dir_sign) * step_cm;
    }
    if (travelled_cm >= DRIVE_OBS_DEFAULT_MAX_DIST_CM) {
      if (!drive_stopped) {
        control_set_target_ticks_per_dt(0, 0);
        motor_stop();
        drive_stopped = 1U;
      }
      break;
    }

    if (++sample_counter < DRIVE_OBS_SAMPLE_INTERVAL_TICKS) {
      continue;
    }
    sample_counter = 0U;

    // Read from ultrasonic cache produced by ultrasonic_task
    float distance = g_ultra_dist_cm;
    uint32_t age = osKernelGetTickCount() - g_ultra_age_ms;
    if (distance <= 0.0f || age >= 200U) {
      // No fresh sample: skip this cycle without blocking
      continue;
    }
    distance_window[distance_index] = distance;
    if (distance_count < 3U) {
      distance_count++;
    }
    distance_index = (uint8_t)((distance_index + 1U) % 3U);

    float filtered_distance;
    if (distance_count < 3U) {
      filtered_distance = average_window(distance_window, distance_count);
    } else {
      filtered_distance = median_of_three(distance_window[0], distance_window[1], distance_window[2]);
    }

    float delta_cm = filtered_distance - target_threshold_cm;
    if (dir_sign == 0) {
      if (delta_cm >= -DRIVE_OBS_STOP_TOLERANCE_CM &&
          delta_cm <= (DRIVE_OBS_STOP_TOLERANCE_CM + DRIVE_OBS_STOP_HYST_CM)) {
        goto hard_stop_and_confirm;
      }
      dir_sign = (delta_cm > 0.0f) ? +1 : -1;
      // If we need to reverse to reach the threshold, bias the target closer by 2 cm
      if (dir_sign < 0 && !reverse_threshold_adjusted) {
        target_threshold_cm -= 2.0f;
        if (target_threshold_cm < 1.0f) {
          target_threshold_cm = 1.0f; // clamp to sane minimum
        }
        reverse_threshold_adjusted = 1U;
        // Recompute delta with adjusted threshold
        delta_cm = filtered_distance - target_threshold_cm;
      }
    }

    if ((dir_sign > 0 && delta_cm <= (DRIVE_OBS_STOP_TOLERANCE_CM + DRIVE_OBS_STOP_HYST_CM)) ||
        (dir_sign < 0 && delta_cm >= -DRIVE_OBS_STOP_TOLERANCE_CM)) {
      goto hard_stop_and_confirm;
    }

    float distance_to_threshold = fabsf(delta_cm);

    float commanded_ticks;
    if (distance_to_threshold <= DRIVE_OBS_SLOW_DISTANCE_CM) {
      float ratio = distance_to_threshold / DRIVE_OBS_SLOW_DISTANCE_CM;
      if (ratio < 0.0f) ratio = 0.0f;
      if (ratio > 1.0f) ratio = 1.0f;
      commanded_ticks = V_MIN_TICKS_PER_DT +
                        ((float)DRIVE_OBS_TICKS_PER_DT - V_MIN_TICKS_PER_DT) * ratio;
    } else {
      commanded_ticks = (float)DRIVE_OBS_TICKS_PER_DT;
    }

    if (commanded_ticks < (float)V_MIN_TICKS_PER_DT) {
      commanded_ticks = (float)V_MIN_TICKS_PER_DT;
    }
    if (commanded_ticks > (float)DRIVE_OBS_TICKS_PER_DT) {
      commanded_ticks = (float)DRIVE_OBS_TICKS_PER_DT;
    }

    int32_t base_ticks = (int32_t)lroundf(commanded_ticks);
    if (base_ticks < V_MIN_TICKS_PER_DT) {
      base_ticks = V_MIN_TICKS_PER_DT;
    }
    if (base_ticks > DRIVE_OBS_TICKS_PER_DT) {
      base_ticks = DRIVE_OBS_TICKS_PER_DT;
    }
    // Apply a small reduction to base ticks to soften approach
    base_ticks -= 3;
    if (base_ticks < V_MIN_TICKS_PER_DT) {
      base_ticks = V_MIN_TICKS_PER_DT;
    }
    base_ticks *= dir_sign;
    float current_yaw = imu_get_yaw();
    float filtered_yaw = yaw_filter_apply(current_yaw);
    int32_t yaw_bias = (int32_t)lroundf(-filtered_yaw * YAW_KP_TICKS_PER_DEG);
    if (dir_sign < 0) {
      yaw_bias = -yaw_bias;
    }
    int32_t max_bias = abs(base_ticks) / 2;
    if (yaw_bias > max_bias) yaw_bias = max_bias;
    if (yaw_bias < -max_bias) yaw_bias = -max_bias;

    int32_t left_ticks = base_ticks - yaw_bias;
    int32_t right_ticks = base_ticks + yaw_bias;

    if (!drive_stopped) {
      control_set_target_ticks_per_dt(left_ticks, right_ticks);
      /* Make left wheel 1 tick slower than right (sign-aware for direction). */
      control_set_target_ticks_per_dt(left_ticks - dir_sign, right_ticks);
    }
  }

  goto done_driving;

hard_stop_and_confirm:
  {
    if (!drive_stopped) {
      control_set_target_ticks_per_dt(0, 0);
      motor_brake();
      control_reset_integrators();
      osDelay(DRIVE_OBS_BRAKE_MS);

      uint8_t still_count = 0U;
      int32_t l_prev = motor_get_left_encoder_counts();
      int32_t r_prev = motor_get_right_encoder_counts();
      while (still_count < DRIVE_OBS_STOP_CONFIRM_TICKS) {
        osDelay(10);
        if (service_hook) {
          service_hook();
        }

        int32_t l_now = motor_get_left_encoder_counts();
        int32_t r_now = motor_get_right_encoder_counts();
        float avg_ticks_confirm = (fabsf((float)(l_now - l_prev)) + fabsf((float)(r_now - r_prev))) * 0.5f;
        float moved_cm = avg_ticks_confirm / TICKS_PER_CM;

        l_prev = l_now;
        r_prev = r_now;

        control_set_target_ticks_per_dt(0, 0);

        if (moved_cm <= DRIVE_OBS_STOP_EPS_CM) {
          still_count++;
        } else {
          still_count = 0U;
        }
      }

      // Extra hold to avoid post-stop creep
      motor_brake_ms(DRIVE_OBS_POST_HOLD_MS);
      control_reset_integrators();
      motor_brake_end();
      motor_stop();
      drive_stopped = 1U;
    }
    goto done_driving;
  }

done_driving:
  ;

  if (!drive_stopped) {
    control_set_target_ticks_per_dt(0, 0);
    motor_brake();
    control_reset_integrators();
    osDelay(50);
    control_set_target_ticks_per_dt(0, 0);
    motor_brake_end();
    motor_stop();
  }

  Servo_Center(&global_steer);
  return travelled_cm;
}

float move_until_ir(char sensor_side)
{
  uint32_t adc_channel = IR_ADC_CHANNEL_LEFT;
  if (sensor_side == 'R' || sensor_side == 'r') {
    adc_channel = IR_ADC_CHANNEL_RIGHT;
  }

  move_abort();
  Servo_Center(&global_steer);
  osDelay(20);

  motor_reset_encoders();
  control_sync_encoders();
  imu_zero_yaw();
  yaw_filter_reset(0.0f);

  float initial_distance = ir_sample_distance_cm(adc_channel);
  if (initial_distance >= IR_DISTANCE_THRESHOLD_CM) {
    control_set_target_ticks_per_dt(0, 0);
    motor_stop();
    Servo_Center(&global_steer);
    return 0.0f;
  }

  int32_t prev_left = motor_get_left_encoder_counts();
  int32_t prev_right = motor_get_right_encoder_counts();
  float travelled_cm = 0.0f;

  control_set_target_ticks_per_dt(IR_DRIVE_TICKS_PER_DT,
                                  IR_DRIVE_TICKS_PER_DT);

  while (travelled_cm < IR_MAX_TRAVEL_CM) {
    osDelay(IR_POLL_DELAY_MS);

    int32_t left_now = motor_get_left_encoder_counts();
    int32_t right_now = motor_get_right_encoder_counts();
    int32_t dl = left_now - prev_left;
    int32_t dr = right_now - prev_right;
    prev_left = left_now;
    prev_right = right_now;

    float avg_ticks = (fabsf((float)dl) + fabsf((float)dr)) * 0.5f;
    travelled_cm += avg_ticks / TICKS_PER_CM;
    if (travelled_cm >= IR_MAX_TRAVEL_CM) {
      break;
    }

    float distance_cm = ir_sample_distance_cm(adc_channel);
    if (distance_cm >= IR_DISTANCE_THRESHOLD_CM) {
      control_set_target_ticks_per_dt(0, 0);
      motor_brake_ms(80);
      motor_stop();
      Servo_Center(&global_steer);
      return travelled_cm;
    }

    float remaining_cm = IR_DISTANCE_THRESHOLD_CM - distance_cm;
    if (remaining_cm < 0.0f) {
      remaining_cm = 0.0f;
    }

    float commanded_ticks = (remaining_cm <= DRIVE_OBS_SLOW_DISTANCE_CM)
                                ? V_MIN_TICKS_PER_DT +
                                      ((float)IR_DRIVE_TICKS_PER_DT - V_MIN_TICKS_PER_DT) *
                                          (remaining_cm / DRIVE_OBS_SLOW_DISTANCE_CM)
                                : (float)IR_DRIVE_TICKS_PER_DT;

    if (commanded_ticks < V_MIN_TICKS_PER_DT) {
      commanded_ticks = V_MIN_TICKS_PER_DT;
    }
    if (commanded_ticks > (float)IR_DRIVE_TICKS_PER_DT) {
      commanded_ticks = (float)IR_DRIVE_TICKS_PER_DT;
    }

    int32_t base_ticks = (int32_t)lroundf(commanded_ticks);
    if (base_ticks < V_MIN_TICKS_PER_DT) {
      base_ticks = V_MIN_TICKS_PER_DT;
    }
    if (base_ticks > IR_DRIVE_TICKS_PER_DT) {
      base_ticks = IR_DRIVE_TICKS_PER_DT;
    }

    float current_yaw = imu_get_yaw();
    float filtered_yaw = yaw_filter_apply(current_yaw);
    int32_t yaw_bias = (int32_t)lroundf(-filtered_yaw * YAW_KP_TICKS_PER_DEG);
    int32_t max_bias = base_ticks / 2;
    if (yaw_bias > max_bias) yaw_bias = max_bias;
    if (yaw_bias < -max_bias) yaw_bias = -max_bias;

    int32_t left_target = base_ticks - yaw_bias;
    int32_t right_target = base_ticks + yaw_bias;
    control_set_target_ticks_per_dt(left_target, right_target);
  }

  motor_brake_ms(200);
  control_set_target_ticks_per_dt(0, 0);
  motor_stop();
  Servo_Center(&global_steer);
  return travelled_cm;
}

// Variant: drive until IR distance is at or below a custom threshold (in cm).
// Returns the distance travelled (in cm).
float move_until_ir_below(char sensor_side)
{
  uint32_t adc_channel = IR_ADC_CHANNEL_LEFT;
  if (sensor_side == 'R' || sensor_side == 'r') {
    adc_channel = IR_ADC_CHANNEL_RIGHT;
  }

  // Use fixed threshold
  const float threshold_cm = IR_BELOW_FIXED_THRESHOLD_CM;

  move_abort();
  Servo_Center(&global_steer);
  osDelay(20);

  motor_reset_encoders();
  control_sync_encoders();
  imu_zero_yaw();
  yaw_filter_reset(0.0f);

  float initial_distance = ir_sample_distance_cm(adc_channel);
  // Log start state
  {
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "IRB start %c %.1f/%.1f\r\n",
                       (sensor_side == 'R' || sensor_side == 'r') ? 'R' : 'L',
                       initial_distance, threshold_cm);
    if (len > 0) {
      (void)HAL_UART_Transmit(&huart3, (uint8_t*)buf, (uint16_t)len, 50);
    }
  }
  if (initial_distance <= threshold_cm) {
    control_set_target_ticks_per_dt(0, 0);
    motor_stop();
    Servo_Center(&global_steer);
    return 0.0f;
  }

  control_set_target_ticks_per_dt(IR_DRIVE_TICKS_PER_DT,
                                  IR_DRIVE_TICKS_PER_DT);
  uint32_t elapsed_ms = 0U;
  uint32_t log_accum_ms = 0U;
  for (;;) {
    osDelay(IR_POLL_DELAY_MS);
    elapsed_ms += IR_POLL_DELAY_MS;
    log_accum_ms += IR_POLL_DELAY_MS;

    float distance_cm = ir_sample_distance_cm(adc_channel);
    if (distance_cm <= threshold_cm) {
      control_set_target_ticks_per_dt(0, 0);
      motor_brake_ms(200); // match the stronger stop used in move_until_ir
      motor_stop();
      Servo_Center(&global_steer);
      // Log stop
      {
        char buf[48];
        int len = snprintf(buf, sizeof(buf), "IRB stop %.1f\r\n", distance_cm);
        if (len > 0) {
          (void)HAL_UART_Transmit(&huart3, (uint8_t*)buf, (uint16_t)len, 50);
        }
      }
      return 0.0f;
    }

    // Slow down as we approach the threshold from above
    float remaining_cm = distance_cm - threshold_cm;
    if (remaining_cm < 0.0f) {
      remaining_cm = 0.0f;
    }

    float commanded_ticks = (remaining_cm <= DRIVE_OBS_SLOW_DISTANCE_CM)
                                ? V_MIN_TICKS_PER_DT +
                                      ((float)IR_DRIVE_TICKS_PER_DT - V_MIN_TICKS_PER_DT) *
                                          (remaining_cm / DRIVE_OBS_SLOW_DISTANCE_CM)
                                : (float)IR_DRIVE_TICKS_PER_DT;

    if (commanded_ticks < V_MIN_TICKS_PER_DT) {
      commanded_ticks = V_MIN_TICKS_PER_DT;
    }
    if (commanded_ticks > (float)IR_DRIVE_TICKS_PER_DT) {
      commanded_ticks = (float)IR_DRIVE_TICKS_PER_DT;
    }

    int32_t base_ticks = (int32_t)lroundf(commanded_ticks);
    if (base_ticks < V_MIN_TICKS_PER_DT) {
      base_ticks = V_MIN_TICKS_PER_DT;
    }
    if (base_ticks > IR_DRIVE_TICKS_PER_DT) {
      base_ticks = IR_DRIVE_TICKS_PER_DT;
    }

    float current_yaw = imu_get_yaw();
    float filtered_yaw = yaw_filter_apply(current_yaw);
    int32_t yaw_bias = (int32_t)lroundf(-filtered_yaw * YAW_KP_TICKS_PER_DEG);
    int32_t max_bias = base_ticks / 2;
    if (yaw_bias > max_bias) yaw_bias = max_bias;
    if (yaw_bias < -max_bias) yaw_bias = -max_bias;

    int32_t left_target = base_ticks - yaw_bias;
    int32_t right_target = base_ticks + yaw_bias;
    control_set_target_ticks_per_dt(left_target, right_target);

    // Periodic log every ~100 ms
    if (log_accum_ms >= 100U) {
      log_accum_ms = 0U;
      char buf[40];
      int len = snprintf(buf, sizeof(buf), "IRB d=%.1f\r\n", distance_cm);
      if (len > 0) {
        (void)HAL_UART_Transmit(&huart3, (uint8_t*)buf, (uint16_t)len, 20);
      }
    }

    if (elapsed_ms >= IR_MAX_TIME_MS) {
      break; // safety timeout
    }
  }

  // Safety stop if we exceeded max travel without hitting threshold
  control_set_target_ticks_per_dt(0, 0);
  motor_brake_ms(200);
  motor_stop();
  Servo_Center(&global_steer);
  {
    char buf[48];
    int len = snprintf(buf, sizeof(buf), "IRB timeout\r\n");
    if (len > 0) {
      (void)HAL_UART_Transmit(&huart3, (uint8_t*)buf, (uint16_t)len, 50);
    }
  }
  return 0.0f;
}

// Read the specified IR sensor and return the estimated distance in cm
float move_get_ir_distance_cm(char sensor_side)
{
  uint32_t adc_channel = (sensor_side == 'R' || sensor_side == 'r')
                           ? IR_ADC_CHANNEL_RIGHT
                           : IR_ADC_CHANNEL_LEFT;
  return ir_sample_distance_cm(adc_channel);
}


// Call this function at exactly 100 Hz (every 10ms)
void move_tick_100Hz(void) {
  // Always integrate yaw so UI sees live orientation even when idle
  imu_update_yaw_100Hz();
  if (!ms.active) {
    if (ms.servo_recentering_phase != 0U) {
      if (ms.servo_recentering_counter > 0U) {
        ms.servo_recentering_counter--;
      }
      if (ms.servo_recentering_counter == 0U) {
        Servo_Center(&global_steer);
        ms.steer_cmd = 0.0f;
        ms.servo_recentering_phase = 0U;
      }
    }
    return; // no active motion plan; nothing else to do
  }

  float current_yaw = imu_get_yaw();
  float filtered_yaw = yaw_filter_apply(current_yaw);

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

    Servo_WriteAngle(&global_steer, ms.steer_cmd);

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
      schedule_post_turn_servo_recentering();
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

    // Simple differential: outer wheel runs faster based on active turn profile ratio.
    int32_t slow = base;
    if (slow < TURN_MIN_TICKS_PER_DT) {
      slow = TURN_MIN_TICKS_PER_DT;
    }

    float outer_ratio = turn_profile_get_outer_ratio();
    if (outer_ratio < 1.0f) {
      outer_ratio = 1.0f;
    }

    int32_t fast = (int32_t)lroundf((float)slow * outer_ratio);
    if (fast < TURN_MIN_TICKS_PER_DT) {
      fast = TURN_MIN_TICKS_PER_DT;
    }

    int32_t left = (ms.turn_sign > 0) ? slow : fast;
    int32_t right = (ms.turn_sign > 0) ? fast : slow;

    left *= ms.drive_sign;
    right *= ms.drive_sign;

    control_set_target_ticks_per_dt(left, right);
    return;
  }
  else if (ms.mode == MOVE_ARC) {
    ms.prev_yaw_deg = current_yaw;
    uint8_t idx = ms.arc.segment_index;

    if (idx >= 3U) {
      Servo_Center(&global_steer);
      ms.steer_cmd = 0.0f;
      control_set_target_ticks_per_dt(0, 0);
      ms.arc.segment_hold_ticks = 0U;

      if (ms.arc.straighten_delay_ticks == 0U) {
        ms.arc.straighten_delay_ticks = ARC_STRAIGHTEN_DELAY_TICKS;
        return;
      }

      if (ms.arc.straighten_delay_ticks > 0U) {
        ms.arc.straighten_delay_ticks--;
        if (ms.arc.straighten_delay_ticks == 0U) {
          ms.active = 0U;
          ms.mode = MOVE_IDLE;
        }
      }
      return;
    }

    float target = ms.arc.yaw_targets[idx];
    float prev_target = (idx == 0U) ? 0.0f : ms.arc.yaw_targets[idx - 1U];
    float servo_cmd = ms.arc.servo_cmds[idx];

    ms.steer_cmd = servo_cmd;
    Servo_WriteAngle(&global_steer, servo_cmd);

    float base_float = (float)ARC_BASE_TICKS_PER_DT;
    float yaw_error = fabsf(target - current_yaw);
    if (yaw_error < ARC_SLOW_BAND_DEG) {
      float ratio = yaw_error / ARC_SLOW_BAND_DEG;
      base_float = (float)TURN_MIN_TICKS_PER_DT +
                   ((float)ARC_BASE_TICKS_PER_DT - (float)TURN_MIN_TICKS_PER_DT) * ratio;
      if (base_float < (float)TURN_MIN_TICKS_PER_DT) {
        base_float = (float)TURN_MIN_TICKS_PER_DT;
      }
    }

    if (ms.arc.segment_hold_ticks > 0U) {
      base_float = (float)TURN_MIN_TICKS_PER_DT;
      ms.arc.segment_hold_ticks--;
    }

    int32_t base_ticks = (int32_t)lroundf(base_float);
    if (base_ticks < TURN_MIN_TICKS_PER_DT) {
      base_ticks = TURN_MIN_TICKS_PER_DT;
    }
    int32_t inner_ticks = base_ticks;
    int32_t outer_ticks = (int32_t)lroundf((float)base_ticks * ARC_OUTER_RATIO);
    if (outer_ticks < inner_ticks) {
      outer_ticks = inner_ticks;
    }

    int32_t left_target = base_ticks;
    int32_t right_target = base_ticks;
    if (servo_cmd > 0.0f) {
      // Turning right: left wheel is outer.
      left_target = outer_ticks;
      right_target = inner_ticks;
    } else if (servo_cmd < 0.0f) {
      // Turning left: right wheel is outer.
      left_target = inner_ticks;
      right_target = outer_ticks;
    }

    control_set_target_ticks_per_dt(left_target, right_target);

    uint8_t advance = 0U;
    if (target > prev_target) {
      if (current_yaw >= (target - ARC_YAW_TOLERANCE_DEG)) {
        advance = 1U;
      }
    } else {
      if (current_yaw <= (target + ARC_YAW_TOLERANCE_DEG)) {
        advance = 1U;
      }
    }

    if (advance) {
      ms.arc.segment_index = idx + 1U;
      if (ms.arc.segment_index < 2U) {
        ms.arc.segment_hold_ticks = ARC_SEGMENT_MIN_HOLD_TICKS;
      } else if (ms.arc.segment_index == 2U) {
        ms.arc.segment_hold_ticks = ARC_SEGMENT_MIN_HOLD_TICKS;
      } else {
        Servo_Center(&global_steer);
        ms.steer_cmd = 0.0f;
        control_set_target_ticks_per_dt(0, 0);
        ms.arc.straighten_delay_ticks = ARC_STRAIGHTEN_DELAY_TICKS;
      }
    }
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
  int32_t yaw_bias = (int32_t)lroundf(-filtered_yaw * YAW_KP_TICKS_PER_DEG);
  if (ms.dir < 0) yaw_bias = -yaw_bias;
  if (yaw_bias > base_ticks/2) yaw_bias = base_ticks/2;
  if (yaw_bias < -(base_ticks/2)) yaw_bias = -(base_ticks/2);
  
  // 6) Combine base and yaw correction
  int32_t left_target_ticks  = base_ticks - yaw_bias;
  int32_t right_target_ticks = base_ticks + yaw_bias;

  // 6. Set the targets for the low-level PID speed controllers
  int32_t left_final = left_target_ticks * ms.dir;
  int32_t right_final = right_target_ticks * ms.dir;
  if (ms.dir < 0) {
    left_final -= 2; // reverse-only trim to compensate asymmetry
  }
  control_set_target_ticks_per_dt(left_final, right_final);
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
