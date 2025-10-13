// movements.h
#pragma once
#include <stdint.h>

typedef enum {
  MOVE_TURN_PROFILE_PRIMARY = 0,
  MOVE_TURN_PROFILE_SECONDARY = 1
} move_turn_profile_e;

typedef enum {
  MOVE_ARC_SIDE_LEFT  = -1,
  MOVE_ARC_SIDE_RIGHT = +1
} move_arc_side_e;

// Start a straight movement for the given distance in centimeters.
// Positive = forward, Negative = backward.
void move_start_straight(float distance_cm);

// Start a 90-degree in-place turn using the IMU for feedback.
// dir: 'L' or 'l' for left (CCW), 'R' or 'r' for right (CW).
void move_start_turn(char dir);

// Start a fast turn (higher speed).
// dir: 'L' or 'l' for left (CCW), 'R' or 'r' for right (CW).
void move_start_fast_turn(char dir);

// Start a fast turn with custom angle.
// dir: 'L' for left (CCW), 'R' for right (CW).
// angle_deg: desired turn angle in degrees.
void move_start_fast_turn_angle(char dir, float angle_deg);

// Start an arc maneuver around a fixed obstacle. Side selects left/right bypass.
void move_start_arc(move_arc_side_e side);

// Drive forward until ultrasonic distance falls below the provided threshold or max distance.
// Optional service hook (e.g. to poll serial) is called inside the loop; pass NULL if unused.
void move_drive_until_obstacle(float stop_threshold_cm, void (*service_hook)(void));

// Abort an ongoing move and stop motors immediately.
void move_abort(void);

// Returns non-zero if a movement is currently active.
uint8_t move_is_active(void);

// Call this at 100 Hz (every 10 ms) while a movement is active.
void move_tick_100Hz(void);

// Configure fixed accel/decel distances (in centimeters)
void move_set_profile_distances_cm(float accel_cm, float decel_cm);
void move_get_profile_distances_cm(float *accel_cm, float *decel_cm);

// Configure the duration (in 100 Hz ticks) that turns stay at TURN_MIN_TICKS_PER_DT
void move_set_turn_spinup_ticks(uint16_t ticks_100Hz);
uint16_t move_get_turn_spinup_ticks(void);

// Select which turn profile to use for subsequent turns.
void move_set_turn_profile(move_turn_profile_e profile);
move_turn_profile_e move_get_turn_profile(void);
