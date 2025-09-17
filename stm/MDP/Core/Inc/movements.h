// movements.h
#pragma once
#include <stdint.h>

// Start a straight movement for the given distance in centimeters.
// Positive = forward, Negative = backward.
void move_start_straight(float distance_cm);

// Start a 90-degree in-place turn using the IMU for feedback.
// dir: 'L' or 'l' for left (CCW), 'R' or 'r' for right (CW).
void move_start_turn(char dir);

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
