// Simple motor speed control API using TIM5 @ 100 Hz
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

// Initialize PID controllers and start TIM5 interrupts
void control_init(void);

// Set targets in encoder ticks per control period (10 ms)
void control_set_target_ticks_per_dt(int32_t left_ticks, int32_t right_ticks);

// Called from TIM5 ISR every 10 ms
void control_tick(void);

// Get current targets and measured tick deltas (per 10 ms)
void control_get_target_and_measured(int32_t *t_left,
                                     int32_t *t_right,
                                     int32_t *m_left,
                                     int32_t *m_right);

// Get last PWM outputs (percent -100..100) applied to motors
void control_get_outputs(int8_t *out_left, int8_t *out_right);

// Check if control step is due (100 Hz timing)
uint8_t control_is_due(void);

// Clear the control due flag
void control_clear_due(void);

// Run one control step (encoders -> PID -> PWM)
void control_step(void);

/*
Tuning notes (what you need to set):
- Gains: start with KP=2.0, KI=0.5, KD=0.01 for 100 Hz; then tune.
- Max output: 100 (maps to motor_set_speeds percent).
- Integral clamp: 20–60 typical; prevents windup.
- Derivative clamp: 20–60 typical; reduces spikes.
- Target units: encoder ticks per 10 ms. Example: if open-loop 100% gives ~40 ticks/10 ms, to command ~50% speed set target ~20 ticks/10 ms.
*/

#endif // CONTROL_H
