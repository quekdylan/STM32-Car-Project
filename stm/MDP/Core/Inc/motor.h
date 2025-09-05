#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

/**
 * @brief Initializes the PWM and Encoder peripherals.
 * This function should be called once before the RTOS scheduler starts.
 */
void motor_init(void);

/**
 * @brief Sets the speed and direction for each motor independently.
 * Normalizes the input speed [0-100] to the effective PWM range [54-100].
 * @param left_speed_percent Speed for the left motor (-100 to 100). Negative is reverse.
 * @param right_speed_percent Speed for the right motor (-100 to 100). Negative is reverse.
 */
void motor_set_speeds(int8_t left_speed_percent, int8_t right_speed_percent);

/**
 * @brief Stops both motors by setting PWM to zero (coast).
 */
void motor_stop(void);

/**
 * @brief Gets the raw encoder tick count for the left motor.
 */
int32_t motor_get_left_encoder_counts(void);

/**
 * @brief Gets the raw encoder tick count for the right motor.
 */
int32_t motor_get_right_encoder_counts(void);

/**
 * @brief Resets the encoder counts for both motors to zero.
 */
void motor_reset_encoders(void);

#endif /* MOTOR_H */
