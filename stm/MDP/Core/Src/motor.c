#include "motor.h"
#include <math.h> // Required for fabsf()

// --- Hardware Handles (defined in main.c) ---
extern TIM_HandleTypeDef htim2; // Motor A (Left) Encoder on PA15, PB3
extern TIM_HandleTypeDef htim3; // Motor B (Right) Encoder on PA6, PA7
extern TIM_HandleTypeDef htim4; // Motor A (Left) PWM on PB8, PB9
extern TIM_HandleTypeDef htim9; // Motor B (Right) PWM on PE5, PE6

// --- Constants ---
#define MIN_MOTOR_SPEED_PERCENT 54.0f

void motor_init(void) {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // Motor A IN1 (PB9)
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // Motor A IN2 (PB8)
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); // Motor B IN1 (PE5)
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); // Motor B IN2 (PE6)

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Motor A (Left)
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Motor B (Right)

    motor_stop();
}

void motor_set_speeds(int8_t left_speed_percent, int8_t right_speed_percent) {
    if (left_speed_percent > 100) left_speed_percent = 100;
    if (left_speed_percent < -100) left_speed_percent = -100;
    if (right_speed_percent > 100) right_speed_percent = 100;
    if (right_speed_percent < -100) right_speed_percent = -100;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim4);

    // --- Left Motor (Motor A on TIM4) ---
    if (left_speed_percent == 0) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    } else {
        float scaled_speed = MIN_MOTOR_SPEED_PERCENT + (fabsf((float)left_speed_percent) / 100.0f) * (100.0f - MIN_MOTOR_SPEED_PERCENT);
        uint32_t pulse = (uint32_t)(scaled_speed * (arr + 1) / 100.0f);

        if (left_speed_percent > 0) {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        } else {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse);
        }
    }

    // --- Right Motor (Motor B on TIM9) ---
    if (right_speed_percent == 0) {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    } else {
        float scaled_speed = MIN_MOTOR_SPEED_PERCENT + (fabsf((float)right_speed_percent) / 100.0f) * (100.0f - MIN_MOTOR_SPEED_PERCENT);
        uint32_t pulse = (uint32_t)(scaled_speed * (arr + 1) / 100.0f);

        // ** MODIFIED LOGIC FOR MOTOR B DIRECTION **
        if (right_speed_percent > 0) { // Forward
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);     // IN1 = 0
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, pulse); // IN2 = PWM
        } else { // Reverse
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pulse); // IN1 = PWM
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);     // IN2 = 0
        }
    }
}

void motor_stop(void) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
}

int32_t motor_get_left_encoder_counts(void) {
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

// Right motor is reversed, so return the negative.
int32_t motor_get_right_encoder_counts(void) {
	return -((int16_t)__HAL_TIM_GET_COUNTER(&htim3));
}

void motor_reset_encoders(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

