#include <stdint.h>
#include "main.h"
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
    uint32_t arrL;
    uint32_t arrR;
    float scaled_speedL;
    float scaled_speedR;
    uint32_t pulseL;
    uint32_t pulseR;

    if (left_speed_percent > 100)  left_speed_percent  = 100;
    if (left_speed_percent < -100) left_speed_percent  = -100;
    if (right_speed_percent > 100) right_speed_percent = 100;
    if (right_speed_percent < -100)right_speed_percent = -100;

    // Use each timer's own ARR
    arrL = __HAL_TIM_GET_AUTORELOAD(&htim4); // TIM4 (Left)
    arrR = __HAL_TIM_GET_AUTORELOAD(&htim9); // TIM9 (Right)

    // --- Left Motor (TIM4) ---
    if (left_speed_percent == 0) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    } else {
        scaled_speedL = MIN_MOTOR_SPEED_PERCENT + (fabsf((float)left_speed_percent) / 100.0f) * (100.0f - MIN_MOTOR_SPEED_PERCENT);
        pulseL = (uint32_t)((scaled_speedL * (float)(arrL + 1)) / 100.0f);

        if (left_speed_percent > 0) {           // Forward
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulseL); // IN1 = PWM
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);      // IN2 = 0
        } else {                                  // Reverse
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);      // IN1 = 0
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulseL); // IN2 = PWM
        }
    }

    // --- Right Motor (TIM9) ---
    if (right_speed_percent == 0) {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    } else {
        scaled_speedR = MIN_MOTOR_SPEED_PERCENT + (fabsf((float)right_speed_percent) / 100.0f) * (100.0f - MIN_MOTOR_SPEED_PERCENT);
        pulseR = (uint32_t)((scaled_speedR * (float)(arrR + 1)) / 100.0f);

        // Your mapping shows right motor wiring reversed; keep your chosen convention:
        if (right_speed_percent > 0) {            // Forward
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);      // IN1 = 0
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, pulseR); // IN2 = PWM
        } else {                                   // Reverse
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pulseR); // IN1 = PWM
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);      // IN2 = 0
        }
    }
}


void motor_stop(void) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
}

void motor_brake_ms(uint16_t ms) {
    // Set both inputs HIGH on each H-bridge to short-brake
    uint32_t arrL = __HAL_TIM_GET_AUTORELOAD(&htim4);
    uint32_t arrR = __HAL_TIM_GET_AUTORELOAD(&htim9);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, arrL); // IN1 = 100%
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, arrL); // IN2 = 100%
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, arrR); // IN1 = 100%
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, arrR); // IN2 = 100%

    if (ms > 0) {
        HAL_Delay(ms);
    }

    // Release to zero to avoid heating the bridge
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
}

void motor_brake(void) {
    motor_brake_begin();
}

void motor_brake_begin(void) {
    // Set both inputs HIGH on each H-bridge to short-brake
    uint32_t arrL = __HAL_TIM_GET_AUTORELOAD(&htim4);
    uint32_t arrR = __HAL_TIM_GET_AUTORELOAD(&htim9);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, arrL); // IN1 = 100%
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, arrL); // IN2 = 100%
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, arrR); // IN1 = 100%
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, arrR); // IN2 = 100%
}

void motor_brake_end(void) {
    // Release to zero to avoid heating the bridge
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
}

int32_t motor_get_left_encoder_counts(void) {
    return (int32_t)((int16_t)__HAL_TIM_GET_COUNTER(&htim2));
}

// Right motor is reversed, so return the negative.
int32_t motor_get_right_encoder_counts(void) {
	return -((int16_t)__HAL_TIM_GET_COUNTER(&htim3));
}

void motor_reset_encoders(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}
