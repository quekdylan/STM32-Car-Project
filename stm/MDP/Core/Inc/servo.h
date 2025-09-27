#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TIM_HandleTypeDef *htim;   // Timer handle
    uint32_t channel;          // Timer channel (e.g., TIM_CHANNEL_1)
    float tick_us;             // Timer tick in microseconds (after prescaler)
    uint32_t arr;              // Auto-reload register (Period)
    uint16_t min_us;           // Min pulse width (µs) = full left
    uint16_t center_us;        // Center pulse width (µs)
    uint16_t max_us;           // Max pulse width (µs) = full right
} Servo;

/**
 * Attach a servo object to a timer channel.
 * tick_us = timer tick in µs after prescaler.
 */
void Servo_Attach(Servo *s,
                  TIM_HandleTypeDef *htim,
                  uint32_t channel,
                  float tick_us,
                  uint16_t min_us,
                  uint16_t center_us,
                  uint16_t max_us);

/** Start PWM output on the servo’s channel */
HAL_StatusTypeDef Servo_Start(Servo *s);

/** Stop PWM output on the servo’s channel */
HAL_StatusTypeDef Servo_Stop(Servo *s);

/** Write pulse width in microseconds (clamped between min and max) */
void Servo_WriteUS(Servo *s, uint16_t pulse_us);

/** Write servo “angle” (−100 to +100 → maps to min..max around center) */
void Servo_WriteAngle(Servo *s, float deg);

/** Reset servo to center position */
static inline void Servo_Center(Servo *s) { Servo_WriteUS(s, s->center_us); }

/** Update limits (useful after calibration) */
static inline void Servo_SetLimits(Servo *s, uint16_t min_us, uint16_t center_us, uint16_t max_us) {
    s->min_us = min_us;
    s->center_us = center_us;
    s->max_us = max_us;
}

/** Write raw CCR value (counts) directly — for backward compatibility */
static inline void Servo_WriteCCR(Servo *s, uint32_t ccr_counts) {
    if (!s || !s->htim) return;
    if (ccr_counts > s->arr) ccr_counts = s->arr;
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, ccr_counts);
}

/** Convert µs → CCR counts */
static inline uint32_t Servo_US2CCR(const Servo *s, uint16_t us) {
    float c = us / s->tick_us;
    if (c < 0) c = 0;
    if (c > (float)s->arr) c = (float)s->arr;
    return (uint32_t)(c + 0.5f);
}

#define SERVO_LEFT_LIMIT_US   1084U
#define SERVO_CENTER_US       1508U
#define SERVO_RIGHT_LIMIT_US  2690U

#ifdef __cplusplus
}
#endif

#endif // SERVO_H
