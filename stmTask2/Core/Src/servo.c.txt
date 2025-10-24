#include "servo.h"

static inline uint32_t _get_arr(TIM_HandleTypeDef *htim) {
    return __HAL_TIM_GET_AUTORELOAD(htim);
}

void Servo_Attach(Servo *s,
                  TIM_HandleTypeDef *htim,
                  uint32_t channel,
                  float tick_us,
                  uint16_t min_us,
                  uint16_t center_us,
                  uint16_t max_us) {
    if (!s) return;
    s->htim       = htim;
    s->channel    = channel;
    s->tick_us    = tick_us;
    s->arr        = _get_arr(htim);
    s->min_us     = min_us;
    s->center_us  = center_us;
    s->max_us     = max_us;
}

HAL_StatusTypeDef Servo_Start(Servo *s) {
    if (!s || !s->htim) return HAL_ERROR;
    HAL_StatusTypeDef st = HAL_TIM_PWM_Start(s->htim, s->channel);
    if (st != HAL_OK) return st;
    Servo_Center(s); // snap to center on start
    return HAL_OK;
}

HAL_StatusTypeDef Servo_Stop(Servo *s) {
    if (!s || !s->htim) return HAL_ERROR;
    return HAL_TIM_PWM_Stop(s->htim, s->channel);
}

void Servo_WriteUS(Servo *s, uint16_t pulse_us) {
    if (!s || !s->htim) return;
    // clamp to limits
    if (pulse_us < s->min_us) pulse_us = s->min_us;
    if (pulse_us > s->max_us) pulse_us = s->max_us;
    uint32_t ccr = Servo_US2CCR(s, pulse_us);
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, ccr);
}

void Servo_WriteAngle(Servo *s, float deg) {
    if (!s) return;
    if (deg < 0) {
        // map [−100..0] → [min_us..center_us]
        float t = deg / -100.0f; // 0..1
        uint16_t us = (uint16_t)((float)s->center_us - t * (s->center_us - s->min_us));
        Servo_WriteUS(s, us);
    } else {
        // map [0..+100] → [center_us..max_us]
        float t = deg / 100.0f; // 0..1
        uint16_t us = (uint16_t)((float)s->center_us + t * (s->max_us - s->center_us));
        Servo_WriteUS(s, us);
    }
}



