// sensor.h - Ultrasonic distance sensor API
#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize ultrasonic measurement with explicit IC channel and tick period in microseconds
// tick_us: timer tick duration in microseconds (e.g., 1.0f if timer is 1 MHz)
void ultrasonic_init_ex(TIM_HandleTypeDef *htim_echo, uint32_t ic_channel,
						GPIO_TypeDef* trig_port, uint16_t trig_pin,
						float tick_us);

// Back-compat simple init: assumes Channel 1 and 1.0 us per tick (only valid if timer configured to 1 MHz)
void ultrasonic_init(TIM_HandleTypeDef *htim_echo, GPIO_TypeDef* trig_port, uint16_t trig_pin);

// Fire a 10us trigger pulse and start input capture (non-blocking)
void ultrasonic_trigger(void);

// Get last measured distance in centimeters (float). Returns 0.0f if no reading yet.
float ultrasonic_get_distance_cm(void);

#ifdef __cplusplus
}
#endif

