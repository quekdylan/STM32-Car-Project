// movements.h
#pragma once
#include <stdint.h>

typedef enum {
  SPEED_MODE_1 = 1,
  SPEED_MODE_2 = 2
} speed_mode_t;

void move_start_straight(float distance_cm, speed_mode_t mode);
void move_abort(void);
uint8_t move_is_active(void);
void move_tick_100Hz(void);   // call every 10 ms from your control task