/*
 * sensor.c
 *
 * Created on: Sep 15, 2025
 * Author: YourName
 */

 #include "sensor.h"
 #include <stdio.h>
 #include "stm32f4xx_hal.h"
 
 // === PRIVATE DEFINES ===
 #define SPEED_OF_SOUND_CM_PER_US (0.0343f) // Speed of sound in cm/µs
 
 // === PRIVATE VARIABLES ===
 // Pointers to hardware configured in main.c
 static TIM_HandleTypeDef* gp_htim_echo;
 static uint32_t g_ic_channel = TIM_CHANNEL_1;
static float g_tick_us = 1.0f; // timer tick period in microseconds
 static uint8_t g_disabled = 0; // set if bound to a conflicting timer (e.g., TIM8 used by servo)
 extern TIM_HandleTypeDef htim8; // to guard against servo timer usage
 static GPIO_TypeDef* gp_trig_port;
 static uint16_t g_trig_pin;
 static volatile uint8_t g_is_first_capture = 1; // 1 for first edge (rising), 0 for second (falling)

 
 // State machine for input capture
 typedef enum
 {
     IDLE,
     WAITING_FOR_RISING_EDGE,
     WAITING_FOR_FALLING_EDGE
 } CaptureState;
 
 // Volatile variables modified by ISR
 static volatile CaptureState g_capture_state = IDLE;
 static volatile uint32_t g_rising_edge_time = 0;
 static volatile uint32_t g_falling_edge_time = 0;
static volatile float g_distance_cm = 0.0f;
static volatile uint8_t g_busy = 0; // set when a measurement is in progress

// === LOCAL UTILS ===
// Busy-wait for the requested microseconds using DWT cycle counter when available
static void delay_us(uint32_t us)
{
    // Enable DWT CYCCNT if not already enabled
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}
 
 
 // === PUBLIC FUNCTION IMPLEMENTATIONS ===
 
void ultrasonic_init_ex(TIM_HandleTypeDef *htim_echo, uint32_t ic_channel,
                       GPIO_TypeDef* trig_port, uint16_t trig_pin,
                       float tick_us)
{
     gp_htim_echo = htim_echo;
     gp_trig_port = trig_port;
     g_trig_pin = trig_pin;
    g_ic_channel = ic_channel;
    g_tick_us = (tick_us > 0.0f) ? tick_us : 1.0f;
    // Guard: do not attach to TIM8 (servo)
    g_disabled = (gp_htim_echo && gp_htim_echo->Instance == htim8.Instance) ? 1 : 0;
    g_busy = 0;
    g_capture_state = IDLE;
    g_is_first_capture = 1;
}

 void ultrasonic_init(TIM_HandleTypeDef *htim_echo, GPIO_TypeDef* trig_port, uint16_t trig_pin)
 {
     ultrasonic_init_ex(htim_echo, TIM_CHANNEL_1, trig_port, trig_pin, 1.0f);
 }
 
void ultrasonic_trigger(void)
{
    if (g_disabled || gp_htim_echo == NULL || gp_trig_port == NULL) return;
    // Don't start a new measurement if one is already in progress
    if (g_capture_state != IDLE) {
        return;
    }

    // 1. Set the trigger pin HIGH for 10 microseconds
    HAL_GPIO_WritePin(gp_trig_port, g_trig_pin, GPIO_PIN_SET);
    // Accurate ~10us pulse
    delay_us(12);
    
    HAL_GPIO_WritePin(gp_trig_port, g_trig_pin, GPIO_PIN_RESET);

    // 2. Configure for rising edge capture and start
    TIM_IC_InitTypeDef sConfig = {0};
    sConfig.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfig.ICPrescaler = TIM_ICPSC_DIV1;
    // Add a small digital filter to suppress spurious edges on ECHO
    sConfig.ICFilter    = 8; // 0..15, higher = more filtering (sampling clock dependent)
    HAL_TIM_IC_ConfigChannel(gp_htim_echo, &sConfig, g_ic_channel);
    g_is_first_capture = 1;
    g_capture_state = WAITING_FOR_RISING_EDGE;
    g_busy = 1;
    __HAL_TIM_SET_COUNTER(gp_htim_echo, 0);
    HAL_TIM_IC_Start_IT(gp_htim_echo, g_ic_channel);
}
 
float ultrasonic_get_distance_cm(void)
{
    return g_distance_cm;
}

uint8_t ultrasonic_is_busy(void)
{
    return (g_capture_state != IDLE) ? 1U : 0U;
}

void ultrasonic_cancel(void)
{
    if (g_disabled || gp_htim_echo == NULL) {
        g_capture_state = IDLE;
        g_is_first_capture = 1;
        g_busy = 0;
        g_distance_cm = 0.0f;
        return;
    }
    HAL_TIM_IC_Stop_IT(gp_htim_echo, g_ic_channel);
    g_is_first_capture = 1;
    g_capture_state = IDLE;
    g_busy = 0;
    g_distance_cm = 0.0f;
}
 
 
 // === INTERRUPT CALLBACK IMPLEMENTATION ===
 
 /**
   * @brief  Input Capture callback. This is called by the HAL_TIM_IRQHandler.
   * @param  htim: pointer to the TIM_HandleTypeDef structure.
   */
  void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
  {
      // Ensure the interrupt is from our echo timer
      if (g_disabled || gp_htim_echo == NULL) return;
      if (htim->Instance == gp_htim_echo->Instance)
      {
          if (g_is_first_capture)
          {
              // --- First capture: Rising Edge ---
              g_rising_edge_time = HAL_TIM_ReadCapturedValue(htim, g_ic_channel);
              g_is_first_capture = 0; // Next capture is falling edge
              g_capture_state = WAITING_FOR_FALLING_EDGE;
              // Switch to falling edge
              TIM_IC_InitTypeDef sConfig = {0};
              sConfig.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
              sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
              sConfig.ICPrescaler = TIM_ICPSC_DIV1;
              sConfig.ICFilter    = 8;
              HAL_TIM_IC_Stop_IT(htim, g_ic_channel);
              HAL_TIM_IC_ConfigChannel(htim, &sConfig, g_ic_channel);
              HAL_TIM_IC_Start_IT(htim, g_ic_channel);
          }
          else
          {
              // --- Second capture: Falling Edge ---
              g_falling_edge_time = HAL_TIM_ReadCapturedValue(htim, g_ic_channel);
  
              // Stop the timer to prevent further interrupts until the next trigger
              HAL_TIM_IC_Stop_IT(htim, g_ic_channel);
  
              // Calculate the pulse duration
              uint32_t pulse_ticks;
              if (g_falling_edge_time >= g_rising_edge_time) {
                  pulse_ticks = g_falling_edge_time - g_rising_edge_time;
              } else { // Timer overflowed
                  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(gp_htim_echo);
                  pulse_ticks = (arr + 1u - g_rising_edge_time) + g_falling_edge_time;
              }
  
              // Calculate the distance
              float pulse_us = (float)pulse_ticks * g_tick_us;
              g_distance_cm = (pulse_us * SPEED_OF_SOUND_CM_PER_US) / 2.0f;
  
              // --- Cleanup for next measurement ---
              g_is_first_capture = 1; // Reset flag for the next trigger
              g_capture_state = IDLE; // Ready for another trigger
              g_busy = 0;
            }
        }
    }
