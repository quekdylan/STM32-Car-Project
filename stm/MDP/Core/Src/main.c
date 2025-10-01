/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "control.h"
#include "userButton.h"
#include "servo.h"
#include "ICM20948.h"
#include "motor.h"
#include "pid.h"
#include "movements.h"
#include "sensor.h"
#include "commands.h"
#include <stdio.h>
#include <string.h>  // for strlen
#include <math.h>
#include <ctype.h>
#include <stdlib.h>
#include <limits.h>
#include "imu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define UART_CMD_BUF_SIZE 64U
#define UART_RING_BUFFER_SIZE 256U

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_task */
osThreadId_t UART_taskHandle;
const osThreadAttr_t UART_task_attributes = {
  .name = "UART_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
// Global servo object for use by defaultTask
Servo global_steer;

// Current high-level instruction being executed (e.g., 'T','t','L','R','l','r').
// 0 means none; shown as '-' on OLED.
volatile char g_current_instr = 0;

// Flag while the IMU is undergoing calibration so the UI can show status.
volatile uint8_t g_is_calibrating = 0;

// Servo calibration results
volatile float g_left_avg_rate = 0.0f; // Store left turn average rate
volatile float g_right_avg_rate = 0.0f; // Store right turn average rate
volatile uint8_t g_show_results = 0; // Flag to show results on next button press

// Current servo center position for display
volatile uint16_t g_current_servo_pos = SERVO_CENTER_US;

// Last UART character received (for OLED display)
volatile uint8_t g_uart_last_char = 0;
volatile uint8_t g_uart_char_valid = 0;

static uint8_t uart_ring_buffer[UART_RING_BUFFER_SIZE];
static volatile uint16_t uart_ring_head = 0;
static volatile uint16_t uart_ring_tail = 0;
static uint8_t uart_cmd_buffer[UART_CMD_BUF_SIZE];
static uint16_t uart_cmd_length = 0;

#define GYRO_LOG_PERIOD_MS 1000U

typedef struct {
  uint8_t have_prev_sample;
  float   yaw_unwrapped_deg;
  float   prev_yaw_deg;
} gyro_log_state_t;

static gyro_log_state_t g_gyro_log = {0};





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM12_Init(void);
void StartDefaultTask(void *argument);
void control_task(void *argument);
void oled_task(void *argument);
void uart_task(void *argument);

/* USER CODE BEGIN PFP */
static void execute_straight_move(float distance_cm, uint32_t brake_ms);
static void execute_turn_move(char turn_opcode, float angle_deg, uint32_t brake_ms);
static void configure_servo_for_motion(void);
static void perform_initial_calibration(void);
static void process_serial_input(void);
static void execute_command(Command *cmd);
static void wait_for_motion_completion(void);
static char determine_turn_opcode(const Command *cmd);
static uint16_t ring_advance(uint16_t index);
typedef struct {
  char opcode;
  int param;
} scripted_move_t;

#define MAX_SCRIPT_STEPS 16U
#define SCRIPT_DEFAULT_BRAKE_MS 50U

// Edit this string to change the scripted sequence executed after calibration.
static char g_instruction_plan[] = "[['L', 90],['R', 90]]";

static size_t parse_instruction_plan(scripted_move_t *steps, size_t max_steps);
static void run_instruction_plan(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// UART3 RX byte buffer (interrupt-driven)
static uint8_t uart3_rx_byte = 0;

static void uart_send_response(const char *msg);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    uint16_t next = ring_advance(uart_ring_head);
    if (next == uart_ring_tail) {
      uart_ring_tail = ring_advance(uart_ring_tail);
    }

    uart_ring_buffer[uart_ring_head] = uart3_rx_byte;
    uart_ring_head = next;

    if (isprint((unsigned char)uart3_rx_byte)) {
      g_uart_last_char = (uint8_t)uart3_rx_byte;
      g_uart_char_valid = 1U;
    }

    HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    uart_cmd_length = 0U;
    (void)HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
  }
}

static size_t parse_instruction_plan(scripted_move_t *steps, size_t max_steps)
{
  size_t count = 0U;
  const char *cursor = g_instruction_plan;

  while (*cursor != '\0' && count < max_steps) {
    while (*cursor != '\0' && *cursor != '\'' && *cursor != '"') {
      cursor++;
    }

    if (*cursor == '\0' || cursor[1] == '\0') {
      break;
    }

    char quote_char = *cursor;
    char opcode = cursor[1];
    const char *closing = strchr(cursor + 2, quote_char);
    if (!closing) {
      break;
    }

    const char *num_start = closing + 1;
    while (*num_start != '\0' && *num_start != '-' && !isdigit((unsigned char)*num_start)) {
      if (*num_start == '[') {
        break;
      }
      num_start++;
    }

    if (*num_start == '\0' || *num_start == '[') {
      cursor = closing + 1;
      continue;
    }

    char *end_ptr = NULL;
    long value = strtol(num_start, &end_ptr, 10);
    if (end_ptr == num_start) {
      cursor = closing + 1;
      continue;
    }

    steps[count].opcode = opcode;
    steps[count].param = (int)value;
    count++;
    cursor = end_ptr;
  }

  return count;
}

static void configure_servo_for_motion(void)
{
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  uint32_t timclk = pclk2; // APB2 prescaler = 1
  float tick_us = ((float)(htim8.Init.Prescaler + 1U)) * (1000000.0f / (float)timclk);

  Servo_Attach(&global_steer, &htim8, TIM_CHANNEL_1, tick_us,
               SERVO_LEFT_LIMIT_US, SERVO_CENTER_US, SERVO_RIGHT_LIMIT_US);

  if (Servo_Start(&global_steer) != HAL_OK) {
    for(;;) { osDelay(1000); }
  }

  Servo_Center(&global_steer);
}

static void perform_initial_calibration(void)
{
  g_is_calibrating = 1;
  osDelay(1000);
  motor_stop();
  imu_calibrate_bias_blocking(1000);
  imu_zero_yaw();
  g_is_calibrating = 0;

  g_gyro_log.yaw_unwrapped_deg = 0.0f;
  g_gyro_log.prev_yaw_deg = 0.0f;
  g_gyro_log.have_prev_sample = 0U;
}

static void reset_motion_state(void)
{
  Servo_Center(&global_steer);
  control_set_target_ticks_per_dt(0, 0);
  motor_stop();
  g_current_instr = 0;
}

static void run_instruction_plan(void)
{
  scripted_move_t steps[MAX_SCRIPT_STEPS];
  size_t step_count = parse_instruction_plan(steps, MAX_SCRIPT_STEPS);

  for (size_t i = 0; i < step_count; ++i) {
    char opcode = steps[i].opcode;
    int param = steps[i].param;

    if (opcode == CMD_FORWARD_DIST_TARGET || opcode == CMD_BACKWARD_DIST_TARGET) {
      float distance_cm = (opcode == CMD_BACKWARD_DIST_TARGET) ? -fabsf((float)param) : fabsf((float)param);
      char dbg[48];
      (void)snprintf(dbg, sizeof(dbg), "script %c %d\r\n", opcode, param);
      uart_send_response(dbg);
      execute_straight_move(distance_cm, SCRIPT_DEFAULT_BRAKE_MS);
    } else if (opcode == 'L' || opcode == 'R' || opcode == 'l' || opcode == 'r') {
      float angle_deg = fabsf((float)param);
      if (angle_deg <= 0.0f) {
        angle_deg = 90.0f;
      }
      char dbg[48];
      (void)snprintf(dbg, sizeof(dbg), "script turn %c %d\r\n", opcode, (int)angle_deg);
      uart_send_response(dbg);
      execute_turn_move(opcode, angle_deg, SCRIPT_DEFAULT_BRAKE_MS);
    } else {
      continue;
    }

    Servo_Center(&global_steer);
    motor_stop();
    osDelay(20);
  }
}

static void execute_straight_move(float distance_cm, uint32_t brake_ms)
{
  if (distance_cm == 0.0f) {
    return;
  }

  g_current_instr = (distance_cm >= 0.0f) ? CMD_FORWARD_DIST_TARGET : CMD_BACKWARD_DIST_TARGET;
  move_start_straight(distance_cm);
  wait_for_motion_completion();
  g_current_instr = 0;
  control_set_target_ticks_per_dt(0, 0);
  motor_brake_ms(brake_ms);
  motor_stop();
}

static void execute_turn_move(char turn_opcode, float angle_deg, uint32_t brake_ms)
{
  g_current_instr = turn_opcode;
  move_turn(turn_opcode, angle_deg);
  wait_for_motion_completion();
  g_current_instr = 0;
  control_set_target_ticks_per_dt(0, 0);
  motor_brake_ms(brake_ms);
  motor_stop();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
OLED_Init();
motor_init();          // Initialize motors (PWM + encoders)
control_init();        // Start 100 Hz control loop
ultrasonic_init(&htim12, GPIOB, GPIO_PIN_15);

// Initialize IMU (detects 0x68/0x69, sets 2000 dps; calibration happens later)
uint8_t icm_addrSel = 0;
imu_init(&hi2c2, &icm_addrSel);

  g_gyro_log.yaw_unwrapped_deg = 0.0f;
  g_gyro_log.prev_yaw_deg = 0.0f;
  g_gyro_log.have_prev_sample = 0U;

commands_init();

// Arm first RX
HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);

// Optional startup message
const char *hello = "STM32 UART ready\r\n";
HAL_UART_Transmit(&huart3, (uint8_t*)hello, strlen(hello), HAL_MAX_DELAY);


// (Optional) show which address was used on OLED/UART

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* No RTOS queues used */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(control_task, NULL, &controlTask_attributes);

  /* creation of oledTask */
  oledTaskHandle = osThreadNew(oled_task, NULL, &oledTask_attributes);

  /* creation of UART_task */
  UART_taskHandle = osThreadNew(uart_task, NULL, &UART_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 799;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1599;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 15;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 19999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 799;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 15;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim12, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(trigger_pin_GPIO_Port, trigger_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED_DC_Pin|OLED_RES_Pin|OLED_SDA_Pin|OLED_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : trigger_pin_Pin */
  GPIO_InitStruct.Pin = trigger_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(trigger_pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DC_Pin OLED_RES_Pin OLED_SDA_Pin OLED_SCL_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin|OLED_RES_Pin|OLED_SDA_Pin|OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN PV */
// UI helpers for progress display
static void oled_draw_default_layout(void)
{
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t*)"Dist(cm):");
  OLED_ShowString(0, 24, (uint8_t*)"RX char:");
  OLED_ShowString(0, 48, (uint8_t*)"Cmd: ");
  OLED_Refresh_Gram();
}

static void format_rate_line(char *out, size_t len, const char *label, float value)
{
  int value10 = (int)(value * 10.0f + (value >= 0.0f ? 0.5f : -0.5f));
  int sign = (value10 < 0) ? -1 : 1;
  if (value10 < 0) value10 = -value10;
  int whole = value10 / 10;
  int tenth = value10 % 10;
  (void)snprintf(out, len, "%s:%c%d.%d dps", label, (sign < 0) ? '-' : ' ', whole, tenth);
}

static void oled_show_avg_screen(float left_rate, float right_rate)
{
  char line[21];

  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t*)"Servo Avg Rate");

  if (fabsf(left_rate) > 0.01f) {
    format_rate_line(line, sizeof(line), "Left", left_rate);
  } else {
    (void)snprintf(line, sizeof(line), "Left:    ---");
  }
  OLED_ShowString(0, 16, (uint8_t*)line);

  if (fabsf(right_rate) > 0.01f) {
    format_rate_line(line, sizeof(line), "Right", right_rate);
  } else {
    (void)snprintf(line, sizeof(line), "Right:   ---");
  }
  OLED_ShowString(0, 32, (uint8_t*)line);

  if (fabsf(left_rate) > 0.01f && fabsf(right_rate) > 0.01f) {
    float ratio = left_rate / right_rate;
    int ratio100 = (int)(ratio * 100.0f + (ratio >= 0.0f ? 0.5f : -0.5f));
    int ratio_int = ratio100 / 100;
    int ratio_frac = ratio100 % 100;
    (void)snprintf(line, sizeof(line), "Ratio: %d.%02d", ratio_int, ratio_frac);
  } else {
    (void)snprintf(line, sizeof(line), "Ratio:   ---");
  }
  OLED_ShowString(0, 48, (uint8_t*)line);
  OLED_Refresh_Gram();
}

static void uart_send_response(const char *msg)
{
  if (msg == NULL) {
    return;
  }

  size_t len = strlen(msg);
  if (len == 0U) {
    return;
  }

  if (len > UINT16_MAX) {
    len = UINT16_MAX;
  }

  (void)HAL_UART_Transmit(&huart3, (uint8_t *)msg, (uint16_t)len, 100);
}

static uint16_t ring_advance(uint16_t index)
{
  return (uint16_t)((index + 1U) % UART_RING_BUFFER_SIZE);
}

static void process_serial_input(void)
{
  while (uart_ring_tail != uart_ring_head) {
    uint8_t byte = uart_ring_buffer[uart_ring_tail];
    uart_ring_tail = ring_advance(uart_ring_tail);

    if (byte == '\r') {
      continue;
    }

    if (uart_cmd_length < UART_CMD_BUF_SIZE) {
      uart_cmd_buffer[uart_cmd_length++] = byte;
    } else {
      uart_cmd_length = 0U;
      continue;
    }

    if (byte == CMD_END) {
      commands_process(&huart3, uart_cmd_buffer, uart_cmd_length);
      uart_cmd_length = 0U;
    }
  }
}

static void wait_for_motion_completion(void)
{
  while (move_is_active()) {
    osDelay(5);
    process_serial_input();
  }
}

static char determine_turn_opcode(const Command *cmd)
{
  if (cmd == NULL) {
    return 'L';
  }

  if (cmd->dir < 0) {
    return (cmd->angleToSteer >= 0.0f) ? 'r' : 'l';
  }

  return (cmd->angleToSteer >= 0.0f) ? 'R' : 'L';
}

static char command_display_code(const Command *cmd)
{
  if (cmd == NULL) {
    return 0;
  }

  switch (cmd->opType) {
    case COMMAND_OP_STOP:
      return CMD_FULL_STOP;

    case COMMAND_OP_TURN:
      return determine_turn_opcode(cmd);

    case COMMAND_OP_DRIVE:
      switch (cmd->distType) {
        case COMMAND_DIST_TARGET:
          return (cmd->dir < 0) ? CMD_BACKWARD_DIST_TARGET : CMD_FORWARD_DIST_TARGET;
        case COMMAND_DIST_STOP_AWAY:
          return (cmd->dir < 0) ? CMD_BACKWARD_DIST_AWAY : CMD_FORWARD_DIST_AWAY;
        case COMMAND_DIST_STOP_L:
          return CMD_FORWARD_DIST_L;
        case COMMAND_DIST_STOP_R:
          return CMD_FORWARD_DIST_R;
        case COMMAND_DIST_STOP_L_LESS:
          return CMD_BACKWARD_DIST_L;
        case COMMAND_DIST_STOP_R_LESS:
          return CMD_BACKWARD_DIST_R;
        default:
          break;
      }
      break;

    case COMMAND_OP_INFO_DIST:
      return CMD_INFO_DIST;

    case COMMAND_OP_INFO_MARKER:
      return CMD_INFO_MARKER;

    case COMMAND_OP_INVALID:
    default:
      break;
  }

  if (cmd->str && cmd->str_size > 0U) {
    char flag = (char)cmd->str[0];
    if (flag != '\0' && flag != CMD_END) {
      return flag;
    }
  }

  return 0;
}

static void execute_command(Command *cmd)
{
  if (cmd == NULL) {
    return;
  }

  char display_code = command_display_code(cmd);
  g_current_instr = display_code;

  switch (cmd->opType) {
    case COMMAND_OP_STOP:
      move_abort();
      motor_stop();
      break;

    case COMMAND_OP_TURN:
    {
      char turn_opcode = determine_turn_opcode(cmd);
      float angle_deg = fabsf(cmd->val);
      if (angle_deg <= 0.0f) {
        angle_deg = fabsf(cmd->angleToSteer);
      }
      if (angle_deg <= 0.0f) {
        angle_deg = 90.0f;
      }

      move_turn(turn_opcode, angle_deg);
      wait_for_motion_completion();
      control_set_target_ticks_per_dt(0, 0);
      motor_brake_ms(50);
      motor_stop();
      break;
    }

    case COMMAND_OP_DRIVE:
      if (cmd->distType == COMMAND_DIST_TARGET) {
        float distance_cm = fabsf(cmd->val);
        if (distance_cm <= 0.0f) {
          break;
        }

        if (cmd->dir < 0) {
          distance_cm = -distance_cm;
        }

        move_start_straight(distance_cm);
        wait_for_motion_completion();
        control_set_target_ticks_per_dt(0, 0);
        motor_brake_ms(50);
        motor_stop();
      }
      break;

    case COMMAND_OP_INFO_DIST:
    case COMMAND_OP_INFO_MARKER:
    case COMMAND_OP_INVALID:
    default:
      break;
  }

  g_current_instr = 0;
  Servo_Center(&global_steer);
  control_set_target_ticks_per_dt(0, 0);
  motor_stop();
}

/* USER CODE END PV */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  configure_servo_for_motion();
  reset_motion_state();

  uint8_t prev_button_state = user_is_pressed();
  uint8_t run_plan_pending = 0U;
  uint8_t did_initial_calibration = 0U;

  for(;;) {
    process_serial_input();

    uint8_t button_state = user_is_pressed();
    if (!prev_button_state && button_state) {
      run_plan_pending = 1U;
    }
    prev_button_state = button_state;

    if (run_plan_pending && !move_is_active()) {
      if (!did_initial_calibration) {
        perform_initial_calibration();
        reset_motion_state();
        did_initial_calibration = 1U;
      }

      run_instruction_plan();
      reset_motion_state();
      run_plan_pending = 0U;
      continue;
    }

    if (move_is_active()) {
      osDelay(5);
      continue;
    }

    Command *cmd = commands_pop();
    if (cmd == NULL) {
      osDelay(5);
      continue;
    }

    execute_command(cmd);
    commands_end(&huart3, cmd);
    process_serial_input();
    osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_control_task */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_control_task */
void control_task(void *argument)
{
  /* USER CODE BEGIN control_task */
  for(;;) {
    if (control_is_due()) {
      control_step();
      move_tick_100Hz();
      control_clear_due();
    }
    osDelay(1);
  }
  /* USER CODE END control_task */
}

/* USER CODE BEGIN Header_oled_task */
/**
* @brief Function implementing the oledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_oled_task */
void oled_task(void *argument)
{
  /* USER CODE BEGIN oled_task */
  // Update OLED at ~5 Hz; initialize static layout once, then update dynamic values only
  uint32_t last_tick = HAL_GetTick();
  uint8_t showing_results = 0U;

  oled_draw_default_layout();

  for(;;)
  {
    uint8_t show_results = g_show_results;
    if (show_results) {
      if (!showing_results) {
        oled_show_avg_screen(g_left_avg_rate, g_right_avg_rate);
        showing_results = 1U;
      }
      osDelay(50);
      continue;
    } else if (showing_results) {
      showing_results = 0U;
      oled_draw_default_layout();
      last_tick = HAL_GetTick();
    }

    uint32_t now = HAL_GetTick();
    if (now - last_tick >= 200) { // Update at 5 Hz
      last_tick = now;

      // --- SENSOR LOGIC ---
      ultrasonic_trigger();
      osDelay(50); // Wait for measurement
      float distance = ultrasonic_get_distance_cm();
      // --- END SENSOR LOGIC ---

      char buf[16];

      int dist10 = (int)(distance * 10.0f + (distance >= 0.0f ? 0.5f : -0.5f));
      if (dist10 < 0) dist10 = 0;
      int dist_int = dist10 / 10;
      int dist_tenth = dist10 % 10;
      (void)snprintf(buf, sizeof(buf), "%3d.%1d", dist_int, dist_tenth);
      OLED_ShowString(72, 0, (uint8_t*)"       ");
      OLED_ShowString(72, 0, (uint8_t*)buf);

      uint8_t rx_valid = g_uart_char_valid;
      uint8_t rx_char = g_uart_last_char;
      if (rx_valid) {
        OLED_ShowString(72, 24, (uint8_t*)"   ");
        OLED_ShowChar(72, 24, (char)rx_char, 12, 1);
      } else {
        OLED_ShowString(72, 24, (uint8_t*)" - ");
      }

      char cmd_char = g_current_instr;
      if (cmd_char) {
        OLED_ShowString(48, 48, (uint8_t*)"   ");
        OLED_ShowChar(48, 48, cmd_char, 12, 1);
      } else {
        OLED_ShowString(48, 48, (uint8_t*)" - ");
      }

      /*
      // Targets: clear small areas first to avoid ghosting, then draw (show absolute ticks)
      OLED_ShowString(16, 32, (uint8_t*)"    ");
      OLED_ShowString(56, 32, (uint8_t*)"    ");
      OLED_ShowNumber(16, 32, (uint32_t)(tL < 0 ? -tL : tL), 4, 12);
      OLED_ShowNumber(56, 32, (uint32_t)(tR < 0 ? -tR : tR), 4, 12);

      // Measured: clear small areas first to avoid ghosting, then draw (show absolute ticks)
      OLED_ShowString(16, 48, (uint8_t*)"    ");
      OLED_ShowString(56, 48, (uint8_t*)"    ");
      OLED_ShowNumber(16, 48, (uint32_t)(mL < 0 ? -mL : mL), 4, 12);
      OLED_ShowNumber(56, 48, (uint32_t)(mR < 0 ? -mR : mR), 4, 12);
      */

      // Refresh
      OLED_Refresh_Gram();
    }
    osDelay(5);
  }
  /* USER CODE END oled_task */
}

/* USER CODE BEGIN Header_uart_task */
/**
* @brief Function implementing the UART_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart_task */
void uart_task(void *argument)
{
  /* USER CODE BEGIN uart_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END uart_task */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
