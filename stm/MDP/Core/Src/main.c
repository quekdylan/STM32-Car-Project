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

// Current high-level instruction being executed (e.g., 'S','L','R','s','r').
// 0 means none; shown as '-' on OLED.
volatile char g_current_instr = 0;

// Latched when the user presses the start button to begin the run sequence.
volatile uint8_t g_user_start_requested = 0;

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

static char uart3_cmd_buffer[UART_CMD_BUF_SIZE];
static size_t uart3_cmd_length = 0;

// Sequence execution guard
typedef enum {
  CMD_TYPE_MOVE = 0,
  CMD_TYPE_TURN = 1
} command_type_t;

typedef struct {
  command_type_t type;
  int16_t value_cm;   // for moves
  char opcode;        // 'F','B','R','L','r','l','Q'
} command_msg_t;

osMessageQueueId_t commandQueueHandle;
const osMessageQueueAttr_t commandQueue_attributes = {
  .name = "commandQueue"
};

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
static void uart_process_command(const char *cmd);
static void execute_straight_move(float distance_cm, uint32_t brake_ms);
static void execute_turn_move(char turn_opcode, float angle_deg, uint32_t brake_ms);
static void configure_servo_for_motion(void);
static void wait_for_user_start_button(void);
static void perform_initial_calibration(void);
typedef struct {
  char opcode;
  int param;
} scripted_move_t;

#define MAX_SCRIPT_STEPS 16U
#define SCRIPT_DEFAULT_BRAKE_MS 50U

// Edit this string to change the scripted sequence executed after calibration.
static char g_instruction_plan[] = "[['L', 90],['R', 90]]";

static int opcode_is_combinable(char opcode);
static size_t coalesce_instruction_plan(scripted_move_t *steps, size_t count);
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
    char c = (char)uart3_rx_byte;

    if (c == '\r' || c == '\n') {
      if (uart3_cmd_length > 0U) {
        uart3_cmd_buffer[uart3_cmd_length] = '\0';
        uart_process_command(uart3_cmd_buffer);
        uart3_cmd_length = 0U;
      }
    } else {
      if (uart3_cmd_length < (UART_CMD_BUF_SIZE - 1U)) {
        uart3_cmd_buffer[uart3_cmd_length++] = c;
        if (isprint((unsigned char)c)) {
          g_uart_last_char = (uint8_t)c;
          g_uart_char_valid = 1U;
        }
      } else {
        uart3_cmd_length = 0U;
      }
    }

    HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    uart3_cmd_length = 0U;
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

static int opcode_is_combinable(char opcode)
{
  switch (opcode) {
    case 'S':
    case 's':
    case 'L':
    case 'R':
    case 'l':
    case 'r':
      return 1;
    default:
      return 0;
  }
}

static size_t coalesce_instruction_plan(scripted_move_t *steps, size_t count)
{
  if (!steps || count == 0U) {
    return 0U;
  }

  size_t write = 0U;
  for (size_t read = 0U; read < count; ++read) {
    if (write > 0U) {
      scripted_move_t *prev = &steps[write - 1U];
      scripted_move_t *curr = &steps[read];
      if (curr->opcode == prev->opcode && opcode_is_combinable(curr->opcode)) {
        long combined = (long)prev->param + (long)curr->param;
        if (combined > (long)INT_MAX) {
          combined = (long)INT_MAX;
        } else if (combined < (long)INT_MIN) {
          combined = (long)INT_MIN;
        }
        prev->param = (int)combined;
        continue;
      }
    }

    if (write != read) {
      steps[write] = steps[read];
    }
    write++;
  }

  return write;
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

static void wait_for_user_start_button(void)
{
  g_user_start_requested = 0;
  uint8_t prev_pressed = user_is_pressed();

  while (!g_user_start_requested) {
    uint8_t pressed = user_is_pressed();
    if (!prev_pressed && pressed) {
      g_user_start_requested = 1;
    }
    prev_pressed = pressed;
    osDelay(10);
  }
}

static void perform_initial_calibration(void)
{
  g_is_calibrating = 1;
  osDelay(1000);
  motor_stop();
  imu_calibrate_bias_(1000);
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
  step_count = coalesce_instruction_plan(steps, step_count);

  for (size_t i = 0; i < step_count; ++i) {
    char opcode = steps[i].opcode;
    int param = steps[i].param;

    if (opcode == 'S' || opcode == 's') {
      float distance_cm = (opcode == 's') ? -fabsf((float)param) : fabsf((float)param);
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

  g_current_instr = (distance_cm >= 0.0f) ? 'S' : 's';
  move_start_straight(distance_cm);
  while (move_is_active()) {
    osDelay(5);
  }
  g_current_instr = 0;
  control_set_target_ticks_per_dt(0, 0);
  motor_brake_ms(brake_ms);
  motor_stop();
}

static void execute_turn_move(char turn_opcode, float angle_deg, uint32_t brake_ms)
{
  g_current_instr = turn_opcode;
  move_turn(turn_opcode, angle_deg);
  while (move_is_active()) {
    osDelay(5);
  }
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
  commandQueueHandle = osMessageQueueNew(16, sizeof(command_msg_t), &commandQueue_attributes);
  if (commandQueueHandle == NULL) {
    Error_Handler();
  }
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

static void uart_process_command(const char *cmd)
{
  if (cmd == NULL) {
    return;
  }

  if (commandQueueHandle == NULL) {
    uart_send_response("err:init\r\n");
    return;
  }

  char norm[UART_CMD_BUF_SIZE];
  size_t norm_len = 0U;
  size_t raw_len = strlen(cmd);
  for (size_t i = 0U; i < raw_len && norm_len < (UART_CMD_BUF_SIZE - 1U); ++i) {
    unsigned char uc = (unsigned char)cmd[i];
    if (uc == '\r' || uc == '\n') {
      continue;
    }
    if (isspace(uc)) {
      continue;
    }
    norm[norm_len++] = (char)toupper(uc);
  }
  norm[norm_len] = '\0';

  if (norm_len == 0U) {
    uart_send_response("err:empty\r\n");
    return;
  }

  if (norm_len == 1U) {
    uart_send_response("err:cmd\r\n");
    return;
  }

  if (norm_len >= 2U) {
    char c0 = norm[0];
    char c1 = norm[1];
    if ((c0 == 'F' || c0 == 'B') && (c1 == 'R' || c1 == 'L')) {
      char turn_cmd;
      if (c0 == 'F') {
        turn_cmd = (c1 == 'R') ? 'R' : 'L';
      } else {
        turn_cmd = (c1 == 'R') ? 'r' : 'l';
      }

      command_msg_t msg = {
        .type = CMD_TYPE_TURN,
        .value_cm = 90,
        .opcode = turn_cmd
      };

      if (osMessageQueuePut(commandQueueHandle, &msg, 0U, 0U) != osOK) {
        uart_send_response("err:queue\r\n");
      } else {
        char resp[24];
        (void)snprintf(resp, sizeof(resp), "ok %c%c\r\n", c0, c1);
        uart_send_response(resp);
      }
      return;
    }
  }

  char opcode = norm[0];
  if (opcode != 'F' && opcode != 'B') {
    uart_send_response("err:cmd\r\n");
    return;
  }

  size_t pos = 1U;
  while (pos < norm_len && norm[pos] == opcode) {
    pos++;
  }

  if (pos >= norm_len) {
    uart_send_response("err:value\r\n");
    return;
  }

  char num_buf[12];
  size_t num_len = 0U;
  while (pos < norm_len) {
    char ch = norm[pos];
    if (!isdigit((unsigned char)ch)) {
      uart_send_response("err:value\r\n");
      return;
    }
    if (num_len >= sizeof(num_buf) - 1U) {
      uart_send_response("err:range\r\n");
      return;
    }
    num_buf[num_len++] = ch;
    pos++;
  }
  num_buf[num_len] = '\0';

  long dist = strtol(num_buf, NULL, 10);
  if (dist <= 0L || dist > 1000L) {
    uart_send_response("err:range\r\n");
    return;
  }

  command_msg_t msg = {
    .type = CMD_TYPE_MOVE,
    .value_cm = (int16_t)dist,
    .opcode = opcode
  };

  if (osMessageQueuePut(commandQueueHandle, &msg, 0U, 0U) != osOK) {
    uart_send_response("err:queue\r\n");
    return;
  }

  char resp[32];
  (void)snprintf(resp, sizeof(resp), "ok %c %ld\r\n", opcode, dist);
  uart_send_response(resp);
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
  wait_for_user_start_button();
  perform_initial_calibration();
  reset_motion_state();
  run_instruction_plan();
  reset_motion_state();

  for(;;) {
    command_msg_t msg;
    if (osMessageQueueGet(commandQueueHandle, &msg, NULL, osWaitForever) != osOK) {
      continue;
    }

    if (msg.type == CMD_TYPE_MOVE) {
      float dist_cm = (msg.opcode == 'B') ? -(float)msg.value_cm : (float)msg.value_cm;
      g_current_instr = (dist_cm >= 0.0f) ? 'S' : 's';
      char dbg[32];
      (void)snprintf(dbg, sizeof(dbg), "exec %c %d\r\n", msg.opcode, (int)msg.value_cm);
      uart_send_response(dbg);
      move_start_straight(dist_cm);
      while (move_is_active()) {
        osDelay(5);
      }
      control_set_target_ticks_per_dt(0, 0);
      motor_brake_ms(50);
      motor_stop();
      Servo_Center(&global_steer);
      char done_msg[32];
      char dir = (msg.opcode == 'B') ? 'B' : 'F';
      (void)snprintf(done_msg, sizeof(done_msg), "done %c %d\r\n", dir, (int)msg.value_cm);
      uart_send_response(done_msg);
    } else if (msg.type == CMD_TYPE_TURN) {
      g_current_instr = msg.opcode;
      char dbg[24];
      (void)snprintf(dbg, sizeof(dbg), "exec turn %c\r\n", msg.opcode);
      uart_send_response(dbg);
      move_turn(msg.opcode, 90.0f);
      while (move_is_active()) {
        osDelay(5);
      }
      control_set_target_ticks_per_dt(0, 0);
      motor_brake_ms(50);
      motor_stop();
      Servo_Center(&global_steer);
      char primary = (msg.opcode == 'r' || msg.opcode == 'l') ? 'B' : 'F';
      char secondary = (msg.opcode == 'r' || msg.opcode == 'R') ? 'R' : 'L';
      char done_msg[24];
      (void)snprintf(done_msg, sizeof(done_msg), "done %c%c\r\n", primary, secondary);
      uart_send_response(done_msg);
    }

    g_current_instr = 0;
    Servo_Center(&global_steer);
    motor_stop();
    osDelay(20);
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
