/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_CPR        2496.0f
#define CONTROL_DT         0.02f

#define MODE_SPEED         0
#define MODE_POSITION      1

#define SPEED_RPM_MAX      50.0f
#define SPEED_KP           131.0f
#define SPEED_KI           124.0f
#define SPEED_KD           0.06f
#define SPEED_PWM_MAX      3599.0f
#define SPEED_START_PWM    900.0f

#define POSITION_MAX_DEG   180.0f
#define ENCODER_DIR        1
#define POS_KP             12.0f
#define POS_KI             0.5f
#define POS_KD             0.22f
#define POS_PWM_MAX        2500.0f
#define POS_PWM_MIN        2100.0f
#define POS_DEADBAND       2.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t control_mode = MODE_SPEED;

uint16_t adc_value = 0;
uint16_t encoder_last = 0;
int32_t encoder_total = 0;

float target_rpm = 0.0f;
float actual_rpm = 0.0f;

float target_angle = 0.0f;
float actual_angle = 0.0f;

float motor_output = 0.0f;

float speed_integral = 0.0f;
float speed_last_error = 0.0f;

float pos_integral = 0.0f;
float pos_last_error = 0.0f;

uint8_t vofa_divider = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void Motor_SetPWM(uint16_t pwm);
static void Motor_Forward(void);
static void Motor_Reverse(void);
static void Motor_Stop(void);
static void Motor_SetSignedOutput(float output);

static uint16_t ADC_Read(void);
static void Encoder_UpdateSpeed(void);
static void Encoder_UpdatePosition(void);

static void Speed_PID_Reset(void);
static float Speed_PID_Calc(float target, float actual);

static void Position_PID_Reset(void);
static float Position_PID_Calc(float target, float actual);

static void Set_Mode(uint8_t mode);
static void Position_Zero(void);
static void UART_CheckCommand(void);
static void VOFA_Send(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Motor_SetPWM(uint16_t pwm)
{
  if (pwm > 3599)
  {
    pwm = 3599;
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
}

static void Motor_Forward(void)
{
  HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
}

static void Motor_Reverse(void)
{
  HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
}

static void Motor_Stop(void)
{
  Motor_SetPWM(0);
  HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
  motor_output = 0.0f;
}

static void Motor_SetSignedOutput(float output)
{
  float pwm = output;

  if (pwm > POS_PWM_MAX) pwm = POS_PWM_MAX;
  if (pwm < -POS_PWM_MAX) pwm = -POS_PWM_MAX;

  if (pwm > 0.0f)
  {
    Motor_Forward();
    Motor_SetPWM((uint16_t)pwm);
  }
  else if (pwm < 0.0f)
  {
    Motor_Reverse();
    Motor_SetPWM((uint16_t)(-pwm));
  }
  else
  {
    Motor_Stop();
  }

  motor_output = pwm;
}

static uint16_t ADC_Read(void)
{
  uint16_t value = 0;

  HAL_ADC_Start(&hadc1);

  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  }

  HAL_ADC_Stop(&hadc1);
  return value;
}

static void Encoder_UpdateSpeed(void)
{
  uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  int16_t delta = (int16_t)(now - encoder_last);
  int32_t count = delta;

  encoder_last = now;

  if (count < 0)
  {
    count = -count;
  }

  float rpm = ((float)count * 60.0f) / (ENCODER_CPR * CONTROL_DT);
  actual_rpm = actual_rpm * 0.7f + rpm * 0.3f;
}

static void Encoder_UpdatePosition(void)
{
  uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  int16_t delta = (int16_t)(now - encoder_last);

  encoder_last = now;
  encoder_total += (int32_t)delta * ENCODER_DIR;

  actual_angle = ((float)encoder_total * 360.0f) / ENCODER_CPR;
}

static void Speed_PID_Reset(void)
{
  speed_integral = 0.0f;
  speed_last_error = 0.0f;
}

static float Speed_PID_Calc(float target, float actual)
{
  float error = target - actual;

  speed_integral += error * CONTROL_DT;

  if (speed_integral > 100.0f) speed_integral = 100.0f;
  if (speed_integral < -100.0f) speed_integral = -100.0f;

  float derivative = (error - speed_last_error) / CONTROL_DT;
  speed_last_error = error;

  float output = SPEED_KP * error
               + SPEED_KI * speed_integral
               + SPEED_KD * derivative;

  if (output > SPEED_PWM_MAX) output = SPEED_PWM_MAX;
  if (output < 0.0f) output = 0.0f;

  return output;
}

static void Position_PID_Reset(void)
{
  pos_integral = 0.0f;
  pos_last_error = 0.0f;
}

static float Position_PID_Calc(float target, float actual)
{
  float error = target - actual;

  pos_integral += error * CONTROL_DT;

  if (pos_integral > 300.0f) pos_integral = 300.0f;
  if (pos_integral < -300.0f) pos_integral = -300.0f;

  float derivative = (error - pos_last_error) / CONTROL_DT;
  pos_last_error = error;

  float output = POS_KP * error
               + POS_KI * pos_integral
               + POS_KD * derivative;

  if (output > POS_PWM_MAX) output = POS_PWM_MAX;
  if (output < -POS_PWM_MAX) output = -POS_PWM_MAX;

  return output;
}

static void Position_Zero(void)
{
  Motor_Stop();
  __HAL_TIM_SET_COUNTER(&htim2, 0);

  encoder_last = 0;
  encoder_total = 0;
  actual_angle = 0.0f;

  Position_PID_Reset();
}

static void Set_Mode(uint8_t mode)
{
  Motor_Stop();

  control_mode = mode;
  encoder_last = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

  Speed_PID_Reset();
  Position_PID_Reset();

  if (control_mode == MODE_SPEED)
  {
    actual_rpm = 0.0f;
  }
  else
  {
    Position_Zero();
  }
}

static void UART_CheckCommand(void)
{
  uint8_t cmd;

  if (HAL_UART_Receive(&huart1, &cmd, 1, 0) == HAL_OK)
  {
    if ((cmd == 'S') || (cmd == 's'))
    {
      Set_Mode(MODE_SPEED);
    }
    else if ((cmd == 'P') || (cmd == 'p'))
    {
      Set_Mode(MODE_POSITION);
    }
    else if ((cmd == 'Z') || (cmd == 'z'))
    {
      if (control_mode == MODE_POSITION)
      {
        Position_Zero();
      }
    }
  }
}

static void VOFA_Send(void)
{
  float data[3];

  if (control_mode == MODE_SPEED)
  {
    data[0] = target_rpm;
    data[1] = actual_rpm;
    data[2] = motor_output / SPEED_PWM_MAX * 100.0f;
  }
  else
  {
    data[0] = target_angle;
    data[1] = actual_angle;
    data[2] = motor_output / POS_PWM_MAX * 100.0f;
  }

  uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

  HAL_UART_Transmit(&huart1, (uint8_t *)data, sizeof(data), 100);
  HAL_UART_Transmit(&huart1, tail, sizeof(tail), 100);
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_Pin|MOTOR_IN2_Pin|MOTOR_STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR_IN1_Pin MOTOR_IN2_Pin MOTOR_STBY_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN1_Pin|MOTOR_IN2_Pin|MOTOR_STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  encoder_last = 0;
  encoder_total = 0;

  Motor_Stop();
  Speed_PID_Reset();
  Position_PID_Reset();

  for (;;)
  {
    UART_CheckCommand();
    adc_value = ADC_Read();

    if (control_mode == MODE_SPEED)
    {
      Encoder_UpdateSpeed();

      target_rpm = ((float)adc_value * SPEED_RPM_MAX) / 4095.0f;

      if (target_rpm < 3.0f)
      {
        Motor_Stop();
        Speed_PID_Reset();
      }
      else
      {
        motor_output = Speed_PID_Calc(target_rpm, actual_rpm);

        if ((actual_rpm < 2.0f) && (motor_output < SPEED_START_PWM))
        {
          motor_output = SPEED_START_PWM;
        }

        Motor_Forward();
        Motor_SetPWM((uint16_t)motor_output);
      }
    }
    else
    {
      Encoder_UpdatePosition();

      target_angle = ((float)adc_value * POSITION_MAX_DEG) / 4095.0f;

      float error = target_angle - actual_angle;
      float abs_error = error;

      if (abs_error < 0.0f)
      {
        abs_error = -abs_error;
      }

      if (abs_error <= POS_DEADBAND)
      {
        Motor_Stop();
        Position_PID_Reset();
      }
      else
      {
        float output = Position_PID_Calc(target_angle, actual_angle);

        if ((output > 0.0f) && (output < POS_PWM_MIN))
        {
          output = POS_PWM_MIN;
        }
        else if ((output < 0.0f) && (output > -POS_PWM_MIN))
        {
          output = -POS_PWM_MIN;
        }

        Motor_SetSignedOutput(output);
      }
    }

    vofa_divider++;

    if (vofa_divider >= 5)
    {
      vofa_divider = 0;
      VOFA_Send();
    }

    osDelay(20);
  }

  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
