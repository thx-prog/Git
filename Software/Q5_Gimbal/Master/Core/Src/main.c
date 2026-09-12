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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MPU6050_ADDR            (0x68 << 1)
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B

#define CONTROL_DT              0.02f
#define RAD_TO_DEG              57.2957795f
#define COMP_ALPHA              0.98f

#define MODE_POT                0
#define MODE_IMU                1

#define SERVO_CENTER_DEG        90.0f
#define SERVO_MIN_DEG           0.0f
#define SERVO_MAX_DEG           180.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
volatile uint16_t adc_value[2] = {0, 0};

volatile uint16_t pot_yaw = 0;
volatile uint16_t pot_pitch = 0;

volatile uint8_t mpu_whoami = 0;
volatile HAL_StatusTypeDef mpu_status;
volatile uint8_t mpu_init_ok = 0;

volatile int16_t accel_x = 0;
volatile int16_t accel_y = 0;
volatile int16_t accel_z = 0;
volatile int16_t temperature_raw = 0;
volatile int16_t gyro_x = 0;
volatile int16_t gyro_y = 0;
volatile int16_t gyro_z = 0;

volatile float accel_x_g = 0.0f;
volatile float accel_y_g = 0.0f;
volatile float accel_z_g = 0.0f;

volatile float gyro_x_dps = 0.0f;
volatile float gyro_y_dps = 0.0f;
volatile float gyro_z_dps = 0.0f;

volatile float gyro_bias_x = 0.0f;
volatile float gyro_bias_y = 0.0f;
volatile float gyro_bias_z = 0.0f;

volatile float accel_roll_deg = 0.0f;
volatile float accel_pitch_deg = 0.0f;

volatile float roll_deg = 0.0f;
volatile float pitch_deg = 0.0f;
volatile float yaw_deg = 0.0f;

volatile uint8_t control_mode = MODE_POT;

volatile float target_yaw_deg = 90.0f;
volatile float target_pitch_deg = 90.0f;

volatile float imu_yaw_zero = 0.0f;
volatile float imu_pitch_zero = 0.0f;

volatile uint8_t wireless_tx_divider = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef MPU6050_Init(void);
static HAL_StatusTypeDef MPU6050_ReadRaw(void);
static void MPU6050_CalibrateGyro(void);
static void Attitude_Init(void);
static void Attitude_Update(void);
static void Mode_Update(void);
static float ClampFloat(float value, float min_value, float max_value);
static void Wireless_SendTargets(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static HAL_StatusTypeDef MPU6050_Init(void)
{
  uint8_t data;

  if (HAL_I2C_Mem_Read(&hi2c1,
                       MPU6050_ADDR,
                       0x75,
                       I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&mpu_whoami,
                       1,
                       100) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (mpu_whoami != 0x68)
  {
    return HAL_ERROR;
  }

  data = 0x00;
  if (HAL_I2C_Mem_Write(&hi2c1,
                        MPU6050_ADDR,
                        MPU6050_PWR_MGMT_1,
                        I2C_MEMADD_SIZE_8BIT,
                        &data,
                        1,
                        100) != HAL_OK)
  {
    return HAL_ERROR;
  }

  osDelay(100);

  data = 0x00;
  if (HAL_I2C_Mem_Write(&hi2c1,
                        MPU6050_ADDR,
                        MPU6050_GYRO_CONFIG,
                        I2C_MEMADD_SIZE_8BIT,
                        &data,
                        1,
                        100) != HAL_OK)
  {
    return HAL_ERROR;
  }

  data = 0x00;
  if (HAL_I2C_Mem_Write(&hi2c1,
                        MPU6050_ADDR,
                        MPU6050_ACCEL_CONFIG,
                        I2C_MEMADD_SIZE_8BIT,
                        &data,
                        1,
                        100) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef MPU6050_ReadRaw(void)
{
  uint8_t buf[14];

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1,
                                               MPU6050_ADDR,
                                               MPU6050_ACCEL_XOUT_H,
                                               I2C_MEMADD_SIZE_8BIT,
                                               buf,
                                               14,
                                               100);

  if (status != HAL_OK)
  {
    return status;
  }

  accel_x = (int16_t)((buf[0] << 8) | buf[1]);
  accel_y = (int16_t)((buf[2] << 8) | buf[3]);
  accel_z = (int16_t)((buf[4] << 8) | buf[5]);

  temperature_raw = (int16_t)((buf[6] << 8) | buf[7]);

  gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
  gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
  gyro_z = (int16_t)((buf[12] << 8) | buf[13]);

  accel_x_g = (float)accel_x / 16384.0f;
  accel_y_g = (float)accel_y / 16384.0f;
  accel_z_g = (float)accel_z / 16384.0f;

  gyro_x_dps = (float)gyro_x / 131.0f;
  gyro_y_dps = (float)gyro_y / 131.0f;
  gyro_z_dps = (float)gyro_z / 131.0f;

  return HAL_OK;
}

static void MPU6050_CalibrateGyro(void)
{
  const uint16_t samples = 500;
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_z = 0.0f;
  uint16_t valid_samples = 0;

  for (uint16_t i = 0; i < samples; i++)
  {
    if (MPU6050_ReadRaw() == HAL_OK)
    {
      sum_x += gyro_x_dps;
      sum_y += gyro_y_dps;
      sum_z += gyro_z_dps;
      valid_samples++;
    }

    osDelay(2);
  }

  if (valid_samples > 0)
  {
    gyro_bias_x = sum_x / (float)valid_samples;
    gyro_bias_y = sum_y / (float)valid_samples;
    gyro_bias_z = sum_z / (float)valid_samples;
  }
}

static void Attitude_Init(void)
{
  if (MPU6050_ReadRaw() != HAL_OK)
  {
    return;
  }

  accel_roll_deg = atan2f(accel_y_g, accel_z_g) * RAD_TO_DEG;

  accel_pitch_deg = atan2f(-accel_x_g,
                          sqrtf(accel_y_g * accel_y_g +
                                accel_z_g * accel_z_g)) * RAD_TO_DEG;

  roll_deg = accel_roll_deg;
  pitch_deg = accel_pitch_deg;
  yaw_deg = 0.0f;

  imu_yaw_zero = yaw_deg;
  imu_pitch_zero = pitch_deg;
}

static void Attitude_Update(void)
{
  static uint32_t last_sample_tick = 0;
  static uint8_t have_last_sample = 0;

  float gx;
  float gy;
  float gz;
  float dt = CONTROL_DT;

  if (MPU6050_ReadRaw() != HAL_OK)
  {
    return;
  }

  uint32_t now = HAL_GetTick();

  if (have_last_sample)
  {
    uint32_t elapsed_ms = now - last_sample_tick;
    last_sample_tick = now;

    if (elapsed_ms == 0U)
    {
      return;
    }

    if (elapsed_ms > 100U)
    {
      return;
    }

    dt = (float)elapsed_ms * 0.001f;
  }
  else
  {
    last_sample_tick = now;
    have_last_sample = 1U;
  }

  gx = gyro_x_dps - gyro_bias_x;
  gy = gyro_y_dps - gyro_bias_y;
  gz = gyro_z_dps - gyro_bias_z;

  accel_roll_deg = atan2f(accel_y_g, accel_z_g) * RAD_TO_DEG;

  accel_pitch_deg = atan2f(-accel_x_g,
                          sqrtf(accel_y_g * accel_y_g +
                                accel_z_g * accel_z_g)) * RAD_TO_DEG;

  roll_deg =
      COMP_ALPHA * (roll_deg + gx * dt)
      + (1.0f - COMP_ALPHA) * accel_roll_deg;

  pitch_deg =
      COMP_ALPHA * (pitch_deg + gy * dt)
      + (1.0f - COMP_ALPHA) * accel_pitch_deg;

  yaw_deg += gz * dt;
}

static void Mode_Update(void)
{
  static GPIO_PinState last_key = GPIO_PIN_SET;
  static uint32_t last_switch_tick = 0;

  GPIO_PinState key = HAL_GPIO_ReadPin(MODE_KEY_GPIO_Port, MODE_KEY_Pin);

  if ((key == GPIO_PIN_RESET) &&
      (last_key == GPIO_PIN_SET) &&
      ((HAL_GetTick() - last_switch_tick) > 200))
  {
    last_switch_tick = HAL_GetTick();

    if (control_mode == MODE_POT)
    {
      control_mode = MODE_IMU;
      imu_yaw_zero = yaw_deg;
      imu_pitch_zero = pitch_deg;
    }
    else
    {
      control_mode = MODE_POT;
    }
  }

  last_key = key;

  if (control_mode == MODE_POT)
  {
    target_yaw_deg =
        ((float)pot_yaw * 180.0f) / 4095.0f;

    target_pitch_deg =
        ((float)pot_pitch * 180.0f) / 4095.0f;
  }
  else
  {
    float relative_yaw = yaw_deg - imu_yaw_zero;
    float relative_pitch = pitch_deg - imu_pitch_zero;

    target_yaw_deg =
        ClampFloat(SERVO_CENTER_DEG + relative_yaw,
                   SERVO_MIN_DEG,
                   SERVO_MAX_DEG);

    target_pitch_deg =
        ClampFloat(SERVO_CENTER_DEG + relative_pitch,
                   SERVO_MIN_DEG,
                   SERVO_MAX_DEG);
  }
}


static void Wireless_SendTargets(void)
{
  uint8_t frame[5];

  float yaw = ClampFloat(target_yaw_deg, 0.0f, 180.0f);
  float pitch = ClampFloat(target_pitch_deg, 0.0f, 180.0f);

  frame[0] = 0xAA;
  frame[1] = (uint8_t)(yaw + 0.5f);
  frame[2] = (uint8_t)(pitch + 0.5f);
  frame[3] = control_mode;
  frame[4] = frame[0] ^ frame[1] ^ frame[2] ^ frame[3];

  HAL_UART_Transmit(&huart1, frame, sizeof(frame), 20);
}

/* USER CODE END 0 */


int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */


  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  osKernelStart();


  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}


static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}


static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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


static void MX_DMA_Init(void)
{

  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = MODE_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MODE_KEY_GPIO_Port, &GPIO_InitStruct);

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

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, 2) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT | DMA_IT_TC);

  mpu_status = MPU6050_Init();
  mpu_init_ok = (mpu_status == HAL_OK) ? 1 : 0;

  if (mpu_init_ok)
  {
    MPU6050_CalibrateGyro();
    Attitude_Init();
  }

  for (;;)
  {
    pot_yaw = adc_value[0];
    pot_pitch = adc_value[1];

    if (mpu_init_ok)
    {
      Attitude_Update();
    }

    Mode_Update();

    wireless_tx_divider++;
    if (wireless_tx_divider >= 3)
    {
      wireless_tx_divider = 0;
      Wireless_SendTargets();
    }

    osDelay(20);
  }

  /* USER CODE END 5 */
}


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


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
