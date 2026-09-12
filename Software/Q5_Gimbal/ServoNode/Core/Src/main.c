/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#define SERVO_MIN_US        500U
#define SERVO_MAX_US        2500U
#define SERVO_CENTER_US     1500U
#define RX_TIMEOUT_MS       500U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
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
volatile uint8_t rx_yaw_deg = 90;
volatile uint8_t rx_pitch_deg = 90;
volatile uint8_t rx_mode = 0;
volatile uint8_t rx_ok = 0;
volatile uint32_t rx_good_frames = 0;
volatile uint32_t rx_bad_frames = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static uint16_t Servo_AngleToPulse(uint8_t angle);
static void Servo_SetAngles(uint8_t yaw, uint8_t pitch);
static uint8_t Wireless_ReceiveFrame(uint8_t *yaw, uint8_t *pitch, uint8_t *mode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint16_t Servo_AngleToPulse(uint8_t angle)
{
  if (angle > 180U)
  {
    angle = 180U;
  }

  return (uint16_t)(SERVO_MIN_US +
                    ((uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180U);
}

static void Servo_SetAngles(uint8_t yaw, uint8_t pitch)
{
  __HAL_TIM_SET_COMPARE(&htim3,
                        TIM_CHANNEL_1,
                        Servo_AngleToPulse(yaw));

  __HAL_TIM_SET_COMPARE(&htim3,
                        TIM_CHANNEL_2,
                        Servo_AngleToPulse(pitch));
}

static uint8_t Wireless_ReceiveFrame(uint8_t *yaw,
                                     uint8_t *pitch,
                                     uint8_t *mode)
{
  uint8_t header;
  uint8_t frame[4];

  if (HAL_UART_Receive(&huart1, &header, 1, 20) != HAL_OK)
  {
    return 0;
  }

  if (header != 0xAA)
  {
    return 0;
  }

  if (HAL_UART_Receive(&huart1, frame, 4, 20) != HAL_OK)
  {
    rx_bad_frames++;
    return 0;
  }

  uint8_t checksum = (uint8_t)(0xAA ^ frame[0] ^ frame[1] ^ frame[2]);

  if (checksum != frame[3])
  {
    rx_bad_frames++;
    return 0;
  }

  if ((frame[0] > 180U) ||
      (frame[1] > 180U) ||
      (frame[2] > 1U))
  {
    rx_bad_frames++;
    return 0;
  }

  *yaw = frame[0];
  *pitch = frame[1];
  *mode = frame[2];

  return 1;
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  osKernelInitialize();

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  osKernelStart();

  while (1)
  {
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
}

static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

static void MX_USART1_UART_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  Servo_SetAngles(90, 90);

  uint32_t last_valid_rx = HAL_GetTick();

  for (;;)
  {
    uint8_t yaw;
    uint8_t pitch;
    uint8_t mode;

    if (Wireless_ReceiveFrame(&yaw, &pitch, &mode))
    {
      rx_yaw_deg = yaw;
      rx_pitch_deg = pitch;
      rx_mode = mode;
      rx_ok = 1;
      rx_good_frames++;

      last_valid_rx = HAL_GetTick();

      Servo_SetAngles(rx_yaw_deg, rx_pitch_deg);
    }

    if ((HAL_GetTick() - last_valid_rx) > RX_TIMEOUT_MS)
    {
      rx_ok = 0;
    }

    osDelay(1);
  }

  /* USER CODE END 5 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
