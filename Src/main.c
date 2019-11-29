/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pca9685.h"
#include "utils.h"
#include "sbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

osThreadId_t defaultTaskHandle;
osThreadId_t flashBlueHandle;
osThreadId_t parseSbusHandle;
osThreadId_t driveServoHandle;
osThreadId_t driveEscHandle;
osMutexId_t i2cMutexHandle;
osSemaphoreId_t SBusFrameHandle;
osSemaphoreId_t SBusPacketHandle;
/* USER CODE BEGIN PV */
osEventFlagsId_t SBusPacketEvtHandle;
static char buff[40 * 5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartFlashBlue(void *argument);
void StartParseSbus(void *argument);
void StartDriveServo(void *argument);
void StartDriveEsc(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    sprintf(buff, "Hi there \r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)buff, strlen(buff), 5000);
  /* USER CODE END 2 */

  osKernelInitialize();

  /* Create the mutex(es) */
  /* definition and creation of i2cMutex */
  const osMutexAttr_t i2cMutex_attributes = {
    .name = "i2cMutex"
  };
  i2cMutexHandle = osMutexNew(&i2cMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  SBusPacketEvtHandle = osEventFlagsNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of SBusFrame */
  const osSemaphoreAttr_t SBusFrame_attributes = {
    .name = "SBusFrame"
  };
  SBusFrameHandle = osSemaphoreNew(1, 1, &SBusFrame_attributes);

  /* definition and creation of SBusPacket */
  const osSemaphoreAttr_t SBusPacket_attributes = {
    .name = "SBusPacket"
  };
  SBusPacketHandle = osSemaphoreNew(1, 1, &SBusPacket_attributes);

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
  /* definition and creation of defaultTask */
  const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 256
  };
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* definition and creation of flashBlue */
  const osThreadAttr_t flashBlue_attributes = {
    .name = "flashBlue",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 256
  };
  flashBlueHandle = osThreadNew(StartFlashBlue, NULL, &flashBlue_attributes);

  /* definition and creation of parseSbus */
  const osThreadAttr_t parseSbus_attributes = {
    .name = "parseSbus",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 512
  };
  parseSbusHandle = osThreadNew(StartParseSbus, NULL, &parseSbus_attributes);

  /* definition and creation of driveServo */
  const osThreadAttr_t driveServo_attributes = {
    .name = "driveServo",
    .priority = (osPriority_t) osPriorityBelowNormal,
    .stack_size = 1024
  };
  driveServoHandle = osThreadNew(StartDriveServo, NULL, &driveServo_attributes);

  /* definition and creation of driveEsc */
  const osThreadAttr_t driveEsc_attributes = {
    .name = "driveEsc",
    .priority = (osPriority_t) osPriorityBelowNormal,
    .stack_size = 512
  };
  driveEscHandle = osThreadNew(StartDriveEsc, NULL, &driveEsc_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
#pragma clang diagnostic pop
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 100000;
  huart4.Init.WordLength = UART_WORDLENGTH_9B;
  huart4.Init.StopBits = UART_STOPBITS_2;
  huart4.Init.Parity = UART_PARITY_EVEN;
  huart4.Init.Mode = UART_MODE_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Green_LED_Pin|Red_LED_Pin|Blue_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TST_SIG_GPIO_Port, TST_SIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Green_LED_Pin Red_LED_Pin Blue_LED_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin|Red_LED_Pin|Blue_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TST_SIG_Pin */
  GPIO_InitStruct.Pin = TST_SIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TST_SIG_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t something = 0U;
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
    something = 1U;
    something = 2U;
}
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
  /* Infinite loop */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
  for(;;)
  {
    osDelay(1);
  }
#pragma clang diagnostic pop
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartFlashBlue */
/**
* @brief Function implementing the flashBlue thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFlashBlue */
void StartFlashBlue(void *argument)
{
  /* USER CODE BEGIN StartFlashBlue */
  /* Infinite loop */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
  for(;;)
  {
    HAL_GPIO_TogglePin(Blue_LED_GPIO_Port, Blue_LED_Pin);
    osDelay(100 / (portTICK_RATE_MS));
  }
#pragma clang diagnostic pop
  /* USER CODE END StartFlashBlue */
}

/* USER CODE BEGIN Header_StartParseSbus */
/**
* @brief Function implementing the parseSbus thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartParseSbus */
void StartParseSbus(void *argument)
{
  /* USER CODE BEGIN StartParseSbus */
  PCA9685_Reset(&hi2c1);
  PCA9685_SetPWMFreq(&hi2c1, 60.0f);
  osSemaphoreAcquire(SBusFrameHandle, 0U);

  /* Infinite loop */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
  for(;;)
  {
    HAL_GPIO_WritePin(TST_SIG_GPIO_Port, TST_SIG_Pin, GPIO_PIN_RESET);
    osSemaphoreAcquire(SBusFrameHandle, osWaitForever);
    HAL_GPIO_WritePin(TST_SIG_GPIO_Port, TST_SIG_Pin, GPIO_PIN_SET);

    SBus_DecodeFrame();
//    osSemaphoreRelease(SBusPacketHandle);
    osEventFlagsSet(SBusPacketEvtHandle, 0x00000001U);

  }
#pragma clang diagnostic pop
  /* USER CODE END StartParseSbus */
}

/* USER CODE BEGIN Header_StartDriveServo */
/**
* @brief Function implementing the driveServo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDriveServo */
void StartDriveServo(void *argument)
{
  /* USER CODE BEGIN StartDriveServo */
  /* Infinite loop */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
  for(;;)
  {
//    osSemaphoreAcquire(SBusPacketHandle, osWaitForever);
//    HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
//    osEventFlagsWait(SBusPacketEvtHandle, 0x00000001U, osFlagsWaitAny, osWaitForever);
//    HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);
//    uint16_t val = SBus_GetChannel(1U);
//    val = sbusToPwm(val);
//    osMutexAcquire(i2cMutexHandle, osWaitForever);
//    PCA9685_SetPWM(&hi2c1, STEER_SERVO_PIN, 0, val);
//    osMutexRelease(i2cMutexHandle);

    sprintf(buff, "\r\nTask           \tAbs Time\t%% Time\r\n****************************************\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)buff, strlen(buff), 5000);
    vTaskGetRunTimeStats(buff);
//    sprintf(buff, "Hi there \r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)buff, strlen(buff), 5000);
    osDelay(5000 / (portTICK_RATE_MS));
  }
#pragma clang diagnostic pop
  /* USER CODE END StartDriveServo */
}

/* USER CODE BEGIN Header_StartDriveEsc */
/**
* @brief Function implementing the driveEsc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDriveEsc */
void StartDriveEsc(void *argument)
{
  /* USER CODE BEGIN StartDriveEsc */
  /* Infinite loop */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
  for(;;)
  {
//    HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);
    osEventFlagsWait(SBusPacketEvtHandle, 0x00000001U, osFlagsWaitAny, osWaitForever);
//    HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_SET);
    uint16_t val = SBus_GetChannel(0U);
    val = sbusToPwm(val);
//    osMutexAcquire(i2cMutexHandle, osWaitForever);
    PCA9685_SetPWM(&hi2c1, DRIVE_ESC_PIN, 0, val);
//    osMutexRelease(i2cMutexHandle);

      val = SBus_GetChannel(1U);
      val = sbusToPwm(val);
//    osMutexAcquire(i2cMutexHandle, osWaitForever);
      PCA9685_SetPWM(&hi2c1, STEER_SERVO_PIN, 0, val);
  }
#pragma clang diagnostic pop
  /* USER CODE END StartDriveEsc */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
