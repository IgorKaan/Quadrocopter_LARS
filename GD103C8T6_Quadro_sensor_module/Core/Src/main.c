/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "madgwickFilter.h"
#include "MPU9250.h"
#include <string.h>
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

/* USER CODE BEGIN PV */
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeaderRoll;
CAN_TxHeaderTypeDef TxHeaderPitch;
CAN_TxHeaderTypeDef TxHeaderYaw;
CAN_TxHeaderTypeDef TxHeaderAccel;
CAN_TxHeaderTypeDef TxHeaderGyro;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox = 0;
const uint32_t headerIdRoll = 0x11;
const uint32_t headerIdPitch = 0x12;
const uint32_t headerIdYaw = 0x13;
const uint32_t headerIdAccel = 0x14;
const uint32_t headerIdGyro = 0x15;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
uint8_t MPU9250_Init();
void MPU9250_calibrate();
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
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  for (int i = 0; i < 5; ++i) {
//	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	//HAL_Delay(500);
  }
  MPU9250_calibrate();
  HAL_Delay(2000);
  MPU9250_Init();
  HAL_Delay(500);

  TxHeaderRoll.StdId = headerIdRoll;
  TxHeaderRoll.ExtId = 0;
  TxHeaderRoll.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
  TxHeaderRoll.IDE = CAN_ID_STD;   // CAN_ID_EXT
  TxHeaderRoll.DLC = 4;
  TxHeaderRoll.TransmitGlobalTime = 0;

  TxHeaderPitch.StdId = headerIdPitch;
  TxHeaderPitch.ExtId = 0;
  TxHeaderPitch.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
  TxHeaderPitch.IDE = CAN_ID_STD;   // CAN_ID_EXT
  TxHeaderPitch.DLC = 4;
  TxHeaderPitch.TransmitGlobalTime = 0;

  TxHeaderYaw.StdId = headerIdYaw;
  TxHeaderYaw.ExtId = 0;
  TxHeaderYaw.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
  TxHeaderYaw.IDE = CAN_ID_STD;   // CAN_ID_EXT
  TxHeaderYaw.DLC = 4;
  TxHeaderYaw.TransmitGlobalTime = 0;

  TxHeaderAccel.StdId = headerIdAccel;
  TxHeaderAccel.ExtId = 0;
  TxHeaderAccel.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
  TxHeaderAccel.IDE = CAN_ID_STD;   // CAN_ID_EXT
  TxHeaderAccel.DLC = 6;
  TxHeaderAccel.TransmitGlobalTime = 0;

  TxHeaderGyro.StdId = headerIdGyro;
  TxHeaderGyro.ExtId = 0;
  TxHeaderGyro.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
  TxHeaderGyro.IDE = CAN_ID_STD;   // CAN_ID_EXT
  TxHeaderGyro.DLC = 6;
  TxHeaderGyro.TransmitGlobalTime = 0;

  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterIdHigh = 0;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  //sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
  HAL_CAN_Start(&hcan);
  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  if (htim->Instance == TIM4) {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
