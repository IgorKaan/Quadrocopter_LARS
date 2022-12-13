/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "MadgwickFilter/MadgwickAHRS.h"
#include "madgwickFilter.h"
#include "MPU9250.h"
#include "math.h"
#include <string.h>
#include "can.h"
#include "usart.h"
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
/* USER CODE BEGIN Variables */
extern CAN_TxHeaderTypeDef TxHeaderRoll;
extern CAN_TxHeaderTypeDef TxHeaderPitch;
extern CAN_TxHeaderTypeDef TxHeaderYaw;
extern CAN_TxHeaderTypeDef TxHeaderAccel;
extern CAN_TxHeaderTypeDef TxHeaderGyro;
extern uint32_t TxMailbox;
volatile float mass[1000];
uint8_t can_data[8] = {0,};
char buffer[10];
float gyroX;
float gyroY;
float gyroZ;
float accelX;
float accelY;
float accelZ;
float gyroX_filtered;
float gyroY_filtered;
float gyroZ_filtered;
float accelX_filtered;
float accelY_filtered;
float accelZ_filtered;
float accelX_summ;
float accelY_summ;
float accelZ_summ;
float accelX_average;
float accelY_average;
float accelZ_average;
float gyroX_summ;
float gyroY_summ;
float gyroZ_summ;
float gyroX_average;
float gyroY_average;
float gyroZ_average;

float roll = 0.0, pitch = 0.0, yaw = 0.0;

uint8_t acc_data[8] = {0,};

uint8_t gyro_data[8] = {0,};

extern uint8_t _buffer[21];

extern uint16_t ii, packet_count, fifo_count;

uint32_t can = 0;
//uint8_t buffer[12];

int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
int32_t accel_bias_reg[3] = {0, 0, 0};

uint32_t count = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MPUTask */
osThreadId_t MPUTaskHandle;
const osThreadAttr_t MPUTask_attributes = {
  .name = "MPUTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MPU9250_GetData(int16_t* AccData, int16_t* MagData, int16_t* GyroData);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMPUTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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

  /* creation of MPUTask */
  MPUTaskHandle = osThreadNew(StartMPUTask, NULL, &MPUTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMPUTask */
/**
* @brief Function implementing the MPUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMPUTask */
void StartMPUTask(void *argument)
{
  /* USER CODE BEGIN StartMPUTask */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 5;
  xLastWakeTime = xTaskGetTickCount();
  int i = 0;
  /* Infinite loop */
  for(;;)
  {
	int16_t AccData[3], GyroData[3], MagData[3];
	MPU9250_GetData(AccData, GyroData, MagData);
	accelX_average = accelX_filtered;
	accelY_average = accelY_filtered;
	accelZ_average = accelZ_filtered;
	gyroX_average = gyroX_filtered;
	gyroY_average = gyroY_filtered;
	gyroZ_average = gyroZ_filtered;
	imu_filter(accelX_average, accelY_average, accelZ_average, gyroX_average, gyroY_average, 0);
	yaw = 0;
	eulerAngles(q_est, &roll, &pitch, &yaw);
//	mass[i] = accelY;
//	i++;
//	if (i == 999) {
//		i = 0;
//	}
//	sprintf(buffer, "%0.5f", roll);
//	buffer[8] = '\r';
//	buffer[9] = '\n';
//	HAL_UART_Transmit(&huart2, buffer, 10, HAL_MAX_DELAY);
//	//osDelay(1);
//	for (int i = 0; i < 10; ++i) {
//		buffer[i] = 0;
//	}
	memcpy(can_data, &accelY_average, 4);
	memcpy(&can_data[4], &accelX_average, 4);
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderRoll, can_data, &TxMailbox) == HAL_OK) {
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	}
	osDelay(1);
	memcpy(can_data, &accelZ_average, 4);
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderYaw, can_data, &TxMailbox) == HAL_OK) {
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	}

//	memcpy(can_data, &roll, 4);
//	memcpy(&can_data[4], &pitch, 4);
//	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderRoll, can_data, &TxMailbox) == HAL_OK) {
//		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//	}
//	osDelay(1);
//	memcpy(can_data, &yaw, 4);
//	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderYaw, can_data, &TxMailbox) == HAL_OK) {
//		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//	}
	count = HAL_GetTick();
	vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartMPUTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

