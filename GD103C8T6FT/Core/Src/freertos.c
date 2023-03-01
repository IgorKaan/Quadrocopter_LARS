/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "madgwickFilter.h"
#include "usart.h"
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
/* USER CODE BEGIN Variables */
volatile float mass[1000];
uint8_t can_data[8] = {0,};
uint8_t can_alt_data[8] = {0,};
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

float auxiliary_roll = 0.0, auxiliary_pitch = 0.0, auxiliary_yaw = 0.0;

uint8_t acc_data[8] = {0,};

uint8_t gyro_data[8] = {0,};

extern uint8_t _buffer[21];

extern uint16_t ii, packet_count, fifo_count;

uint32_t can = 0;

uint32_t gyroZ_drift = 0;

extern volatile float relativeAltitudeFiltered;

float data[6] = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };
uint8_t data_8t[24];
//uint8_t buffer[12];

int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
int32_t accel_bias_reg[3] = {0, 0, 0};
uint32_t rawTemperature = 0;
uint32_t count = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MPU9250_GetData(int16_t* AccData, int16_t* MagData, int16_t* GyroData);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10;
  xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	int16_t AccData[3], GyroData[3], MagData[3];
	MPU9250_GetData(AccData, GyroData, MagData);
//	accelX_average = accelX_filtered;
//	accelY_average = accelY_filtered;
//	accelZ_average = accelZ_filtered;
//	gyroX_average = gyroX_filtered;
//	gyroY_average = gyroY_filtered;
//	gyroZ_average = gyroZ_filtered;
	//imu_filter_rp(-accelX_average, -accelY_average, -accelZ_average, gyroX_average, gyroY_average, 0);

	//imu_filter_y(accelX_average, accelY_average, accelZ_average, gyroX_average, gyroY_average, gyroZ_average);
	//yaw = 0;
	//q_est_rp.q4 = 0;
	//eulerAngles(q_est_rp, &roll, &pitch, &yaw);
	//eulerAngles(q_est_y, &auxiliary_roll, &auxiliary_pitch, &auxiliary_yaw);
	memcpy(&data_8t, &accelX, sizeof(float));
	memcpy(&data_8t[4], &accelY, sizeof(float));
	memcpy(&data_8t[8], &accelZ, sizeof(float));
	memcpy(&data_8t[12], &gyroX, sizeof(float));
	memcpy(&data_8t[16], &gyroY, sizeof(float));
	memcpy(&data_8t[20], &gyroZ, sizeof(float));
	uint8_t start_symbol[1] = {'S'};
	uint8_t stop_symbol[1] = {'E'};
	HAL_UART_Transmit(&huart1, (uint8_t*)start_symbol, sizeof(start_symbol), 1000);
	HAL_UART_Transmit(&huart1, (uint8_t*)start_symbol, sizeof(start_symbol), 1000);
	HAL_UART_Transmit(&huart1, (uint8_t*)data_8t, sizeof(data_8t), 1000);
	HAL_UART_Transmit(&huart1, (uint8_t*)stop_symbol, sizeof(start_symbol), 1000);
	HAL_UART_Transmit(&huart1, (uint8_t*)stop_symbol, sizeof(start_symbol), 1000);
	vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

