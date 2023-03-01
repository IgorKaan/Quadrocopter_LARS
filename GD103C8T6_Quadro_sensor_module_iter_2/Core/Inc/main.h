/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define INT_MPU_Pin GPIO_PIN_3
#define INT_MPU_GPIO_Port GPIOA
#define NSS_MPU_Pin GPIO_PIN_4
#define NSS_MPU_GPIO_Port GPIOA
#define INT_2_Pin GPIO_PIN_10
#define INT_2_GPIO_Port GPIOB
#define NSS_3_Pin GPIO_PIN_11
#define NSS_3_GPIO_Port GPIOB
#define NSS_MS_Pin GPIO_PIN_12
#define NSS_MS_GPIO_Port GPIOB
#define NSS_2_Pin GPIO_PIN_8
#define NSS_2_GPIO_Port GPIOA
#define INT_3_Pin GPIO_PIN_11
#define INT_3_GPIO_Port GPIOA
#define INT_4_Pin GPIO_PIN_12
#define INT_4_GPIO_Port GPIOA
#define INT_5_Pin GPIO_PIN_15
#define INT_5_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
