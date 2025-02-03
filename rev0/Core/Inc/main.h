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
#include "stm32f0xx_hal.h"

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
#define SBUS_CONV_Pin GPIO_PIN_2
#define SBUS_CONV_GPIO_Port GPIOA
#define HCU_RESET_Pin GPIO_PIN_5
#define HCU_RESET_GPIO_Port GPIOA
#define DEBUG_LED_Pin GPIO_PIN_0
#define DEBUG_LED_GPIO_Port GPIOB
#define EN_N_Pin GPIO_PIN_11
#define EN_N_GPIO_Port GPIOA
#define INVALID_N_Pin GPIO_PIN_12
#define INVALID_N_GPIO_Port GPIOA
#define FORCEON_Pin GPIO_PIN_6
#define FORCEON_GPIO_Port GPIOB
#define FORCEOFF_N_Pin GPIO_PIN_7
#define FORCEOFF_N_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
