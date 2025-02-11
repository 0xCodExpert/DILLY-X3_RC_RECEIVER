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
#define STATUS_HOTRC_Pin GPIO_PIN_12
#define STATUS_HOTRC_GPIO_Port GPIOB
#define STATUS_FUTABA_Pin GPIO_PIN_13
#define STATUS_FUTABA_GPIO_Port GPIOB
#define DEBUG_LED_B_Pin GPIO_PIN_14
#define DEBUG_LED_B_GPIO_Port GPIOB
#define DEBUG_LED_R_Pin GPIO_PIN_15
#define DEBUG_LED_R_GPIO_Port GPIOB
#define EN_N_Pin GPIO_PIN_10
#define EN_N_GPIO_Port GPIOC
#define FORCEOFF_N_Pin GPIO_PIN_11
#define FORCEOFF_N_GPIO_Port GPIOC
#define FORCEON_Pin GPIO_PIN_12
#define FORCEON_GPIO_Port GPIOC
#define INVALID_N_Pin_Pin GPIO_PIN_2
#define INVALID_N_Pin_GPIO_Port GPIOD
#define STB_CAN_Pin GPIO_PIN_5
#define STB_CAN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
