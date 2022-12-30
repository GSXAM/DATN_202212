/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_Pin GPIO_PIN_3
#define SD_GPIO_Port GPIOA
#define C_SCP_Pin GPIO_PIN_5
#define C_SCP_GPIO_Port GPIOA
#define S_SCP_Pin GPIO_PIN_6
#define S_SCP_GPIO_Port GPIOA
#define S_SCP_EXTI_IRQn EXTI4_15_IRQn
#define BUZZ_Pin GPIO_PIN_7
#define BUZZ_GPIO_Port GPIOA
#define BT_MODE_Pin GPIO_PIN_0
#define BT_MODE_GPIO_Port GPIOB
#define BT_MODE_EXTI_IRQn EXTI0_1_IRQn
#define BT_MINUS_Pin GPIO_PIN_1
#define BT_MINUS_GPIO_Port GPIOB
#define BT_MINUS_EXTI_IRQn EXTI0_1_IRQn
#define BT_PLUS_Pin GPIO_PIN_8
#define BT_PLUS_GPIO_Port GPIOA
#define BT_PLUS_EXTI_IRQn EXTI4_15_IRQn
#define BT_OK_Pin GPIO_PIN_9
#define BT_OK_GPIO_Port GPIOA
#define BT_OK_EXTI_IRQn EXTI4_15_IRQn
#define CD_Pin GPIO_PIN_10
#define CD_GPIO_Port GPIOA
#define RESETLCD_Pin GPIO_PIN_11
#define RESETLCD_GPIO_Port GPIOA
#define CSLCD_Pin GPIO_PIN_12
#define CSLCD_GPIO_Port GPIOA
#define PGOOD_Pin GPIO_PIN_15
#define PGOOD_GPIO_Port GPIOA
#define CSDAC_Pin GPIO_PIN_4
#define CSDAC_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
