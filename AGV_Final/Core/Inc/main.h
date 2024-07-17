/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define IN1_Pin GPIO_PIN_2
#define IN1_GPIO_Port GPIOE
#define Button_1_Pin GPIO_PIN_3
#define Button_1_GPIO_Port GPIOE
#define Button_2_Pin GPIO_PIN_4
#define Button_2_GPIO_Port GPIOE
#define IN4_Pin GPIO_PIN_5
#define IN4_GPIO_Port GPIOE
#define IN5_Pin GPIO_PIN_6
#define IN5_GPIO_Port GPIOE
#define IN6_Pin GPIO_PIN_13
#define IN6_GPIO_Port GPIOC
#define IN7_Pin GPIO_PIN_14
#define IN7_GPIO_Port GPIOC
#define RS485_EN_Pin GPIO_PIN_6
#define RS485_EN_GPIO_Port GPIOA
#define Break_2_Pin GPIO_PIN_7
#define Break_2_GPIO_Port GPIOA
#define Break_1_Pin GPIO_PIN_4
#define Break_1_GPIO_Port GPIOC
#define OUT1_Pin GPIO_PIN_7
#define OUT1_GPIO_Port GPIOE
#define OUT2_Pin GPIO_PIN_8
#define OUT2_GPIO_Port GPIOE
#define OUT3_Pin GPIO_PIN_10
#define OUT3_GPIO_Port GPIOE
#define OUT4_Pin GPIO_PIN_12
#define OUT4_GPIO_Port GPIOE
#define OUT5_Pin GPIO_PIN_13
#define OUT5_GPIO_Port GPIOE
#define OUT6_Pin GPIO_PIN_14
#define OUT6_GPIO_Port GPIOE
#define IN14_Pin GPIO_PIN_15
#define IN14_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
