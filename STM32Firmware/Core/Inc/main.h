/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define MOT1_F_Pin GPIO_PIN_13
#define MOT1_F_GPIO_Port GPIOC
#define MOT1_R_Pin GPIO_PIN_14
#define MOT1_R_GPIO_Port GPIOC
#define MOT1_P_Pin GPIO_PIN_15
#define MOT1_P_GPIO_Port GPIOC
#define MOT1_S_Pin GPIO_PIN_0
#define MOT1_S_GPIO_Port GPIOA
#define MOT2_F_Pin GPIO_PIN_1
#define MOT2_F_GPIO_Port GPIOA
#define MOT2_R_Pin GPIO_PIN_4
#define MOT2_R_GPIO_Port GPIOA
#define MOT2_P_Pin GPIO_PIN_5
#define MOT2_P_GPIO_Port GPIOA
#define MOT2_S_Pin GPIO_PIN_6
#define MOT2_S_GPIO_Port GPIOA
#define RUD3_S_Pin GPIO_PIN_7
#define RUD3_S_GPIO_Port GPIOA
#define RUD3_P_Pin GPIO_PIN_0
#define RUD3_P_GPIO_Port GPIOB
#define RUD2_S_Pin GPIO_PIN_1
#define RUD2_S_GPIO_Port GPIOB
#define RUD2_P_Pin GPIO_PIN_2
#define RUD2_P_GPIO_Port GPIOB
#define RUD1_S_Pin GPIO_PIN_12
#define RUD1_S_GPIO_Port GPIOB
#define RUD1_P_Pin GPIO_PIN_13
#define RUD1_P_GPIO_Port GPIOB
#define RUD4_S_Pin GPIO_PIN_14
#define RUD4_S_GPIO_Port GPIOB
#define RUD4_P_Pin GPIO_PIN_15
#define RUD4_P_GPIO_Port GPIOB
#define RUD5_S_Pin GPIO_PIN_8
#define RUD5_S_GPIO_Port GPIOA
#define RUD5_P_Pin GPIO_PIN_9
#define RUD5_P_GPIO_Port GPIOA
#define RUD6_S_Pin GPIO_PIN_10
#define RUD6_S_GPIO_Port GPIOA
#define RUD6_P_Pin GPIO_PIN_11
#define RUD6_P_GPIO_Port GPIOA
#define BEEP_Pin GPIO_PIN_12
#define BEEP_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
