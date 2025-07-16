/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"

#include "OpenFWDconf.h"
#include "mainLoop.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_PWM_CHANNEL 16
#define PWM_FACT 200
#define RUDDER_FACT 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PWM_UP(PIN, ID) \
  if(pwm_counter == pwm_width[ID]) \
    PIN##_GPIO_Port->BRR = PIN##_Pin;
#define PWM_DOWN(PIN, ID) \
  PIN##_GPIO_Port->BSRR = PIN##_Pin; \
  pwm_width[ID] = pwm_new_width[ID];
#define RUDDER_UP(PIN, ID) \
  if(rudder_counter == pwm_width[ID]) \
    PIN##_GPIO_Port->BRR = PIN##_Pin;
#define RUDDER_DOWN(PIN, ID) PWM_DOWN(PIN, ID)
#define MOTOR_MARCO(MACRO, MOTID) \
  MACRO(MOT##MOTID##_F, (2 * MOTID - 2)); \
  MACRO(MOT##MOTID##_R, (2 * MOTID - 1));
#ifdef MOT1_ENABLED
  #define MOTOR1_MARCO(MARCO) MOTOR_MARCO(MARCO, 1)
#else
  #define MOTOR1_MARCO(MARCO)
#endif
#ifdef MOT2_ENABLED
  #define MOTOR2_MARCO(MARCO) MOTOR_MARCO(MARCO, 2)
#else
  #define MOTOR2_MARCO(MARCO)
#endif
#define PWM_MACROS(MARCO) \
  MOTOR1_MARCO(MARCO); \
  MOTOR2_MARCO(MARCO); \
  //MACRO(LED1, 4); \
  //MACRO(LED2, 5); \
  //MACRO(LED3, 6); \
  //MACRO(LED4, 7);
#ifdef RUD1_ENABLED
  #define RUD1_MARCO(MARCO) MARCO(RUD1_S, 8)
#else
  #define RUD1_MARCO(MARCO)
#endif
#ifdef RUD2_ENABLED
  #define RUD2_MARCO(MARCO) MARCO(RUD2_S, 8)
#else
  #define RUD2_MARCO(MARCO)
#endif
#ifdef RUD3_ENABLED
  #define RUD3_MARCO(MARCO) MARCO(RUD3_S, 8)
#else
  #define RUD3_MARCO(MARCO)
#endif
#ifdef RUD4_ENABLED
  #define RUD4_MARCO(MARCO) MARCO(RUD4_S, 8)
#else
  #define RUD4_MARCO(MARCO)
#endif
#ifdef RUD5_ENABLED
  #define RUD5_MARCO(MARCO) MARCO(RUD5_S, 8)
#else
  #define RUD5_MARCO(MARCO)
#endif
#ifdef RUD6_ENABLED
  #define RUD6_MARCO(MARCO) MARCO(RUD6_S, 8)
#else
  #define RUD6_MARCO(MARCO)
#endif
#define RUDDER_MARCOS(MARCO) \
  RUD1_MARCO(MARCO) \
  RUD2_MARCO(MARCO) \
  RUD3_MARCO(MARCO) \
  RUD4_MARCO(MARCO) \
  RUD5_MARCO(MARCO) \
  RUD6_MARCO(MARCO)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint8_t pwm_new_width[MAX_PWM_CHANNEL];
extern TIM_HandleTypeDef htim2;
static uint8_t beep_en;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_pwm_width(uint8_t id, uint8_t width) {
  pwm_new_width[id] = width;
}
void set_beep(uint8_t en, uint16_t arr) {
  beep_en = en;
  htim2.Instance->ARR = arr;
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
  // this func is called every 10us or 100kHz
  static uint8_t pwm_counter = 0;
  static uint16_t rudder_counter = 0;
  static uint8_t pwm_width[MAX_PWM_CHANNEL];
  TIM1->SR = ~TIM_SR_UIF;
  pwm_counter++; rudder_counter++;
  PWM_MACROS(PWM_DOWN);
  RUDDER_MARCOS(RUDDER_DOWN);
  if(pwm_counter == PWM_FACT) {
    pwm_counter = 0;
    PWM_MACROS(PWM_UP);
    if(rudder_counter == RUDDER_FACT) {
      rudder_counter = 0;
      RUDDER_MARCOS(RUDDER_UP);
      //mid freq = 100kHz / 2000 = 50Hz, T = 20ms
      mainLoop_pushTask(LOW_FREQ_SCHED);
    }
  }
  return;
  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  static uint8_t beep_status;
  TIM2->SR = ~TIM_SR_UIF;
  if(beep_en) {
    if(beep_status) {
      BEEP_GPIO_Port->BRR = BEEP_Pin;
      beep_status = 0;
    } else {
      BEEP_GPIO_Port->BSRR = BEEP_Pin;
      beep_status = 1;
    }
  }
  return;
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
