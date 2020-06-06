/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
void mcp1130_txrx(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD_Pin GPIO_PIN_13
#define LD_GPIO_Port GPIOC
#define SERVO1_NSEL_Pin GPIO_PIN_8
#define SERVO1_NSEL_GPIO_Port GPIOA
#define SERVO2_NSEL_Pin GPIO_PIN_9
#define SERVO2_NSEL_GPIO_Port GPIOA
#define LIMSW1_Pin GPIO_PIN_11
#define LIMSW1_GPIO_Port GPIOA
#define LIMSW1_EXTI_IRQn EXTI15_10_IRQn
#define LIMSW2_Pin GPIO_PIN_12
#define LIMSW2_GPIO_Port GPIOA
#define LIMSW2_EXTI_IRQn EXTI15_10_IRQn
#define SERVO1_Pin GPIO_PIN_15
#define SERVO1_GPIO_Port GPIOA
#define SERVO2_Pin GPIO_PIN_3
#define SERVO2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define NUM_POS 128
#define NUM_POS_MSK 0x3F
#define NUM_POS_Pos 6

#define UART_BUF_SIZE 7

#define TRAJ_ITER_LIM 8

#define TIM2_FREQ 50

#define MCP3002_CONFIG (0b11010000 << 8)
#define MCP3002_ODD_SIGN_Pos 13

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
