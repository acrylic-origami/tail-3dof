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
typedef enum global_state_e {
	GLOBAL_RESET, GLOBAL_HOMING, GLOBAL_RUN, GLOBAL_BREAK
} global_state_t;
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
#define TIM2_J1_PWM_Pin GPIO_PIN_0
#define TIM2_J1_PWM_GPIO_Port GPIOA
#define TIM2_J2_PWM_Pin GPIO_PIN_1
#define TIM2_J2_PWM_GPIO_Port GPIOA
#define TIM2_J0_PWM_Pin GPIO_PIN_2
#define TIM2_J0_PWM_GPIO_Port GPIOA
#define Xin_Pin GPIO_PIN_3
#define Xin_GPIO_Port GPIOA
#define Yin_Pin GPIO_PIN_0
#define Yin_GPIO_Port GPIOB
#define Zin_Pin GPIO_PIN_1
#define Zin_GPIO_Port GPIOB
#define LIMSW2_Pin GPIO_PIN_12
#define LIMSW2_GPIO_Port GPIOB
#define LIMSW2_EXTI_IRQn EXTI15_10_IRQn
#define SERVO1_NSEL_Pin GPIO_PIN_8
#define SERVO1_NSEL_GPIO_Port GPIOA
#define SERVO2_NSEL_Pin GPIO_PIN_9
#define SERVO2_NSEL_GPIO_Port GPIOA
#define LIMSW1_Pin GPIO_PIN_10
#define LIMSW1_GPIO_Port GPIOA
#define LIMSW1_EXTI_IRQn EXTI15_10_IRQn
#define SERVO1_Pin GPIO_PIN_15
#define SERVO1_GPIO_Port GPIOA
#define SERVO2_Pin GPIO_PIN_3
#define SERVO2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define MIN_HOMING_CCR (-RNG_HW)
#define HOMING_TICK 40 // ms
#define HOMING_STRIDE 4 // CCR units

#define NUM_HAND_CTRL_BUF 64
#define NUM_HAND_CTRL_AX 3
#define HAND_CTRL_BUF_DS 2 // downsampling factor (simple skipping)
#define HAND_CTRL_DEADBAND 3
// (11.704699910719626, [-4, 0, -11]) (18.16590212458495, [-16, -5, -7])
#define HAND_R0 117 // UNFLOAT 0.4572
#define HAND_R1 182 // UNFLOAT 0.7096

#define NUM_POS 128
#define NUM_POS_MSK 0x3F
#define NUM_POS_Pos 6
#define NUM_JOINTS 3
#define NUM_POS_DERIV 2
#define POS_STRIDE (NUM_JOINTS * NUM_POS_DERIV)
#define NUM_POS_ELE (NUM_POS * POS_STRIDE)

#define UART_BUF_SIZE 10

#define TRAJ_ITER_LIM 3

#define TIM2_FREQ 50

#define LOWER_LIM_BREAK_TIMEOUT (TIM2_FREQ)
#define UPPER_LIM_BREAK_INIT (TIM2_FREQ / 4)

#define MCP3002_CONFIG (0b11010000 << 8)
#define MCP3002_ODD_SIGN_Pos 13

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
