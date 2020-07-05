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
#include "stm32f3xx_hal.h"

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
uint16_t pos_idx(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define Z_Pin GPIO_PIN_4
#define Z_GPIO_Port GPIOA
#define Y_Pin GPIO_PIN_6
#define Y_GPIO_Port GPIOA
#define X_Pin GPIO_PIN_7
#define X_GPIO_Port GPIOA
#define SERVO2_NSEL_Pin GPIO_PIN_1
#define SERVO2_NSEL_GPIO_Port GPIOB
#define SERVO_EN_Pin GPIO_PIN_8
#define SERVO_EN_GPIO_Port GPIOA
#define SERVO1_NSEL_Pin GPIO_PIN_10
#define SERVO1_NSEL_GPIO_Port GPIOA
#define LIMSW2_Pin GPIO_PIN_11
#define LIMSW2_GPIO_Port GPIOA
#define LIMSW1_Pin GPIO_PIN_12
#define LIMSW1_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD_Pin GPIO_PIN_3
#define LD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define MIN_HOMING_CCR (-RNG_HW)
#define HOMING_TICK 40 // ms
#define HOMING_STRIDE 4 // CCR units

#define NUM_HAND_CTRL_BUF 64
#define NUM_HAND_CTRL_AX 3
#define HAND_CTRL_BUF_DS 2 // downsampling factor (simple skipping)
#define HAND_CTRL_DEADBAND 3
#define NUM_HAND_CTRL_D_COEF 4
// (11.704699910719626, [-4, 0, -11]) (18.16590212458495, [-16, -5, -7])
#define HAND_R0 135 // UNFLOAT 0.470
#define HAND_R1 158 // UNFLOAT 0.578

#define NUM_POS 128
#define NUM_POS_MSK 0x3F
#define NUM_POS_Pos 6
#define NUM_JOINTS 3
#define NUM_POS_DERIV 2
#define POS_STRIDE (NUM_JOINTS * NUM_POS_DERIV)
#define NUM_POS_ELE (NUM_POS * POS_STRIDE)

#define SPEEDUP_FACTOR_N 9
#define SPEEDUP_FACTOR_D 4

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
