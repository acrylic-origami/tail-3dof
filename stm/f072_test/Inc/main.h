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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

////////////
// AS1130 //
////////////

#define AS1130_ADDR             0b01101110
#define AS1130_ADDR_W           (AS1130_ADDR_R | 1)
#define AS1130_ADDR_R           AS1130_ADDR
#define AS1130_REG_SEL_ADDR     0xFD
#define AS1130_CONFIG_REG       0x06
#define AS1130_SHDN_REG         0x09
#define AS1130_CURR_REG         0x05
#define AS1130_PIC_REG          0x00
#define AS1130_PIC_EN_SIZE      0x02
#define AS1130_ONOFF_FR0_REG    0x01
#define AS1130_PWM_SET0_REG     0x40
#define AS1130_DATA_LOAD_ADDR   0x18
typedef enum as1130_state_e {
  CONFIG,
  SHDN,
  VFY_SEL,
  VFY,
  CURR,
  PIC_EN,
	ONOFF_SEL,
	ONOFF,
	PWM_SEL,
	READY
} as1130_state_t;
#define AS1130_CONFIG_SIZE      2
#define AS1130_SHDN_SIZE        2
#define AS1130_CURR_SIZE        2
#define AS1130_ONOFF_SEL_SIZE   2
#define AS1130_ONOFF_DATA_SIZE  24
#define AS1130_PWM_SEL_SIZE     2
#define PX_SIZE                 25

/////////////
// MCP3002 //
/////////////

#define MCP3002_CONFIG 0b11010000
#define MCP3002_ODD_SIGN_Pos 5

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
