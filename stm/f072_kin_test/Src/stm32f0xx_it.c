/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "traj.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
typedef enum motion_state_t_e {
	RUN,
	FREEFALL
} motion_state_t;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern int16_t pos_dbl_buf[2][NUM_POS][3];
extern volatile int16_t pos[NUM_POS][3];
extern const uint16_t MIDs[3];
extern volatile uint8_t Ts_idx, buf_fresh;
extern uint16_t tick_fin, tick_fin_buf, tick_cur;
// TODO expand these to all the joints
volatile int32_t I_safety = 0, ERROR_safety = 0;
volatile uint16_t I_safety_buf = 0, ERROR_safety_buf = 0;
volatile motion_state_t motion_state = RUN;
volatile uint8_t mcp1130_ch = 1;
volatile uint16_t mcp1130_mosi = 0;

// TEMP
volatile int16_t error;
volatile uint8_t spi_valid = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
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
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_memtomem_dma1_channel1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	uint16_t tick = ((uint16_t)tick_cur * NUM_POS) / tick_fin;
	uint16_t chs[3] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3 };
	tick = tick >= NUM_POS ? NUM_POS - 1 : tick;
	switch(motion_state) {
	case RUN: {
		if(buf_fresh) {
			if(buf_fresh && tick_cur >= tick_fin && HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel1, &pos_dbl_buf[1], pos, sizeof(pos[0][0]) * NUM_POS * 3) == HAL_OK) {
				Ts_idx++;
				buf_fresh = 0;
				tick_cur = 0;
				tick_fin = 0;
			}
		}
		if(tick_cur < tick_fin) {
			for(uint8_t i = 1; i < 3; i++) {
				__HAL_TIM_SET_COMPARE(&htim3, chs[i], MIDs[i] + (pos[tick][i] << 1));
			}
			tick_cur++;
			//P[D?] controller for Hobbywing
			int16_t delta = ((pos[tick][0] + ((int16_t)TIM1->CNT)) * P_HW_N) / P_HW_D; // pos[tick][0]
			delta = delta < -MAX_CCR_HW ? -MAX_CCR_HW : (delta > MAX_CCR_HW ? MAX_CCR_HW : delta);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, CCR_MID_HW + sgn(delta) * DEADBAND_HW + delta);

			if(abs(delta) < 4) {
				delta++;
			}

			if(spi_valid >= 2) {
				// TODO generalize over all joints
				error = (pos[tick > 0 ? tick - 1 : 0][0] << 1) - (int32_t)(ERROR_safety_buf - POT_MID_HS) * ERROR_SCALE_HS_N / ERROR_SCALE_HS_D;
				I_safety = I_safety_buf + (I_safety * I_FILT_COEFF_N) / I_FILT_COEFF_D;
				ERROR_safety = error + (ERROR_safety * ERROR_FILT_COEFF_N) / ERROR_FILT_COEFF_D;
				if(abs(I_safety) > I_MAX_N || abs(ERROR_safety) > ERROR_MAX_N) {
					// disable hobbywing
		//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, CCR_MID_HW);
					// TODO disable servo enables
					error++;
				}
			}
		}
		break;
	}
	case FREEFALL:
		break;
	}

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */
  if(hspi1.State == HAL_SPI_STATE_READY) {
	  I_safety_buf &= 0x3FF;
	  ERROR_safety_buf &= 0x3FF;
	  mcp1130_txrx();
	  spi_valid = spi_valid >= 2 ? 2 : spi_valid + 1;
  }
  /* USER CODE END SPI1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
