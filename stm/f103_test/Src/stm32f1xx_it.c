/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "traj.h"
#include "kin.h"
#include "consts.h"
#include <string.h>
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
extern TIM_HandleTypeDef htim4;

extern int16_t pos_dbl_buf[2][NUM_POS][3];
extern volatile int16_t pos[NUM_POS][3];
extern volatile uint8_t Ts_idx, buf_fresh;
extern const uint16_t MIDs[3];
extern uint16_t tick_fin, tick_fin_buf, tick_cur;
// TODO expand these to all the joints
volatile int32_t hw_state = 0;
volatile int32_t I_safety = 0, ERROR_safety = 0;
volatile uint16_t I_safety_buf = 0, ERROR_safety_buf = 0;
volatile motion_state_t motion_state = RUN;
volatile uint8_t mcp1130_ch = 1;
volatile uint16_t mcp1130_mosi = 0;

extern uint8_t uart_buf_in[UART_BUF_SIZE];
extern volatile uint32_t uart_buf[UART_BUF_SIZE];
extern volatile uint8_t uart_buf_fresh;

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
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart3;
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
  * @brief This function handles DMA1 channel1 global interrupt.
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
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	uint16_t tick = ((uint16_t)tick_cur * NUM_POS) / tick_fin;
	uint16_t chs[3] = { TIM_CHANNEL_3, TIM_CHANNEL_1, TIM_CHANNEL_2 };
	tick = tick >= NUM_POS ? NUM_POS - 1 : tick;
	switch(motion_state) {
	case RUN: {
		if(buf_fresh) {
			if(buf_fresh && tick_cur >= tick_fin && HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel1, &pos_dbl_buf[1], pos, sizeof(pos[0][0]) * NUM_POS * 3) == HAL_OK) {
				Ts_idx++;
				buf_fresh = 0;
				tick_cur = 0;
				tick_fin = tick_fin_buf;
			}
		}

		// PI controller for Hobbywing
		int16_t delta = ((pos[tick][0] * TICKS_PER_RAD_HW >> _W) + (int16_t)(__HAL_TIM_GET_COUNTER(&htim4))) * HW_P_N / HW_P_D; // pos[tick][0]
		delta = delta < -MAX_CCR_HW ? -MAX_CCR_HW : (delta > MAX_CCR_HW ? MAX_CCR_HW : delta);
		hw_state = delta + (hw_state * HW_I_N / HW_I_D);
		__HAL_TIM_SET_COMPARE(&htim2, chs[0], CCR_MID_HW + sgn(hw_state) * DEADBAND_HW + hw_state);

		if(abs(delta) < 4) {
			delta++;
		}

		if(tick_cur < tick_fin) {
			for(uint8_t i = 1; i < 3; i++) {
				__HAL_TIM_SET_COMPARE(&htim2, chs[i], MIDs[i] + (pos[tick][i] << 1));
			}
			tick_cur++;

			if(spi_valid >= 2) {

				// TODO generalize over all joints
				uint16_t pos_ = pos[tick > 0 ? tick - 1 : 0][1];
				error = (pos_ << 1) - (int32_t)(ERROR_safety_buf - POT_MID_HS) * ERROR_SCALE_HS_N / ERROR_SCALE_HS_D;

//				{
//					// TEMP
//					elog[elog_idx & NUM_ELOG_MSK][0] = pos_;
//					elog[elog_idx & NUM_ELOG_MSK][1] = ERROR_safety_buf - POT_MID_HS;
//					elog_idx++;
//				}

				I_safety = I_safety_buf + (I_safety * I_FILT_COEFF_N) / I_FILT_COEFF_D;
				ERROR_safety = error + (ERROR_safety * ERROR_FILT_COEFF_N) / ERROR_FILT_COEFF_D;
				if(abs(I_safety) > I_MAX_N || abs(ERROR_safety) > ERROR_MAX_N) {
					// disable hobbywing
					//			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CCR_MID_HW);
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

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	if(huart3.ErrorCode == HAL_UART_ERROR_NONE) {
		if(uart_buf_in[UART_BUF_SIZE - 1] == 0xFF) {
			uart_buf_fresh = 1;
			HAL_GPIO_TogglePin(LD_GPIO_Port, LD_Pin);
			for(uint8_t i = 0; i < UART_BUF_SIZE - 1; i++) {
				uart_buf[i] = ((int8_t)uart_buf_in[i]) * (int32_t)(A1 + A2 + A3);
			}
		}
	}
	if(huart3.RxState == HAL_UART_STATE_READY) {
		uart_rx();
	}
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void uart_rx(void) {
	HAL_UART_Receive_IT(&huart3, uart_buf_in, UART_BUF_SIZE * sizeof(uart_buf_in[0]));
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
