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
#include "math_util.h"
#include "imath.h"
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
extern const uint16_t RNGs[3];
extern const uint16_t PER_RADS[3];
extern const int8_t RHR_SGNS[3];
extern volatile int16_t pos[NUM_POS][3];
extern volatile uint8_t Ts_idx, buf_fresh;
extern const uint16_t MIDs[3];
extern uint16_t tick_fin, tick_fin_buf, tick_cur;
// TODO expand these to all the joints

// ENSURE: all of these remain in natural units
// will use 256-units in the T0 equation, since the output
// is a fraction of a T0
#define MAX_HW_INTEG 1650 // 1rad for 1s
volatile int32_t hw_integ = 0;

#define NUM_HW_D_COEF 4 // ENSURE: power of 2
const int32_t D_COEF[NUM_HW_D_COEF] = { -85, 384, -768, 469 }; // back to front, 256-units
volatile int32_t hw_deriv_buf[NUM_HW_D_COEF] = { 0 };
volatile uint8_t hw_deriv_buf_idx = 0;
const int32_t HW_P[][2] = { { HW_P_N_0, HW_P_D_0 }, { HW_P_N_1, HW_P_D_1 }, { HW_P_N_2, HW_P_D_2 } };
const int32_t HW_I[][2] = { { HW_I_N_0, HW_I_D_0 }, { HW_I_N_1, HW_I_D_1 }, { HW_I_N_2, HW_I_D_2 } };
const int32_t HW_D[][2] = { { HW_D_N_0, HW_D_D_0 }, { HW_D_N_1, HW_D_D_1 }, { HW_D_N_2, HW_D_D_2 } };

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
volatile uint8_t hall_ticked = 0;
volatile uint8_t spi_valid = 0;
uint8_t uartb[6] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
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
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */
  if(hspi2.State == HAL_SPI_STATE_READY) {
	  I_safety_buf &= 0x3FF;
	  ERROR_safety_buf >>= 1;
	  ERROR_safety_buf &= 0x3FF;
	  mcp1130_txrx();
	  spi_valid = spi_valid >= 2 ? 2 : spi_valid + 1;
  }
  /* USER CODE END SPI2_IRQn 1 */
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM2) {
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
			{
				int16_t hw_theta = __HAL_TIM_GET_COUNTER(&htim4); // units: ticks
				int16_t hw_theta_targ = pos[tick][0];
				int16_t hw_delta = (hw_theta_targ * TICKS_PER_RAD_HW >> _W) - hw_theta; // units: ticks

				uint8_t region = 0;
				if(hw_theta < HW_P_REGION_0)
					region = 0;
				else if(hw_theta < HW_P_REGION_1)
					region = 1;
				else
					region = 2;

				uint8_t targ_region = 0;
				if(hw_theta_targ < HW_P_REGION_0)
					targ_region = 0;
				else if(hw_theta_targ < HW_P_REGION_1)
					targ_region = 1;
				else
					targ_region = 2;

				int32_t T_load = -((((int16_t)icos((hw_theta * 0x3F) / GEAR_RATIO_HW) << 1)) * I_HW >> _W); // kg-cm-256
				// NOTE: a bit of a coincidence, if switching motors need to change GEAR_RATIO_HW to
				// match hall sensor spacing
				hw_deriv_buf[(hw_deriv_buf_idx++) & (NUM_HW_D_COEF - 1)] = hw_delta;
				int32_t hw_deriv = 0;
				for(uint8_t i = 0; i < NUM_HW_D_COEF; i++) {
					hw_deriv += hw_deriv_buf[(hw_deriv_buf_idx + i) & (NUM_HW_D_COEF - 1)] * D_COEF[i];
				}
				hw_deriv >>= _W;

				int32_t hw_ccr;
				if(region != targ_region) {
					// target velocity, with feedback to the derivative
					hw_ccr = (S_TRANSITION_HW * RNG_HW) / SMAX_HW * sgn(hw_theta_targ - hw_theta) - (T_load * RNG_HW) / T0_HW;
				}
				else {
					int32_t T_offset = -(hw_deriv * T0_HW / SMAX_HW << _W) / TICKS_PER_RAD_HW / TIM2_FREQ; // kg-m-256
					//
					int32_t baseline_ccr = (-(T_load + T_offset) * RNG_HW) / T0_HW;

					hw_integ += hw_delta; // units: ticks
					hw_integ = max(-MAX_HW_INTEG, min(MAX_HW_INTEG, hw_integ));

					// ENSURE: first multiplication is with RNG_HW
					// to avoid catastrophic cancellation
					// (these are in fractions of T0 after all)

					hw_ccr =
						baseline_ccr
					  + (
							(hw_integ * RNG_HW * HW_I[region][0]) / HW_I[region][1] / TIM2_FREQ
						  + (hw_delta * RNG_HW * HW_P[region][0]) / HW_P[region][1]
//						  + (hw_deriv * RNG_HW * HW_D[region][0] * TIM2_FREQ) / HW_D[region][1]
		                ) / TICKS_PER_RAD_HW; // all the hw_* are in units of ticks: need to convert to rads


					{
						uartb[0] = (hw_delta * RNG_HW * HW_P[region][0]) / HW_P[region][1] / TICKS_PER_RAD_HW;
						uartb[1] = (hw_integ * RNG_HW * HW_I[region][0]) / HW_I[region][1] / TIM2_FREQ / TICKS_PER_RAD_HW;
						uartb[2] = T_load >> 8;
						uartb[3] = T_offset >> 8;
						uartb[4] = hw_ccr;
						uartb[5] = hw_theta;
						HAL_UART_Transmit_IT(&huart3, uartb, 6);
					}
				}
				hw_ccr = max(-MAX_CCR_HW, min(MAX_CCR_HW, hw_ccr));
//				__HAL_TIM_SET_COMPARE(&htim2, chs[0], CCR_MID_HW + sgn(hw_ccr) * DEADBAND_HW + hw_ccr);
			}

			if(tick_cur < tick_fin) {
				for(uint8_t i = 1; i < 2; i++) {
					int16_t pos_ = ((pos[tick][i] * PER_RADS[i]) >> (_W - 1)) * RHR_SGNS[i]; // CONVERT
					pos_ = max(-RNGs[i], min(RNGs[i], pos_));
//					__HAL_TIM_SET_COMPARE(&htim2, chs[i], MIDs[i] + pos_);
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
	}
}
void uart_rx(void) {
	HAL_UART_Receive_IT(&huart3, uart_buf_in, UART_BUF_SIZE * sizeof(uart_buf_in[0]));
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
