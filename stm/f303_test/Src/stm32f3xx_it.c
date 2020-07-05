/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
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
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "traj.h"
#include "kin.h"
//#include "adc.h"
#include "consts.h"
#include "math_util.h"
#include "imath.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern volatile global_state_t __STATE;

extern int16_t pos_dbl_buf[2][NUM_POS_ELE];
extern const uint16_t RNGs[3];
extern const uint16_t PER_RADS[3];
extern const int8_t RHR_SGNS[3];
extern volatile int16_t pos[NUM_POS_ELE];
extern uint8_t _buf_fresh, _con_fresh;
extern const uint16_t MIDs[3];
extern volatile uint16_t tick_fin, tick_fin_buf, tick_start_buf, tick_cur;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc2;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
//extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
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
  * @brief This function handles Pre-fetch fault, memory access fault.
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
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
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
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
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
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXT line 26.
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
int16_t uart_buf[6];
const uint16_t chs[3] = { TIM_CHANNEL_3, TIM_CHANNEL_1, TIM_CHANNEL_2 };
extern uint16_t j0_ccr_adjust;
volatile int16_t lower_lim_break = -1, upper_lim_break = -1;
volatile uint16_t lower_lim_break_j0_ccr = 0, upper_lim_break_j0_ccr = 0;
#define NUM_CCR_LPF 4 // delay of approx. 50ms
volatile uint16_t ccr_lpf_buf[NUM_CCR_LPF][3] = { 0 };
volatile uint16_t ccr_lpf_idx = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(__STATE == GLOBAL_RUN) {
		if(htim->Instance == TIM2) {
			if(HAL_GPIO_ReadPin(LIMSW1_GPIO_Port, LIMSW1_Pin))
				lower_lim_break = -1; // re-enable unconditionally if the bottom limsw is reset

			uint16_t tick = pos_idx();
			if(_buf_fresh && HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel1, (uint32_t)(&pos_dbl_buf[1][0]), (uint32_t)(pos), sizeof(pos[0]) * NUM_POS_ELE) == HAL_OK) {
				//					for(uint8_t i = 0; i < 3; i++)
				//						uart_buf[i] = pos_dbl_buf[1][i * NUM_POS_DERIV];
				//					for(uint8_t i = 0; i < 3; i++)
				//						uart_buf[i+3] = pos_dbl_buf[1][(NUM_POS - 1) * POS_STRIDE + i * NUM_POS_DERIV];
				//					HAL_UART_Transmit_IT(&huart3, uart_buf, 6 * sizeof(uart_buf[0]));

				HAL_GPIO_TogglePin(LD_GPIO_Port, LD_Pin);
				_buf_fresh = 0;
				_con_fresh = 1;
				tick_cur = tick_start_buf;
				tick_fin = tick_fin_buf;
			}

			if(tick_cur <= tick_fin) {
				for(uint8_t i = 0; i < 3; i++) {
					int16_t pos_ = ((pos[tick * POS_STRIDE + i * NUM_POS_DERIV] * PER_RADS[i]) >> _W) * RHR_SGNS[i]; // CONVERT
					pos_ = max(-RNGs[i], min(RNGs[i], pos_));
					uint16_t ccr = MIDs[i] + pos_;
					ccr_lpf_buf[ccr_lpf_idx & (NUM_CCR_LPF - 1)][i] = ccr;

					//						if(i == 0)
					//							ccr += j0_ccr_adjust;

					if(i != 0 || (upper_lim_break < 0 && (lower_lim_break < 0 || (lower_lim_break > 0 && ccr > lower_lim_break_j0_ccr)))) {
						uint32_t sum = 0;
						for(uint8_t j = 0; j < NUM_CCR_LPF; sum += ccr_lpf_buf[j++][i]);
						__HAL_TIM_SET_COMPARE(&htim2, chs[i], sum / NUM_CCR_LPF);
					}
					else {
						// force freefall
						__HAL_TIM_SET_COMPARE(&htim2, chs[i], HW_FREEFALL_CCR);
						if(upper_lim_break >= 0 && HAL_GPIO_ReadPin(LIMSW2_GPIO_Port, LIMSW2_Pin) && ccr < upper_lim_break_j0_ccr)
							upper_lim_break--;

						if(ccr > lower_lim_break_j0_ccr && upper_lim_break == -1 && lower_lim_break > 0) { // if lower_lim_break == 0, timed out: lock until reset
							lower_lim_break--;
						}
					}
				}
				tick_cur++;
				ccr_lpf_idx++;

				//					if(spi_valid >= 2) {
				//						uint16_t pos_ = pos[(tick > 0 ? tick - 1 : 0) * POS_STRIDE + 1 * NUM_POS_DERIV]; // TODO generalize over all axes
				//						error = (pos_ << 1) - (int32_t)(ERROR_safety_buf - POT_MID_HS) * ERROR_SCALE_HS_N / ERROR_SCALE_HS_D;
				//
				//						//				{
				//						//					// TEMP
				//						//					elog[elog_idx & NUM_ELOG_MSK][0] = pos_;
				//						//					elog[elog_idx & NUM_ELOG_MSK][1] = ERROR_safety_buf - POT_MID_HS;
				//						//					elog_idx++;
				//						//				}
				//
				//						I_safety = I_safety_buf + (I_safety * I_FILT_COEFF_N) / I_FILT_COEFF_D;
				//						ERROR_safety = error + (ERROR_safety * ERROR_FILT_COEFF_N) / ERROR_FILT_COEFF_D;
				//						if(abs(I_safety) > I_MAX_N || abs(ERROR_safety) > ERROR_MAX_N) {
				//							// disable hobbywing
				//							//			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CCR_MID_HW);
				//							// TODO disable servo enables
				//							error++;
				//						}
				//					}
			}
			else if(upper_lim_break >= 0 || lower_lim_break >= 0) {
				__HAL_TIM_SET_COMPARE(&htim2, chs[0], HW_FREEFALL_CCR);
			}
		}
		else {

		}
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	switch(pin) {
	case LIMSW2_Pin:
		if(upper_lim_break < 0) {
//			upper_lim_break = UPPER_LIM_BREAK_INIT; // freefall for some time
			upper_lim_break_j0_ccr = __HAL_TIM_GET_COMPARE(&htim2, chs[0]);
		}
		break;
	case LIMSW1_Pin:
		if(__STATE == GLOBAL_HOMING) {
			__STATE = GLOBAL_RUN;
		}
		else {
			lower_lim_break = LOWER_LIM_BREAK_TIMEOUT;
			lower_lim_break_j0_ccr = __HAL_TIM_GET_COMPARE(&htim2, chs[0]);
		}
		break;
	}
}
void uart_rx(void) {
//	HAL_UART_Receive_IT(&huart3, uart_buf_in, UART_BUF_SIZE * sizeof(uart_buf_in[0]));
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
