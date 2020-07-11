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
//#include "adc.h"
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
extern volatile global_state_t __STATE;
extern ADC_HandleTypeDef hadc1;

extern int16_t pos_dbl_buf[2][NUM_POS_ELE];
extern const uint16_t RNGs[3];
extern const uint16_t PER_RADS[3];
extern const int8_t RHR_SGNS[3];
extern volatile int16_t pos[NUM_POS_ELE];
extern uint8_t _buf_fresh, _con_fresh;
extern const uint16_t MIDs[3];
extern volatile uint16_t tick_fin, tick_fin_buf, tick_start_buf, tick_cur;
// TODO expand these to all the joints

// ENSURE: all of these remain in natural units
// will use 256-units in the T0 equation, since the output
// is a fraction of a T0
//#define MAX_HW_INTEG 1650 // 1rad for 1s
//volatile int32_t hw_integ = 0;

const int32_t D_COEF[NUM_HW_D_COEF] = { -85, 384, -768, 469 }; // back to front, 256-units
//volatile int32_t hw_deriv_buf[NUM_HW_D_COEF] = { 0 };
//volatile uint8_t hw_deriv_buf_idx = 0;

volatile int32_t I_safety = 0, ERROR_safety = 0;
volatile uint16_t I_safety_buf = 0, ERROR_safety_buf = 0;
volatile motion_state_t motion_state = RUN;
volatile uint8_t mcp1130_ch = 1;
volatile uint16_t mcp1130_mosi = 0;

//extern uint8_t uart_buf_in[UART_BUF_SIZE];

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
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel2;
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
  HAL_DMA_IRQHandler(&hdma_adc1);
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
  HAL_DMA_IRQHandler(&hdma_memtomem_dma1_channel2);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
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
//	if(huart3.ErrorCode == HAL_UART_ERROR_NONE) {
//		if(uart_buf_in[UART_BUF_SIZE - 1] == 0xFF) {
//			uart__buf_fresh = 1;
//			HAL_GPIO_TogglePin(LD_GPIO_Port, LD_Pin);
//			for(uint8_t i = 0; i < UART_BUF_SIZE - 1; i++) {
//				uart_buf[i] = ((int8_t)uart_buf_in[i]) * (int32_t)(A1 + A2 + A3);
//			}
//		}
//	}
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
			switch(motion_state) {
			case RUN: {

				if(_buf_fresh && HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel2, (uint32_t)(&pos_dbl_buf[1][0]), (uint32_t)(pos), sizeof(pos[0]) * NUM_POS_ELE) == HAL_OK) {
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
				break;
			}
			case FREEFALL:
				break;
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
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//	if(hadc->Instance == ADC1) {
//		adc_buf[adc_buf_idx & (NUM_ADC_BUF - 1)][adc_ch++] = HAL_ADC_GetValue(hadc);
//		if(adc_ch == NUM_ADC_CH) {
//			adc_ch = 0;
//			adc_buf_idx++;
//		}
//
//		adc_rx();
//	}
//}
//void adc_rx(void) {
//	HAL_ADC_Start_IT(&hadc1);
//}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
