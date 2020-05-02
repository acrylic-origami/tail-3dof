/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kin.h"
#include "traj.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t pos_dbl_buf[2][NUM_POS][3] = { 0 };
volatile int16_t pos[NUM_POS][3] = { 0 };
uint8_t buf_fresh = 0;
uint16_t tick_fin_buf = 0, tick_fin = 0, tick_cur = 0;
float A[4] = { 0.0, A1, A2, A3 };

extern volatile uint8_t mcp1130_ch;
extern volatile uint16_t mcp1130_mosi;
extern volatile uint16_t I_safety_buf, ERROR_safety_buf;

const joint_phys_t JOINT_PHYS[3] = {
		{ T0_HW, SMAX_HW, I_HW },
		{ T0_HS, SMAX_HS, I_HS },
		{ T0_DS, SMAX_DS, I_DS },
};
const uint16_t MIDs[3] = { CCR_MID_HW, CCR_MID_HS, CCR_MID_DS };
const uint16_t RNGs[3] = { RNG_HW / 2, RNG_HS / 2, RNG_DS / 2 };
const uint16_t PER_RADS[3] = { 1, CCR_PER_RAD_HS, CCR_PER_RAD_DS };
float vf_prev[3] = { 0 }; // previous final velocity
float t_prev[3] = { 0 }; // previous target angles (after inv kin)
const float MAX_TF = (NUM_POS - 1) * 1.0 / TIM3_FREQ;
const int8_t RHR_SGNS[3] = { SGN_HW, SGN_HS, SGN_DS };

// TEMP
#define NUM_CART_POS 2
float Ts_[NUM_CART_POS][6] = {
		{ 0.1, 0, A1 + A2 + A3 - 0.015, 0, 0.3, -0.3 },
		{ -0.1, 0, A1 + A2 + A3 - 0.015, 0, -0.3, 0.3 },
};
volatile uint8_t Ts_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float inter = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  htim3.Instance->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
//  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, CCR_MID_HS);

//  mcp1130_txrx();
//  uint8_t i = 0;
//  while(i < NUM_POS) {
//	  if(hspi1.State == HAL_SPI_STATE_READY) {
//		  pot[i] = ERROR_safety_buf & 0x3FF;
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, CCR_MID_HS + (pos[i][0] << 1));
//		  HAL_Delay(100);
//		  mcp1130_txrx();
//		  i++;
//	  }
//  }
//  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, CCR_MID_HS + (pos[0][0] << 1));
//  pot[0]++;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t k = CCR_MID_HS, edge = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(0) {
		  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET && !edge) {
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, k += 2);
			  edge = 1;
		  }
		  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET)
			  edge = 0;

		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, (k >> 1) & 1);
	  }

	  {
		  while(buf_fresh) {}; // TEMP
		  float max_tf = 0;
		  float t[3], v[3], tf[3], tbnd[2];
		  uint8_t _converged = 1;
		  if(!inv_kin(A, Ts_[Ts_idx & (NUM_CART_POS - 1)], t)) {
			  for(uint8_t i = 0; i < 3 && _converged; i++) {
				  uint8_t j = 0;
				  for(float v_ = Ts_[Ts_idx & (NUM_CART_POS - 1)][i + 3]; j < TRAJ_ITER_LIM; v_ /= 2, j++) {
					  if(!traj_t(t_prev[i], t[i], vf_prev[i], v_, tbnd, &JOINT_PHYS[i]) && tbnd[1] > 0) {
						  tf[i] = 1 / tbnd[1];
						  v[i] = v_;
//						  if(tf_ > 2 * MAX_TF)
//							  _converged = 0;
						  break;
					  }
				  }
				  if(j == TRAJ_ITER_LIM) {
					  _converged = 0;
					  break;
				  }
			  }
		  }
		  for(uint8_t i = 0; i < 3; i++) {
			  if(tf[i] > max_tf)
				  max_tf = tf[i];
		  }

		  tick_fin_buf = max_tf * 3 * TIM3_FREQ;
		  if(_converged) {
			  for(uint8_t i = 0; i < NUM_POS; i++) {
				  for(uint8_t j = 0; j < 3; j++) {
					  inter = lerp(t_prev[j], t[j], vf_prev[j], v[j], max_tf, (i * max_tf) / NUM_POS) * PER_RADS[j];
					  int16_t pos_ = fmax(-RNG_DS, fmin(RNG_DS, floor(inter)));
					  pos_ *= RHR_SGNS[j];
					  pos_dbl_buf[0][i][j] = pos_ < -RNGs[j] ? -RNGs[j] : (pos_ > RNGs[j] ? RNGs[j] : pos_);
				  }
			  }
	//		  for(uint8_t i = 0
	//		    ; i < 32 && HAL_DMA_Start(&hdma_memtomem_dma1_channel1, &pos_dbl_buf[0], &pos_dbl_buf[1], NUM_POS * 3) != HAL_OK
	//		    ; i++) {}
			  memcpy(&pos_dbl_buf[1], &pos_dbl_buf[0], sizeof(pos_dbl_buf[0][0][0]) * NUM_POS * 3);
			  for(uint8_t i = 0; i < 3; i++) {
				  vf_prev[i] = v[i];
				  t_prev[i] = t[i];
			  }
//			  memcpy(&vf_prev, &v, 3 * sizeof(float));
			  buf_fresh = 1;

			  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  }
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void mcp1130_txrx() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	asm("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;"); // try to guarantee at least 330ns of hold time
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	asm("nop;nop;nop;nop;nop;"); // try to guarantee at least 100/0.96 ns of hold time
	mcp1130_mosi = MCP3002_CONFIG | (mcp1130_ch << MCP3002_ODD_SIGN_Pos);
	uint16_t *rx = mcp1130_ch ? &ERROR_safety_buf : &I_safety_buf;
	HAL_SPI_TransmitReceive_IT(&hspi1, &mcp1130_mosi, rx, 1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
