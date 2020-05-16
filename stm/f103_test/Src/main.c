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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kin.h"
#include "stm32f1xx_it.h"
#include "traj.h"
#include "consts.h"
#include "math_util.h"
#include "as1130.h"
#include <string.h>

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
int32_t A[4] = { 0, A1, A2, A3 };

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
int32_t vf_prev[3] = { 0 }; // previous final velocity
int32_t t_prev[3] = { 0 }; // previous target angles (after inv kin)
const int8_t RHR_SGNS[3] = { SGN_HW, SGN_HS, SGN_DS };

// TEMP
#define NUM_CART_POS 2
int32_t Ts_[NUM_CART_POS][6] = {
		{-20, A1 + A2 + A3 - 40, 0, 0, 77, -77}, // { A2, A1+20, -(A3-40), 0, 77, -77 },
		{20, A1 + A2 + A3 - 40, 0, 0, -77, 77}, // { -A2, A1+20, A3-40, 0, -77, 77 },
};
volatile uint8_t Ts_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void mcp1130_txrx(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// TEMP
uint8_t px[2] = { 0 };
const uint8_t k[2] = { 0xFD, 0x40 };

uint8_t uart_buf_in[UART_BUF_SIZE] = { 0 };
volatile uint32_t uart_buf[UART_BUF_SIZE] = { 0 };
volatile int8_t uart_buf_fresh = 0;
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
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
  HAL_InitTick(TICK_INT_PRIORITY);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim4);

  AS1130_Init();

  htim2.Instance->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
  htim3.Instance->CCER |= TIM_CCER_CC2E;

  // TODO startup routine by driving j0 to limit switch
  // TODO softer startup routine by reading from servo pot ADC and slow-driving them to center
  HAL_GPIO_WritePin(SERVO1_GPIO_Port, SERVO1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO2_GPIO_Port, SERVO2_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COUNTER(&htim4, 51);

  uart_rx();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  for(uint8_t j = 0; ; j++)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  {
		  for(uint8_t i = 0; i < 132; i++) {
			  if(hi2c1.State == HAL_I2C_STATE_READY) {
		//	  		  px[(i >> 3) + 1] |= 1 << (i & 7);
		//	  		  for(uint8_t j = 1; j < PX_SIZE; j++) {
		//	  			  px[j] = 1 << (i & 7);
		//	  		  }
		//	  		  px[2] &= 0x1F; // preserve PWM frame
				  HAL_I2C_Master_Transmit(&hi2c1, AS1130_ADDR, k, 2, -1);
				  if(j % 2) {
					  px[0] = 0x18 + i;
					  px[1] = 0x00;
					  HAL_I2C_Master_Transmit(&hi2c1, AS1130_ADDR, px, 2, -1);
				  }
				  else {
					  px[0] = 0x18 + ((i + 1) % 132);
					  px[1] = 0x80;
					  HAL_I2C_Master_Transmit(&hi2c1, AS1130_ADDR, px, 2, -1);
				  }
			  }
			  HAL_Delay(1);
		  }
	  }

	  if(0) {
		  while(buf_fresh && !uart_buf_fresh) {}; // TEMP

		  HAL_GPIO_TogglePin(LD_GPIO_Port, LD_Pin);
		  int32_t max_tf = 0;
		  int32_t t[3], v[3], tf[3], tbnd[2];
		  uint8_t _converged = 1;
		  if(!inv_kin(A, Ts_[Ts_idx & (NUM_CART_POS - 1)], t, t_prev[0])) {
			  for(uint8_t i = 0; i < 3 && _converged; i++) {
				  uint8_t j = 0;
				  for(int32_t v_ = Ts_[Ts_idx & (NUM_CART_POS - 1)][i + 3]; j < TRAJ_ITER_LIM; v_ /= 2, j++) {
					  if(!traj_t(t_prev[i], t[i], vf_prev[i], v_, tbnd, &JOINT_PHYS[i]) && tbnd[1] > 0) {
						  tf[i] = (1 << _TW) / tbnd[1];
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

		  tick_fin_buf = max_tf * 6 * TIM3_FREQ >> _W; // WATCH: TIM3_FREQ is base units
		  if(_converged) {
			  for(uint8_t i = 0; i < NUM_POS; i++) {
				  for(uint8_t j = 0; j < 3; j++) {
					  int32_t inter = lerp(t_prev[j], t[j], vf_prev[j], v[j], max_tf, (i * max_tf) / (NUM_POS - 1)) * PER_RADS[j]; // WATCH: NUM_POS and PER_RADS is base units
					  pos_dbl_buf[0][i][j] = max(-RNGs[j], min(RNGs[j], inter >> _W)) * RHR_SGNS[j]; // CONVERT
					  // interestingly, the integer arithmetic introduces rounding-like errors, not only floor-like errors (e.g. skips up to 2)
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
  //			  memcpy(&vf_prev, &v, 3 * sizeof(int32_t));
			  buf_fresh = 1;
			  uart_buf_fresh = 0;

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
