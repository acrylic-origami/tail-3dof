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
const int16_t MIC_FILT[2][MIC_FILT_SIZE] = {
		{4088,-2779,-186,2679,-3966,1923,785,-1280,3632,-1176,-1848,-854,-2771,1276,3103,2588,1338,-2205,-3957,-2928,195,3238,3930,1791,-1546,-3674,-3241,-854,1590,2475,1618,158,-534,73,1482,2791,3485,3669,3730,3871,3996,3951,3739,3423,3017,2534,1987,1394,770,135,-492,-1095,-1658,-2163,-2599,-2954,-3220,-3393,-3470,-3452,-3343,-3150,-2881,-2548,-2164,-1741,-1295,-840,-390,42,444,805,1118,1376,1575,1712,1789,1808,1773,1690,1566,1409,1227,1030,826,622,427,246,84,-54,-167,-254,-315,-350,-364,-358,-336,-303,-261,-216,-171,-128,-90,-59,-34,-17,-7,-1,0,0},
		{0,2937,-3848,2621,195,-3188,2420,-1940,371,3550,-564,623,-1789,-3719,-570,1162,3363,3357,360,-2752,-4084,-2495,832,3425,3412,905,-2217,-3855,-3252,-1374,72,31,-1238,-2622,-3170,-2746,-1876,-1118,-614,-167,402,1050,1667,2238,2747,3182,3528,3779,3927,3969,3905,3740,3480,3134,2714,2234,1709,1155,591,31,-506,-1009,-1462,-1855,-2179,-2427,-2596,-2686,-2697,-2634,-2503,-2313,-2073,-1794,-1487,-1164,-837,-516,-211,69,318,530,702,832,919,966,975,951,898,823,730,627,520,412,311,218,137,70,17,-19,-43,-55,-58,-52,-42,-30,-18,-8,-2,0}
};
volatile uint32_t mic_buffer[MIC_BUFFER_SIZE] = {0};
extern volatile uint8_t mic_fresh;
extern DMA_HandleTypeDef hdma_spi1_rx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  htim3.Instance->CR1 |= TIM_CR1_CEN;
  htim3.Instance->CCER |= TIM_CCER_CC1E;
  htim3.Instance->BDTR |= TIM_BDTR_MOE;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 32);
  __HAL_TIM_SET_COUNTER(&htim3, 24);
  htim3.Instance->EGR |= TIM_EGR_UG;
  HAL_SPI_Receive_DMA(&hspi1, mic_buffer, MIC_BUFFER_SIZE << 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if(hspi1.State == HAL_SPI_STATE_READY)
//		  HAL_SPI_Receive_DMA(&hspi1, mic_buffer, MIC_BUFFER_SIZE << 2);
	  if(mic_fresh) {
		  uint8_t c0 = (__HAL_DMA_GET_COUNTER(&hdma_spi1_rx) >> 2);
		  uint32_t E = 0;
		  for(
			  uint8_t i = ((c0 - 1) | 1) & MIC_BUFFER_MSK
			        , j = MIC_FILT_SIZE;
			  i != c0
				  && i > c0 != i < MIC_BUFFER_SIZE - (__HAL_DMA_GET_COUNTER(&hdma_spi1_rx) >> 2)
			  	  && j > 0;
			  i = (i - 2) & MIC_BUFFER_MSK
			, j--
		  ) {
			  E += ((mic_buffer[i] << 1) >> 12) * MIC_FILT[0][j];
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
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
