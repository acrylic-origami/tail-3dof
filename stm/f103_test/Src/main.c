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
#include <stdlib.h>
#include "main.h"
#include "adc.h"
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
#include "imath.h"
#include <math.h>
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
volatile global_state_t __STATE = GLOBAL_RESET;

typedef struct hand_ctrl_cfg_s {
	int16_t rng[2];
	uint16_t lims[2];
	int8_t sgn;
} hand_ctrl_cfg_t;
volatile uint16_t hand_ctrl_buf[NUM_HAND_CTRL_BUF * NUM_HAND_CTRL_AX] = { 0 };
uint16_t hand_ctrl_buf_2[NUM_HAND_CTRL_BUF * NUM_HAND_CTRL_AX] = { 0 };
const hand_ctrl_cfg_t HAND_CTRL_RNGS[NUM_HAND_CTRL_AX] = {
	{ .rng = { -1500, 5596 }, .lims = { 509, 3600 }, .sgn = 1 },
	{ .rng = { 0, 3198 }, .lims = { 0, 1600 }, .sgn = -1 },
	{ .rng = { 1272, 3600 }, .lims = { 1272, 3600 }, .sgn = 1 },
};
// map the ADC via channel order to XYZ

//volatile uint8_t adc_ch = 0, adc_buf_idx = 0;
int16_t pos_dbl_buf[2][NUM_POS_ELE] = { 0 };
volatile int16_t pos[NUM_POS_ELE] = { 0 };
volatile uint8_t _buf_fresh = 0, _con_fresh = 0;
volatile uint16_t tick_fin_buf = 0, tick_fin = 0, tick_start_buf = 0, tick_cur = 1; // start with consumer hungry
int32_t A[4] = { 0, A1, A2, A3 };

extern volatile uint8_t mcp1130_ch;
extern volatile uint16_t mcp1130_mosi;
extern volatile uint16_t I_safety_buf, ERROR_safety_buf;

extern int32_t D_COEF[NUM_HW_D_COEF];
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef huart3;

const joint_phys_t JOINT_PHYS[3] = {
		{ T0_HW, SMAX_HW, I_HW },
		{ T0_HS, SMAX_HS, I_HS },
		{ T0_DS, SMAX_DS, I_DS },
};
const uint16_t MIDs[3] = { CCR_MID_HW, CCR_MID_HS, CCR_MID_DS };
const uint16_t RNGs[3] = { RNG_HW / 2, RNG_HS / 2, RNG_DS / 2 };
const uint16_t PER_RADS[3] = { CCR_PER_RAD_HW, CCR_PER_RAD_HS, CCR_PER_RAD_DS };
int32_t vf_prev[3] = { 0 }; // previous final velocity
int32_t t_prev[3] = { 0 }; // previous target angles (after inv kin)
const int8_t RHR_SGNS[3] = { SGN_HW, SGN_HS, SGN_DS };

// bitmaps
extern const uint8_t bump[NBUMP][NBUMP];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t pos_idx(void);
void mcp1130_txrx(void);
static void adc_rx();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int32_t hand_ctrl_norm[2][NUM_HAND_CTRL_AX] = { 0 };
int16_t j0_ccr_adjust = -RNG_HW / 2;

// TEMP
uint8_t px[2] = { 0 };
const uint8_t k[2] = { 0xFD, 0x40 };
uint8_t uart_buf_in[UART_BUF_SIZE] = { 0 };
volatile uint32_t uart_buf[UART_BUF_SIZE] = { 0 };
volatile int8_t uart_buf_fresh = 0;
int16_t hand_ctrl_dpack[4] = {0};

#define NUM_WAYPOINTS 19
int32_t waypoints[NUM_WAYPOINTS][NUM_JOINTS] = {
	{ 322, 0, 121 },
	{ 241, 0, 0 },
	{ 322, 0, 121 },
	{ -322, 0, -121 },
	{ -241, 0, 0 },
	{ -322, 0, -121 },
	{ 0, -241, -80 },
	{ 0, 241, 80 },
	{ 0, -241, -80 },
	{ 0, 241, 80 },
	{ -241, -241, -80 },
	{ 241, 0, 121 },
	{ -241, 241, -80 },
	{ -241, -241, -80 },
	{ -121, 241, -40 },
	{ 0, -241, 0 },
	{ 121, 241, 40 },
	{ 241, -241, 80 },
	{ -120, 0, 0 }
};
uint32_t waypoint_idx = 0;
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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

//  while(HAL_GPIO_ReadPin(LIMSW1_GPIO_Port, LIMSW1_Pin)); // wait for bottom limit switch trigger

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim4);

  AS1130_Init();

  htim2.Instance->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
  htim3.Instance->CCER |= TIM_CCER_CC2E;

  // TODO softer startup routine by reading from servo pot ADC and slow-driving them to center
  // TEMP
  HAL_GPIO_WritePin(SERVO1_GPIO_Port, SERVO1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO2_GPIO_Port, SERVO2_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(SERVO1_NSEL_GPIO_Port, SERVO1_NSEL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO2_NSEL_GPIO_Port, SERVO2_NSEL_Pin, GPIO_PIN_RESET);

  uint16_t R0 = A[1] + A[2] + A[3];

  mcp1130_txrx();
  uart_rx();
  adc_rx();

  ////////////
  // HOMING //
  ////////////

  __STATE = GLOBAL_HOMING;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MIDs[1] - RNGs[1]);
  while(1);
  for(; 0 && __STATE == GLOBAL_HOMING && HAL_GPIO_ReadPin(LIMSW1_GPIO_Port, LIMSW1_Pin); j0_ccr_adjust = max(MIN_HOMING_CCR, j0_ccr_adjust - HOMING_STRIDE)) {
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, MIDs[0] + j0_ccr_adjust);
	  HAL_Delay(HOMING_TICK);
  }
  __STATE = GLOBAL_RUN;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int16_t pos_stash[POS_STRIDE] = { 0 };
  for(uint32_t iter = 0; ; iter++)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  {
		  uint16_t tick = pos_idx();
		  uint16_t loc[2] = {
			(pos[tick * POS_STRIDE + 1 * NUM_POS_DERIV] * PER_RADS[1] / RNGs[1] / 2) + 127,
			-(pos[tick * POS_STRIDE + 0 * NUM_POS_DERIV] * PER_RADS[0] / RNGs[0] / 2) + 127,
		  };
		  uint8_t *square = bump; // { 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, };
		  uint16_t dims[2] = { NBUMP, NBUMP }; // { 3, 3 };
		  uint16_t scale[2] = {
			32, // + abs(pos[tick * POS_STRIDE + 0 * NUM_POS_DERIV] - pos[max(0, tick - 1) * POS_STRIDE + 0 * NUM_POS_DERIV]) * 512 * 6 / NBUMP,
			32, // + abs(pos[tick * POS_STRIDE + 1 * NUM_POS_DERIV] - pos[max(0, tick - 1) * POS_STRIDE + 1 * NUM_POS_DERIV]) * 512 * 6 / NBUMP
		  };
		  AS1130_blit(square, dims, loc, scale);
	  }

	  {
//		  while(_buf_fresh) {
//			  // low-priority actions
//
//			  // total YZ moment of inertia calculation
//			  int32_t t_cur[3];
//			  fwd_kin(A, pos[NUM_POS], t_cur);
////			  I_YZ = isqrt(t_cur[1] * t_cur[1] + t_cur[2] * t_cur[2]) / 2 * M_TOT_256;
//		  };

		  int32_t max_tf = 0;
		  int32_t xyz[3] = { 0 }, xyzd[3] = { 0 }, *t, v[3] = { 0 }, tf[3] = { 0 }, tbnd[2] = { 0 }, tp[3] = { 0 };
		  float tp_[3];
		  uint8_t _converged = 0, _hand_ctrl_moved = 0;

		  ///////////////////////////////
		  // CONVERSIONS FOR HAND CTRL //
		  ///////////////////////////////

		  // calculate hand controller speeds
		  uint8_t hand_ctrl_i0;
		  {
			  size_t tf_size = NUM_HAND_CTRL_BUF * NUM_HAND_CTRL_AX * sizeof(hand_ctrl_buf[0]);
//			  __disable_irq();
			  // sensitive region copying buffer
			  // consider disabling interrupts
			  // although this should happen much faster than values change on the controller
			  // that's what's really important
			  hand_ctrl_i0 = hdma_adc1.Instance->CNDTR;
			  memcpy(hand_ctrl_buf_2, hand_ctrl_buf, tf_size);
//			  __enable_irq();
		  }

		  hand_ctrl_i0 /= NUM_HAND_CTRL_AX; // watch for round-down division
		  hand_ctrl_i0 = (NUM_HAND_CTRL_BUF - hand_ctrl_i0 + 2) % NUM_HAND_CTRL_BUF; // (NUM_HAND_CTRL_BUF - NUM_HW_D_COEF - 1)
		  int32_t hand_ctrl_d_buf[NUM_HW_D_COEF][NUM_HAND_CTRL_AX] = { 0 },
				  hand_ctrl_d[NUM_HAND_CTRL_AX] = { 0 }; // 256-units
		  {
			  uint16_t stride = NUM_HAND_CTRL_BUF / NUM_HW_D_COEF / HAND_CTRL_BUF_DS - 1, // -1 to ensure we always miss at least the one potentially incomplete sample currently being DMA'd
					  n_iter = stride * NUM_HW_D_COEF;
			  // downsample by `stride` via averaging
			  // plus hard downsampling by HAND_CTRL_BUF_DS
			  for(uint16_t i = 0; i < n_iter; i++) {
				  for(uint16_t k = 0; k < NUM_HAND_CTRL_AX; k++) {
					  hand_ctrl_d_buf[i / stride][k] +=
							  max(HAND_CTRL_RNGS[k].lims[0],
									  min(HAND_CTRL_RNGS[k].lims[1],
											  hand_ctrl_buf_2[((hand_ctrl_i0 + i * HAND_CTRL_BUF_DS) & (NUM_HAND_CTRL_BUF - 1)) * NUM_HAND_CTRL_AX + k]
									  )
							  );
				  }
			  }
			  for(uint16_t i = 0; i < NUM_HW_D_COEF; i++) {
				  for(uint16_t k = 0; k < NUM_HAND_CTRL_AX; k++) {
					  hand_ctrl_d_buf[i][k] /= stride;
				  }
			  }

			  // perform derivative
			  if(0) {
				  const uint16_t HALF_ADC_BITS = 9;
				  for(uint16_t i = 0; i < NUM_HW_D_COEF; i++) {
					  for(uint16_t k = 0; k < NUM_HAND_CTRL_AX; k++)
						  hand_ctrl_d[k] +=
								  ((hand_ctrl_d_buf[i][k] * D_COEF[i]) >> HALF_ADC_BITS) // cut down dynamic range a bit
								  * R0 / (stride * HAND_CTRL_BUF_DS) * ADC_FREQ / NUM_HAND_CTRL_AX
								  / ((HAND_CTRL_RNGS[k].rng[1] - HAND_CTRL_RNGS[k].rng[0] >> 1) >> HALF_ADC_BITS)
								  * HAND_CTRL_RNGS[k].sgn
								  >> _W; // convert to 256-m/s
				  }
			  }

			  // average previously subsampled values
			  memset(&hand_ctrl_norm[1][0], 0, NUM_HAND_CTRL_AX * sizeof(hand_ctrl_norm[1][0]));
			  for(uint16_t i = 0; i < NUM_HW_D_COEF; i++) {
				  for(uint16_t k = 0; k < NUM_HAND_CTRL_AX; k++)
					  hand_ctrl_norm[1][k] +=
							  ((hand_ctrl_d_buf[i][k] - ((HAND_CTRL_RNGS[k].rng[1] + HAND_CTRL_RNGS[k].rng[0]) >> 1)) << _W)
							  / (HAND_CTRL_RNGS[k].rng[1] - HAND_CTRL_RNGS[k].rng[0] >> 1)
							  * HAND_CTRL_RNGS[k].sgn;
			  }
			  for(uint16_t k = 0; k < NUM_HAND_CTRL_AX; k++) {
				  hand_ctrl_norm[1][k] /= NUM_HW_D_COEF; // convert to 256-norm (+/-1) centered around 0
			  }

			  for(uint8_t i = 0; i < NUM_HAND_CTRL_AX; i++) {
				  if(abs(hand_ctrl_norm[0][i] - hand_ctrl_norm[1][i]) > HAND_CTRL_DEADBAND) {
					  memcpy(&hand_ctrl_norm[0][0], &hand_ctrl_norm[1][0], NUM_HAND_CTRL_AX * sizeof(hand_ctrl_norm[1][0]));
					  _hand_ctrl_moved = 1;
					  // itrig functions domain 0->256 => 0->2pi, range -128->127 => -1->1
					  // controller 0 1 2 -> yaw radius pitch
					  int32_t c0 = icos(hand_ctrl_norm[0][2] >> 2), c1 = icos(hand_ctrl_norm[0][0] >> 2),
							  s0 = isin(hand_ctrl_norm[0][2] >> 2), s1 = isin(hand_ctrl_norm[0][0] >> 2);
					  int32_t R = (hand_ctrl_norm[0][1] * (HAND_R1 - HAND_R0) >> _W) + HAND_R0;
					  xyz[0] = (c0 * s1 >> (_W - 2)) * R >> _W;
					  xyz[1] = (c0 * c1 >> (_W - 2)) * R >> _W;
					  xyz[2] = (s0 << 1) * R >> _W;

					  int32_t *td = hand_ctrl_d;
					  xyzd[0] = (((-s0 * s1 * td[2] + c0 * c1 * td[0]) >> (_TW - 2)) * R + xyz[0] * td[1]) >> _W; // dx/dt
					  xyzd[1] = (-((s0 * c1 * td[2] + c0 * s1 * td[0]) >> (_TW - 2)) * R + xyz[1] * td[1]) >> _W; // dy/dt
					  xyzd[2] = ((c0 * td[2] >> (_W - 1)) * R + xyz[2] * td[1]) >> _W; // dz/dt

					  break;
				  }
			  }
		  }

		  /////////////////////////////////////////////////////
		  // SOLVING TARGET POSITION & SPEED VIA INVERSE KIN //
		  /////////////////////////////////////////////////////
		  memset(uart_buf, 0, UART_BUF_SIZE * sizeof(uart_buf[0]));
		  int16_t tick_fin_stash, tick_cur_stash;
		  {
			  __disable_irq();
			  _con_fresh = 0;
			  tick_fin_stash = tick_fin;
			  tick_cur_stash = tick_cur;
			  uint16_t tick = pos_idx();
			  memcpy(pos_stash, &pos[tick * POS_STRIDE], POS_STRIDE * sizeof(pos[0]));
			  __enable_irq();
		  }
		  int32_t t_[6] = { 0 };
		  if(0 && HAL_GPIO_ReadPin(HAND_CTRL_EN_N_GPIO_Port, HAND_CTRL_EN_N_Pin)) {
			  if(_hand_ctrl_moved && !inv_kin(A, xyz, t_)) { // (!inv_kin(A, Ts_[Ts_idx & (NUM_CART_POS - 1)], t, t_prev[0]) && !inv_jacob(A, t, &Ts_[Ts_idx & (NUM_CART_POS - 1)][3], tp_)) { // uart_buf, uart_buf[3]
				  uint8_t _unreachable[2] = { 0 };
				  for(uint8_t i = 0; i < 2; i++) {
					  // iterate over solutions
					  for(uint8_t j = 0; j < NUM_JOINTS; j++) {
						  if(abs(t_[i * NUM_JOINTS + j] * PER_RADS[j] >> _W) > RNGs[j]) {
							  _unreachable[i] = j + 1;
							  break;
						  }
					  }
				  }
				  switch((_unreachable[1] == 0) << 1 | (_unreachable[0] == 0)) {
				  case 0:
					  uart_buf[UART_BUF_SIZE - 1] = cantor_pair(_unreachable[0], _unreachable[1]);
					  break;
				  case 0b01:
					  t = &t_[0];
					  break;
				  case 0b10:
					  t = &t_[NUM_JOINTS];
					  break;
				  case 0b11:
					  if(abs(t_[0] - pos_stash[0]) < abs(t_[NUM_JOINTS] - pos_stash[0]))
						  t = &t_[0];
					  else
						  t = &t_[NUM_JOINTS];
					  break;
				  }
				  if((_unreachable[0] == 0 && _unreachable[1] == 0) && !inv_jacob(A, t, xyzd, tp_)) {
					  _converged = 1;
					  for(uint8_t i = 0; i < 3; i++) {
						  tp[i] = (int32_t)(tp_[i] * (1 << _W));
						  uart_buf[i] = tp[i];
					  }
				  }
			  }
			  else {
				  uart_buf[UART_BUF_SIZE - 1] = 1;
			  }
			  if(_hand_ctrl_moved) {
				  for(uint8_t i = 0; i < 3; i++) {
					  uart_buf[i + 6] = xyz[i];
				  }
	//			  uart_buf[7] = tick_cur_stash;
	//			  uart_buf[8] = tick_fin_stash;
				  HAL_UART_Transmit_IT(&huart3, &uart_buf[UART_BUF_SIZE - 4], 4 * sizeof(uart_buf[0]));
			  }
		  }
		  else {
//			  HAL_Delay(rand() >> 21);
//			  for(uint8_t i = 0; i < NUM_JOINTS; i++) {
//				  t_[i] = (((rand() - (RAND_MAX >> 1)) >> (31 - _W)) * I_PI) >> _W; // rand_lpf((int32_t)pos_stash[i*NUM_POS_DERIV] << (15 - _W)) >> (15 - _W); // push up to 15-bit to match rand()
//				  uart_buf[i] = t_[i];
//			  }
//			  t = &t_[0];
//			  _converged = 1;
//			  HAL_UART_Transmit_IT(&huart3, &uart_buf[0], 3 * sizeof(uart_buf[0]));

			  if(tick_cur >= tick_fin && !_buf_fresh) {
				  t = &waypoints[(waypoint_idx++) % NUM_WAYPOINTS][0];
				  _converged = 1;
			  }
		  }

		  for(uint8_t i = 0; i < 3 && _converged; i++) {
			  uint8_t j = 0;
			  for(
					  int32_t va_ = pos_stash[i * NUM_POS_DERIV + 1], //  * (int32_t)tick_fin_buf / TIM2_FREQ
					  	  	  vb_ = tp[i]
							   ; j < TRAJ_ITER_LIM
							   ; va_ /= 2, vb_ /= 2, j++
			  ) {
				  if(!traj_t(pos_stash[i * NUM_POS_DERIV], t[i], va_, vb_, tbnd, &JOINT_PHYS[i]) && tbnd[1] > 0) {
					  tf[i] = (1 << _TW) / tbnd[1];
					  v[i] = vb_;
					  //						  if(tf_ > 2 * MAX_TF)
					  //							  _converged = 0;
					  hand_ctrl_dpack[3] = 0;
					  for(uint8_t i = 0; i < 3; i++) {
						  if(tf[i] > max_tf)
							  max_tf = tf[i];
					  }
					  break;
				  }
			  }
			  if(j == TRAJ_ITER_LIM) {
				  _converged = 0;
				  uart_buf[UART_BUF_SIZE - 1] = 2;
				  break;
			  }
		  }

		  /////////////
		  // LERPING //
		  /////////////

		  if(_converged) {
			  tick_fin_buf = (max_tf * TIM2_FREQ * SPEEDUP_FACTOR_D / SPEEDUP_FACTOR_N) >> _W; // WATCH: TIM3_FREQ is base units
			  tick_fin_buf = max(1, tick_fin_buf);
			  //				  memcpy(uart_buf, t, 3 * sizeof(t[0]));
			  //				  uart_buf[3] = (hand_ctrl_norm[0][1] << 16) | (hand_ctrl_norm[0][0] & 0xFFFF);
			  //				  uart_buf[4] = (max_tf << 16) | (hand_ctrl_norm[0][2] & 0xFFFF);
			  //			  memcpy(&uart_buf[3], hand_ctrl_norm, 3 * sizeof(hand_ctrl_norm[0]));
			  //			  HAL_UART_Transmit_IT(&huart3, uart_buf, 4 * 5); // sizeof(uart_buf[0]));

			  for(uint8_t i = 0; i < NUM_POS; i++) {
				  for(uint8_t j = 0; j < 3; j++) {
					  lerp(pos_stash[j * NUM_POS_DERIV], t[j], pos_stash[j * NUM_POS_DERIV + 1], v[j], max_tf, (i * max_tf) / (NUM_POS - 1), &pos_dbl_buf[0][i * POS_STRIDE + j * NUM_POS_DERIV]); // WATCH: NUM_POS and PER_RADS is natural units
					  // interestingly, the integer arithmetic introduces rounding-like errors, not only floor-like errors (e.g. skips up to 2)
				  }
			  }
			  //		  for(uint8_t i = 0
			  //		    ; i < 32 && HAL_DMA_Start(&hdma_memtomem_dma1_channel1, &pos_dbl_buf[0][0], &pos_dbl_buf[1][0], NUM_POS_ELE) != HAL_OK
			  //		    ; i++) {}
			  _buf_fresh = 0;
			  while(hdma_memtomem_dma1_channel2.Instance->CNDTR); // wait for DMA to finish transferring buffer
			  uint16_t next_tick_start;
			  if(_con_fresh) {
				  // i.e. it took a fresh buffer in the time it took to calculate the next result
				  // by smoothness and speed, the linear approximation says we're okay to jump by the number of ticks elapsed
				  next_tick_start = tick_cur - tick_start_buf;
			  }
			  else {
				  next_tick_start = tick_cur - tick_cur_stash;
			  }
			  if(next_tick_start < tick_fin_buf) {
//				  uart_buf[0] =
//				  HAL_UART_Transmit_IT(&huart3, uart_buf, sizeof(uart_buf[0])); // UART_BUF_SIZE *
				  // possible that the timer might have taken the recent buffer and outstripped the brand new one we made
				  // in that case, trash this conversion
				  memcpy(&pos_dbl_buf[1][0], &pos_dbl_buf[0][0], sizeof(pos_dbl_buf[0][0]) * NUM_POS_ELE);
				  tick_start_buf = next_tick_start;
				  _buf_fresh = 1;
			  }
			  _con_fresh = 0;

			  //			  memcpy(&vf_prev, &v, 3 * sizeof(int32_t));
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void mcp1130_txrx() {
	HAL_GPIO_WritePin(SERVO2_NSEL_GPIO_Port, SERVO2_NSEL_Pin, GPIO_PIN_SET);
	asm("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;"); // try to guarantee at least 330ns of hold time
	HAL_GPIO_WritePin(SERVO2_NSEL_GPIO_Port, SERVO2_NSEL_Pin, GPIO_PIN_RESET);
	asm("nop;nop;nop;nop;nop;"); // try to guarantee at least 100/0.96 ns of hold time
	mcp1130_mosi = MCP3002_CONFIG | (mcp1130_ch << MCP3002_ODD_SIGN_Pos);
	volatile uint16_t *rx = mcp1130_ch ? &ERROR_safety_buf : &I_safety_buf;
	HAL_SPI_TransmitReceive_IT(&hspi2, &mcp1130_mosi, rx, 1);
}
void adc_rx(void) {
	HAL_ADC_Start_DMA(&hadc1, hand_ctrl_buf, NUM_HAND_CTRL_BUF * NUM_HAND_CTRL_AX);
}
uint16_t pos_idx(void) {
	return min(NUM_POS - 1, (tick_cur * (NUM_POS - 1)) / tick_fin);
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
	while(1);
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
