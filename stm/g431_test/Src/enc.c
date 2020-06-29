/*
 * enc.c
 *
 *  Created on: May 21, 2020
 *      Author: derek-lam
 */

#include "enc.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;
int16_t enc(uint8_t *abz) {
//	uint8_t a = HAL_GPIO_ReadPin(A_GPIO_Port, A_Pin);
//	uint8_t b = HAL_GPIO_ReadPin(B_GPIO_Port, B_Pin);
	// TEMP
	uint8_t z = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
	uint8_t b = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
	// TODO hardware: need to connect the B on the hall interface with the button (currently PC10 coupled to A)
	uint8_t a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
//	uint8_t z = HAL_GPIO_ReadPin(Z_GPIO_Port, Z_Pin);

	*abz = (a << 2) | (b << 1) | z;

	int16_t t0 = __HAL_TIM_GET_COUNTER(&htim4);
	t0 += t0 >> 1; // MUST use right-shift for symmetry about 0
				   // off by at most 1, don't worry too much about it
	               // plus depends on encoder position at startup
	               // possible to compensate if it really is critical

	// NOTE: two vars below depend on the way hall sensors are connected from TIM2 to TIM3
	if((a == z) == Z_ST) {
		return t0 + ((Z_A_POL == a) == b);
	}
	else {
		return t0;
	}
}
