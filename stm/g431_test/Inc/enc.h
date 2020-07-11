/*
 * enc.h
 *
 *  Created on: May 21, 2020
 *      Author: derek-lam
 */

#include <stm32g4xx_hal.h>

#ifndef ENC_H_
#define ENC_H_

int16_t enc(uint8_t *abz);
#define Z_ST 0
#define Z_A_POL 1
#define ENCPOL 1
#define HALL_INIT -26

#endif /* ENC_H_ */
