/*
 * it_util.h
 *
 *  Created on: Mar 26, 2020
 *      Author: derek-lam
 */

#ifndef IT_UTIL_H_
#define IT_UTIL_H_

#define BIT2MS(a_256) ((a_256 * TIM2_MS) >> 8)

uint16_t poly(uint16_t *p, uint16_t t);
void motor_tick(void);

#endif /* IT_UTIL_H_ */
