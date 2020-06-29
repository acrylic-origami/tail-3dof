/*
 * imath.h
 *
 *  Created on: Mar 30, 2020
 *      Author: derek-lam
 */

#ifndef IMATH_H_
#define IMATH_H_

#include <stdint.h>

#define max(a,b) \
		({ __typeof__ (a) _a = (a); \
		__typeof__ (b) _b = (b); \
		_a > _b ? _a : _b; })

#define min(a,b) \
		({ __typeof__ (a) _a = (a); \
		__typeof__ (b) _b = (b); \
		_a < _b ? _a : _b; })

int8_t isin(int8_t x);
int8_t icos(int8_t x);
int8_t iasin(int8_t x);
int8_t iacos(uint8_t x);
int8_t icsc(int8_t x);
int8_t isec(int8_t x);

#endif /* IMATH_H_ */
