/*
 * kin.h
 *
 *  Created on: Apr 19, 2020
 *      Author: derek-lam
 */

#ifndef KIN_H_
#define KIN_H_
#include <stdint.h>

// physical dimensions //
#define A1 0.2
#define A2 0.2
#define A3 0.4

void fwd_kin(float *A, float *q, float *r);
float lerp(float a, float b, float av, float bv, float tf, float t);
uint8_t inv_kin(float *A, float *T, float *t);

#endif /* KIN_H_ */
