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
#define A1 50 // UNFLOAT 0.195
#define A2 50 // UNFLOAT 0.195
#define A3 60 // UNFLOAT 0.235

void fwd_kin(int32_t *A, int32_t *q, int32_t *r);
uint8_t inv_jacob(int32_t *A, int32_t *q, int32_t *tb, float *qb);
uint8_t inv_kin(int32_t *A, int32_t *T, int32_t *t);

#endif /* KIN_H_ */
