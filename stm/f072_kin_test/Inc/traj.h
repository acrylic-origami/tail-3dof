/*
 * traj.h
 *
 *  Created on: Apr 19, 2020
 *      Author: derek-lam
 */

#ifndef TRAJ_H_
#define TRAJ_H_
#include <stdint.h>


// servo parameters //
#define ALPHA 2.0

// Hitec D645
#define T0_HS 16.0
#define SMAX_HS 6.16
#define I_HS 8.0
#define CCR_PER_RAD_HS 220
#define CCR_MID_HS 632
#define RNG_HS 288

// DSServo 3218MG
#define T0_DS 20.0
#define SMAX_DS 7.5
#define I_DS 3.0
#define CCR_PER_RAD_DS 252
#define CCR_MID_DS 588
#define RNG_DS 432

// Hobbywing ESC
#define CCR_PER_RAD_S_HW 0.695
#define GEAR_RATIO_HW 51
#define DEADBAND_HW 14
#define CCR_MID_HW 562
//temp
#define MAX_CCR_HW 10
#define P_HW_N 1
#define P_HW_D 3

float sgn(float x);
uint8_t traj_t(float x0, float x1, float v0, float v1, float *bnd);

#endif /* TRAJ_H_ */
