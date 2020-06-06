/*
 * traj.h
 *
 *  Created on: Apr 19, 2020
 *      Author: derek-lam
 */

#ifndef TRAJ_H_
#define TRAJ_H_
#include <stdint.h>
#include "consts.h"


// servo parameters //
// overshoot scaling
#define ALPHA 512 // UNFLOAT 2.0
#define MAX_T_DEFAULT 0x7FFFFFFF

// Newton's method limit
#define TIM_ITER_LIM 16
#define TIM_ITER_EPS 16384 // UNFLOAT 64

// safety
// n cycles time constant, defines threshold for PI thresholding, tied to filter coeff
#define I_TC_N 5
#define I_TC_D 1
#define I_FILT_COEFF_N 4
#define I_FILT_COEFF_D 5
// scaled against 4A/1024
#define I_MAX_N 512
#define ERROR_TC 5
// TODO need to tune this to the ADC settings
// TODO for the moment: 20deg at 151deg range (spanning 1024)
//#define ERROR_MAX_N 136
#define ERROR_MAX_N 50
#define ERROR_FILT_COEFF_N 4
#define ERROR_FILT_COEFF_D 5

#define ERROR_SCALE_HS_N 9
#define ERROR_SCALE_HS_D 20

// freefall filter for reset
// n cycles time constant
#define FREEFALL_TC_N 36
#define FREEFALL_TC_D 25
#define FREEFALL_FILT_COEFF_N 1
#define FREEFALL_FILT_COEFF_D 2
// allow approx. 10 "degree-time-constants" of error (stricter than instantaneous)
// TC * (thresh / rng * rng_ccr)
// TODO: I don't think this filter will work without the setpoint:
// could trip back into operation by coincidence while it's still moving
// probably want a kind of difference component to make sure it's not moving much anymore
#define FREEFALL_THRESH 28

typedef struct joint_phys_s {
	int32_t T0;
	int32_t SMAX;
	int32_t I;
} joint_phys_t;

#define G_256 2509
#define M_TOT_256 256 // kg

// MOI units are in kg-cm
// Hitec D645
#define T0_HS 4096 // UNFLOAT 16.0
#define SMAX_HS 1577 // UNFLOAT 6.16
#define I_HS 2048 // UNFLOAT 80

#define CCR_PER_RAD_HS 220
#define CCR_MID_HS 600
#define RNG_HS 576
#define SGN_HS -1

#define POT_MID_HS 548
#define POT_RNG_HS 590

// DSServo 3218MG
#define T0_DS 5120 // UNFLOAT 20.0
#define SMAX_DS 1920 // UNFLOAT 7.5
#define I_DS 768 // UNFLOAT 30
#define CCR_PER_RAD_DS 252
#define CCR_MID_DS 588
#define RNG_DS 864
#define SGN_DS 1

#define POT_LOW_DS 0
#define POT_HIGH_DS 1024

// Hobbywing
#define T0_HW 36310 // UNFLOAT 142 // kg-cm
#define SMAX_HW 350 // UNFLOAT 1.37 // rad/s
#define I_HW 2560 // UNFLOAT 50 // kg-cm

// expressed in units of T0's per radian
// regions are right-hand bounds of angle
#define HW_D_N_0 3
#define HW_D_D_0 19
#define HW_P_N_0 8
#define HW_P_D_0 13
#define HW_I_N_0 8
#define HW_I_D_0 35
#define HW_P_REGION_0 (-TICKS_PER_RAD_HW * 4 / 7) // -(pi - 2) / 2 radians

#define HW_D_N_1 3
#define HW_D_D_1 19
#define HW_P_N_1 2
#define HW_P_D_1 29
#define HW_I_N_1 4
#define HW_I_D_1 60
#define HW_P_REGION_1 (TICKS_PER_RAD_HW * 4 / 7) // +(pi - 2) / 2 radians

#define HW_D_N_2 3
#define HW_D_D_2 19
#define HW_P_N_2 7
#define HW_P_D_2 13
#define HW_I_N_2 7
#define HW_I_D_2 32

// ccr/rad/s _unloaded_
#define CCR_PER_RAD_S_HW 4 // UNFLOAT 0.01363
#define RNG_HW (SMAX_HW * CCR_PER_RAD_S_HW * GEAR_RATIO_HW >> _W)
#define S_TRANSITION_HW 100 // units of rad-s-256
#define TICKS_PER_RAD_HW 33
#define GEAR_RATIO_HW 51
#define DEADBAND_HW 14
#define CCR_MID_HW 562
#define SGN_HW 1
//temp
#define MAX_CCR_HW 30
#define P_HW_N 1
#define P_HW_D 3

int32_t sgn(int32_t x);
uint8_t traj_t(int32_t x0, int32_t x1, int32_t v0, int32_t v1, int32_t *bnd, joint_phys_t *phys);

#endif /* TRAJ_H_ */
