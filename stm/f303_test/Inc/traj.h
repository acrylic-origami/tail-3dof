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
#define ALPHA 1024 // UNFLOAT 4.0
#define MAX_T_DEFAULT 0x7FFFFFFF

// Newton's method limit
#define TIM_ITER_LIM 16
#define TIM_ITER_EPS 16384 // UNFLOAT 64

//// safety
//// n cycles time constant, defines threshold for PI thresholding, tied to filter coeff
//#define I_TC_N 5
//#define I_TC_D 1
//#define I_FILT_COEFF_N 4
//#define I_FILT_COEFF_D 5
//// scaled against 4A/1024
//#define I_MAX_N 512

//// ERROR CONTROL //
//#define ERROR_TC 5
//// TODO need to tune this to the ADC settings
//// TODO for the moment: 20deg at 151deg range (spanning 1024)
////#define ERROR_MAX_N 136
//#define ERROR_MAX_N 50
//#define ERROR_FILT_COEFF_N 4
//#define ERROR_FILT_COEFF_D 5
//
//#define ERROR_SCALE_HS_N 9
//#define ERROR_SCALE_HS_D 20

//// FREEFALL CTRL //
//// freefall filter for reset
//// n cycles time constant
//#define FREEFALL_TC_N 36
//#define FREEFALL_TC_D 25
//#define FREEFALL_FILT_COEFF_N 1
//#define FREEFALL_FILT_COEFF_D 2
//// allow approx. 10 "degree-time-constants" of error (stricter than instantaneous)
//// TC * (thresh / rng * rng_ccr)
//// TODO: I don't think this filter will work without the setpoint:
//// could trip back into operation by coincidence while it's still moving
//// probably want a kind of difference component to make sure it's not moving much anymore
//#define FREEFALL_THRESH 28

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
#define I_HS 10240 // UNFLOAT 40

#define CCR_PER_RAD_HS 252
#define CCR_MID_HS 562
#define RNG_HS 628
#define SGN_HS -1

#define POT_MID_HS 548
#define POT_RNG_HS 590

// DSServo 3218MG
#define T0_DS 5120 // UNFLOAT 20.0
#define SMAX_DS 1920 // UNFLOAT 7.5
#define I_DS 7680 // UNFLOAT 30
#define CCR_PER_RAD_DS 237
//#define CCR_MID_DS 560
#define CCR_MID_DS 690
#define RNG_DS 808
// #define RNG_DS 550
#define SGN_DS 1

#define POT_LOW_DS 0
#define POT_HIGH_DS 1024

// Hobbywing
#define T0_HW 31481 // UNFLOAT 122.9
#define SMAX_HW 350 // UNFLOAT 7.5
#define I_HW 12800 // UNFLOAT 50
#define CCR_PER_RAD_HW 127
#define CCR_MID_HW 627
//#define CCR_MID_HW 550
#define RNG_HW 400
#define SGN_HW 1
#define HW_FREEFALL_CCR 100 // << 900us

//temp
#define MAX_CCR_HW 30
#define P_HW_N 1
#define P_HW_D 3

int32_t sgn(int32_t x);
uint8_t traj_t(int32_t x0, int32_t x1, int32_t v0, int32_t v1, int32_t *bnd, const joint_phys_t *phys);
void lerp(int32_t a, int32_t b, int32_t av, int32_t bv, int32_t tf, int32_t t, int16_t *tr);

#define RAND_FILT_BETA 250 // UNFLOAT 0.996
#define RAND_FILT_VAR_MULT 32678 // UNFLOAT atan(pi/ln(BETA))/ln(BETA)/pi = 24.648
#define RAND_FILT_STDDEV_MULT 1270 // UNFLOAT sqrt(RAND_FILT_VAR_MULT)
int32_t rand_lpf(int32_t prev);

#endif /* TRAJ_H_ */
