/*
 * motor.c
 *
 *  Created on: May 22, 2020
 *      Author: derek-lam
 */

#include "enc.h"
#include "main.h"
#include "motor.h"
#include "math_util.h"

/*
Considered the inductor energy dumping to decide about deadtime + complimentary
Assuming 100uH coils and ~1ohm of series resistance, time constant around 600us
Therefore using complimentary dumps most of the current to ground
Plus the diodes can take those couple of amps, for the off-time (~10us) energy
  transfer is small enough (since the diode resistance >> motor coil resistance (although isn't that worse re: i2r?))
anyways, worst thing for power is to short to ground so don't do that
 */
const uint16_t state_en[6] = {
//	0b010000010000, // -+0
//	0b000000010100, // 0+-
//	0b000100000100, // +0-
//	0b000101000000, // +-0
//	0b000001000001, // 0-+
//	0b010000000001, // -0+

	0b010000010000, // -+0
	0b000000010100, // 0+-
	0b000100000100, // +0-
	0b000101000000, // +-0
	0b000001000001, // 0-+
	0b010000000001, // -0+
};
// 451326 (state fwd halls aligned)
// state_en[3] = 4 & 6 hall (fwd = 4 -> 6)
// hall fwd: 326451
#define HALL2STATE_INVALID 0 // use legal values for invalid, juuuust in case
const uint8_t hall2state[8] = {
	HALL2STATE_INVALID,
	3,5,4,1,2,0
	// aligned with backwards offset, oriented forwards
	// to go backward, current hall sensor input as idx
	// to go forward, +1
	,HALL2STATE_INVALID
};
volatile uint32_t* const ccrs[3] = { &(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR3) };
volatile uint32_t* const state_ccr[6][3] = {
	{ &(TIM1->CCR2), &(TIM1->CCR3), &(TIM1->CCR1) },
	{ &(TIM1->CCR2), &(TIM1->CCR1), &(TIM1->CCR3) },
	{ &(TIM1->CCR3), &(TIM1->CCR1), &(TIM1->CCR2) },
	{ &(TIM1->CCR3), &(TIM1->CCR2), &(TIM1->CCR1) },
	{ &(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR3) },
	{ &(TIM1->CCR1), &(TIM1->CCR3), &(TIM1->CCR2) },
};
volatile int32_t ctrl_i = 0, ctrl_p = 0, ctrl_d_buf[NUM_CTRL_D_BUF] = { 0 };
volatile uint8_t ctrl_d_idx = 0;

volatile int16_t pwmin = 0;
extern volatile uint8_t abz;
volatile uint8_t abzs[32] = { 0 };
uint8_t abzs_idx = 0;
volatile int16_t e = 0;
volatile uint8_t state = 0;
volatile uint16_t ccr_ = 0;
volatile uint8_t abz_buf = 0, last_abz = 0;
volatile uint16_t diff = 0;
volatile uint32_t ticks = 0, last_change = 0;
volatile int32_t soft_start_coeff = 0;
void motor_tick(void) {
	ticks++;
	if(pwmin >= MIN_PWMIN_PULSE && pwmin <= MAX_PWMIN_PULSE) {
		soft_start_coeff = min(MAX_SOFT_START_COEFF, soft_start_coeff + 1);
		e = enc(&abz);
		// abzs[(abzs_idx++) & 0b11111] = abz;

		int32_t pwmin_ = max(-PWMIN_LIM, min(PWMIN_LIM, (int32_t)pwmin - ((MAX_PWMIN_PULSE + MIN_PWMIN_PULSE) >> 1)));
		int16_t targ = pwmin_ * PWMIN2TARG_N / PWMIN2TARG_D;
		ctrl_p = targ - e;

		volatile int16_t ctrl =
			(
				ctrl_i * CTRL_I_N / CTRL_I_D
				+ ctrl_p * CTRL_P_N / CTRL_P_D
			)
			* soft_start_coeff / MAX_SOFT_START_COEFF;
		// + ctrl_d * CTRL_D_N / CTRL_D_D;

		if(abz != last_abz) {
			diff = ticks - last_change;
			last_change = ticks;
		}
		if(
			abz_buf == 0 ||
			abz_buf != abz && (
				ctrl > 0 && (
					hall2state[abz] != (hall2state[abz_buf] + 1) % 6 &&
					hall2state[abz] != (hall2state[abz_buf] + 2) % 6
				) ||
				ctrl < 0 && (
					hall2state[abz] != (hall2state[abz_buf] + 5) % 6 &&
					hall2state[abz] != (hall2state[abz_buf] + 4) % 6
				)) ||
				ticks - last_change > diff / 8 // give it time to travel nominally 7.5deg (note equilibrium @15deg)
			) {
			abz_buf = abz;
		}

		if(e * sgn(ctrl) < MAX_TICK) { // opposite signs: taking it away from extreme
			// choose between abz (0-deg timing) and abz_buf (nominal 7.5deg timing)
			state = (hall2state[abz] + (ctrl > 0 ? 1 : 4)) % 6; // ctrl = 0 -> stall
			volatile uint16_t ctrl_ = abs((ctrl * CTRL2CCR_N) / CTRL2CCR_D);
			ctrl_ = min(MAX_CCR, ctrl_); // ((ticks) & 0xFF) * MAX_CCR / 0xFF; // 80; // (pwmin - MIN_PWMIN_PULSE) * PWMIN2TARG_N / PWMIN2TARG_D; //

			// TODO add real calculated baseline based on expected load
			uint16_t nccr = min(MAX_CCR, max(10, ctrl_));
			uint16_t ccer = (state_en[state] & ~TIM1_POL_MSK) | TIM1_POL;

			// <timer critical region>
			*(state_ccr[state][0]) = nccr; // nccr ; // PCCR; // high-side
			*(state_ccr[state][1]) = nccr; // + (TIM1->BDTR & TIM_BDTR_DTG_Msk) + 1; // low-side
			TIM1->CCER = ccer;
			// </timer critical region>
		}
		last_abz = abz;
	}
	else {
		soft_start_coeff = 0;
		TIM1->CCER = BRAKE_CCER;
		TIM1->CCR1 = BRAKE_CCR;
		TIM1->CCR2 = BRAKE_CCR;
		TIM1->CCR3 = BRAKE_CCR;
	}
}
