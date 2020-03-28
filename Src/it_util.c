/*
 * it_util.c
 *
 *  Created on: Mar 26, 2020
 *      Author: derek-lam
 */

#include "it_util.h"

extern uint32_t target_256[3] = {0}, target_v_256[3] = {0};
extern volatile uint32_t cur_256[3] = {0}, cur_v_256[3] = {0};

int32_t poly(int32_t *p_128, uint8_t t) {
	int32_t ret = 0;
	for(uint8_t i = 0; i < 4; i++) {
		int32_t mul = p_128[i];
		for(uint8_t j = 0; j < i; j++) {
			mul *= t;
		}
		ret += mul;
	}
	return ret >> 7;
}

void spline(int32_t a, int32_t av, int32_t b, int32_t bv, uint32_t t, int32_t *p_128, uint8_t _ms = 1) {
	// _ms is a flag for using millisecond units vs. natural units (pi/256)
	// positions in [-pi/2, pi/2] -> [-127, 128]
	// therefore velocities scaled by `256/pi`
	// the [u]int32_t in the args are for being lazy with casting: pass in 8-bit please
	int32_t M[2][2] = {
			{ t * t * 3, t * 2 },
			{ t * t * t, t * t }
	};
	// convert range from [-127, 128] to [100, 200] in position, corresponding speed
	if(_ms) {
		a = BIT2MS(a);
		av = BIT2MS(av);
		b = BIT2MS(b);
		bv = BIT2MS(bv);
	}

	int32_t B[2] = { av - bv, a - b - bv * t };
	int32_t det = M[0][0] * M[1][1] - M[0][1] * M[1][0];

	if(_ms)
		B[1] += TIM2_MS + (TIM2_MS >> 1); // center at 1.5ms

	p_128[0] = ((M[1][1] * B[0] - M[0][1] * B[1]) << 7) / det;
	p_128[1] = ((M[0][0] * B[1] - M[1][0] * B[0]) << 7) / det;
	p_128[2] = bv << 7;
	p_128[3] = b << 7;
}

void motor_tick(void) {
	// exponential acceleration, good enough of an approximation
	uint32_t remain = target_256[0] - cur_256[0];
}
