/*
 * traj.c
 *
 *  Created on: Apr 19, 2020
 *      Author: derek-lam
 */

#include <math.h>
#include "traj.h"
#include "consts.h"
#include "math_util.h"

int32_t sgn(int32_t x) {
	return x > 0 ? 1 : -1;
}
static int32_t cmin(int32_t a, int32_t b) {
	if(a < 0)
		return b;
	if(b < 0)
		return a;

	return min(a, b);
}
static int32_t cmax(int32_t a, int32_t b) {
	return max(a, b);
}
static void cbnd(int32_t *a, int32_t *b) {
	a[0] = cmax(a[0], b[0]);
	a[1] = cmin(a[1], b[1]);
}
static uint8_t abs_quad(int32_t a, int32_t b, int32_t z, int32_t *xs) {
	if((abs(a) > abs(z)) && (a > z) == (b > 0))
		return 1;

	if((z >= a) == (b >= 0))
		xs[1] = cmin(xs[1], isqrt(((z - a) << _W) / b) << _HW);

	if((z >= -a) != (b >= 0)) {
		int32_t x_ = isqrt(((z + a) << _W) / -b) << _HW;
		int32_t x__[2] = { -1, x_ };
		if(xs[1] > 0) {
			x__[0] = cmax(xs[1], x_);
		}
		cbnd(xs, x__);
	}
	return 0;
}
static int32_t quad(int32_t a, int32_t b, int32_t c, int32_t x) {
	return ((a*x >> _W)*x >> _W) + (b*x >> _W) + c;
}
static int32_t quadp(int32_t a, int32_t b, int32_t c, int32_t x) {
	return (2*a*x >> _W) + b;
}
static int32_t origin_dist_sq(int32_t a, int32_t b, int32_t c, int32_t x) {
	int32_t y = quad(a, b, c, x);
	int32_t y_hw = (y >> _HW); // pre-adjust because this result could be huge
	return (y_hw * y_hw) + (x*x >> _W);
}
static int32_t origin_dist_sq_p(int32_t a, int32_t b, int32_t c, int32_t x) {
	int32_t q_hw = quad(a, b, c, x) >> _HW;
	int32_t qp_hw = quadp(a, b, c, x) >> _HW;
	return 2 * (q_hw * qp_hw + x);
}
static uint8_t populate(int32_t* vs, int32_t* ts, int32_t* xbnd, joint_phys_t *phys, int32_t t_default) {
	int32_t a = ts[0], b = ts[1];
	for(uint8_t i = 0; i < 2; i++) {
		int32_t c = vs[i * 2], d = vs[i * 2 + 1];
		int32_t xs[2] = { -1, -1 };
		uint8_t corners[2] = { abs(b) < EPS, abs(d) < EPS };
		if(corners[0] && corners[1]) {
			// REALLY degenerate case: a < T0, forall t. fallback to default time (alpha/v)
			if(abs(a) < phys->T0)
				xs[1] = t_default;
			else
				return 1;
		}
		else if(corners[1]) {
			// degenerate case: |a + bx^2| < T0 - abs(c) / SMAX
			int32_t T0_ceil = phys->T0 * ((1 << _W) - (abs(c) << _W) / phys->SMAX) >> _W;
			if(abs_quad(a, b, T0_ceil, xs))
				return 1;
		}
		else if(corners[0]) {
			// instead of doing the |a| < |T0(1-(c-d)/SMAX|, do via quadratic and hijack the machinery that we have already
			// pc[0] = d * d / phys->SMAX / phys->SMAX;
			// int32_t c0 = (1 - c / phys->SMAX);
			// pc[1] = -2 * d * c0 / phys->SMAX;
			// pc[2] = c0 * c0 - a * a / phys->T0 / phys->T0;

			int32_t s = phys->SMAX * ((1 << _W) - (a << _W) / phys->T0) >> _W;
			if(s < 0) { // never intersects
				return 1;
			}
			int32_t xs_[2] = { ((s - c) << _W) / d, (-(s + c) << _W) / d };
//			int32_t xs__[2] = { min(xs_[0], xs_[1]), min(xs_[0], xs_[1]) };
			cbnd(xs, xs_);
		}
		else {
			volatile int32_t k_ = (phys->SMAX << _W) / phys->T0;
			volatile int32_t mid_ = (-c << _W) / d;
			volatile int32_t a_ = ((b * k_ >> _W) * k_ >> _W);
			volatile int32_t b_ = (b * 2 * k_ >> _W) * mid_ >> _W;
			volatile int32_t c_ = ((mid_ * mid_ >> _W) * b >> _W) + a;

			volatile int32_t R2 = phys->T0 * phys->T0 >> _W;

			volatile int32_t x = phys->T0;
			// easiest way... is sadly numerical root finding.
			// pros: easy to make into integer algorithm
			uint8_t i;
			for(i = 0; i < TIM_ITER_LIM; i++) {
				int32_t dist = origin_dist_sq(a_, b_, c_, x);
				int32_t delta = dist - R2;
				if(abs(delta) < TIM_ITER_EPS)
					break;
				x -= delta / (origin_dist_sq_p(a_, b_, c_, x) >> _W); // head away if inside
			}
			if(i == TIM_ITER_LIM)
				return 1;
			xs[1] = (x * k_ >> _W) + mid_;
		}
		cbnd(xbnd, xs);
	}

	return 0;
}
static int32_t smoothstep(int32_t t) {
	return 3 * (t * t >> _W) - 2 * ((t * t >> _W) * t >> _W);
}
uint8_t traj_t(int32_t x0, int32_t x1, int32_t v0, int32_t v1, int32_t *bnd, joint_phys_t *phys) {
	int32_t vm = x1 - x0;
	int32_t vtf[2] = { abs((v0 << _W)/ALPHA), abs((v1 << _W)/ALPHA) };
	int32_t lv[2][2] = {{-176*v0 >> _W, 432*vm >> _W}, {v0, 432*vm >> _W}}; // UNFLOAT 0.6875, 1.6875
	int32_t lt[2] = {(864*((phys->I)*v0 >> _W) >> _W)*vtf[0] >> _W, (864*((phys->I)*vtf[0] >> _W) >> _W)*vm >> _W}; // UNFLOAT 3.375

	int32_t rv[2][2] = {{432*v1 >> _W, -176*vm >> _W}, {432*v1 >> _W, vm}}; // UNFLOAT 0.6875, 1.6875
	int32_t rt[2] = {(864*((phys->I)*v1 >> _W) >> _W)*vtf[1] >> _W, (864*((phys->I)*vtf[1] >> _W) >> _W)*vm >> _W}; // UNFLOAT 3.375
	bnd[0] = -1; bnd[1] = -1;

	return populate(lv, lt, bnd, phys, v0 == 0 ? MAX_T_DEFAULT : (ALPHA << _W) / v0) || populate(rv, rt, bnd, phys, v1 == 0 ? MAX_T_DEFAULT : (ALPHA << _W) / v1);
}
int32_t lerp(int32_t a, int32_t b, int32_t av, int32_t bv, int32_t tf, int32_t t) {
	int32_t av_ = max(abs(av), (1 << _TW) / tf);
	int32_t bv_ = max(abs(bv), (1 << _TW) / tf);

	int32_t l = ((av * t >> _W) + a);
	int32_t m = (b - a) * t / tf + a; // EXEMPT: * followed by /
	int32_t r = ((bv * (t - tf) >> _W) + b);
	if(t < (ALPHA << _W) / av_ && t >= (tf - (ALPHA << _W) / bv_)) {
		int32_t s0 = smoothstep(t * av_ / ALPHA); // EXEMPT: * followed by /
		int32_t s1 = smoothstep((tf - t) * bv_ / ALPHA); // EXEMPT: * followed by /
		int32_t s2 = (t << _W) / tf;
		return (((l * ((1 << _W) - s0) + m * s0) >> _W) * ((1 << _W) - s2) >> _W) + (((r * ((1 << _W) - s1) + m * s1) >> _W) * s2 >> _W); // GROUPED: distributive */+
	}
	else if(t < (ALPHA << _W) / av_) {
		int32_t s = smoothstep(t * av_ / ALPHA); // EXEMPT: * followed by /
		return l * ((1 << _W) - s) + m * s >> _W;
	}
	else if(t >= (ALPHA << _W) / av_ && t < (tf - (ALPHA << _W) / bv_)) {
		return m;
	}
	else {
		int32_t s = smoothstep((tf - t) * bv_ / ALPHA); // EXEMPT: * followed by /
		return r * ((1 << _W) - s) + m * s >> _W;
	}
}
