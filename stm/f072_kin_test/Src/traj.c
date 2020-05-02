/*
 * traj.c
 *
 *  Created on: Apr 19, 2020
 *      Author: derek-lam
 */

#include <math.h>
#include "traj.h"
#include "consts.h"

float sgn(float x) {
	return x > 0 ? 1 : -1;
}
static float cmin(float a, float b) {
	if(a < 0)
		return b;
	if(b < 0)
		return a;

	return fmin(a, b);
}
static float cmax(float a, float b) {
	return fmax(a, b);
}
static void cbnd(float *a, float *b) {
	a[0] = cmax(a[0], b[0]);
	a[1] = cmin(a[1], b[1]);
}
static uint8_t abs_quad(float a, float b, float z, float *xs) {
	if((fabs(a) > fabs(z)) && (a > z) == (b > 0))
		return 1;

	if((z >= a) == (b >= 0))
		xs[1] = cmin(xs[1], sqrt((z - a) / b));

	if((z >= -a) != (b >= 0)) {
		float x_ = sqrt((z + a) / -b);
		float x__[2] = { -1, x_ };
		if(xs[1] > 0) {
			x__[0] = cmax(xs[1], x_);
		}
		cbnd(xs, x__);
	}
	return 0;
}
static float quad(float a, float b, float c, float x) {
	return a*x*x + b*x + c;
}
static float quadp(float a, float b, float c, float x) {
	return 2*a*x + b;
}
static float origin_dist_sq(float a, float b, float c, float x) {
	float y = quad(a, b, c, x);
	return y*y + x*x;
}
static float origin_dist_sq_p(float a, float b, float c, float x) {
	return 2 * (quad(a, b, c, x) * quadp(a, b, c, x) + x);
}
static uint8_t populate(float* vs, float* ts, float* xbnd, joint_phys_t *phys, float t_default) {
	float a = ts[0], b = ts[1];
	for(uint8_t i = 0; i < 2; i++) {
		float c = vs[i * 2], d = vs[i * 2 + 1];
		int8_t A[2] = { 1, -1 };
		float xs[2] = { -1, -1 };
		uint8_t corners[2] = { fabs(b) < EPS, fabs(d) < EPS };
		if(corners[0] && corners[1]) {
			// REALLY degenerate case: a < T0, forall t. fallback to default time (alpha/v)
			if(fabs(a) < phys->T0)
				xs[1] = t_default;
			else
				return 1;
		}
		else if(corners[1]) {
			// degenerate case: |a + bx^2| < T0 - abs(c) / SMAX
			float T0_ceil = phys->T0 * (1 - fabs(c) / phys->SMAX);
			if(abs_quad(a, b, T0_ceil, xs))
				return 1;
		}
		else if(corners[0]) {
			// instead of doing the |a| < |T0(1-(c-d)/SMAX|, do via quadratic and hijack the machinery that we have already
			// pc[0] = d * d / phys->SMAX / phys->SMAX;
			// float c0 = (1 - c / phys->SMAX);
			// pc[1] = -2 * d * c0 / phys->SMAX;
			// pc[2] = c0 * c0 - a * a / phys->T0 / phys->T0;

			float s = phys->SMAX * (1 - a / phys->T0);
			if(s < 0) { // never intersects
				return 1;
			}
			float xs_[2] = { (s - c) / d, -(s + c) / d };
			float xs__[2] = { fmin(xs_[0], xs_[1]), fmin(xs_[0], xs_[1]) };
			cbnd(xs, xs_);
		}
		else {
			float k_ = phys->SMAX / phys->T0;
			float mid_ = -c / d;
			float a_ = b * k_ * k_;
			float b_ = b * 2 * k_ * mid_;
			float c_ = mid_ * mid_ * b + a;

			float R2 = phys->T0 * phys->T0;

			float x = phys->T0;
			// easiest way... is sadly numerical root finding.
			// pros: easy to make into integer algorithm
			uint8_t i;
			for(i = 0; i < TIM_ITER_LIM; i++) {
				float dist = origin_dist_sq(a_, b_, c_, x);
				float delta = dist - R2;
				if(abs(delta) < EPS)
					break;
				x -= delta / origin_dist_sq_p(a_, b_, c_, x); // head away if inside
			}
			if(i == TIM_ITER_LIM)
				return 1;
			xs[1] = x * k_ + mid_;
		}
		cbnd(xbnd, xs);
	}

	return 0;
}
static float smoothstep(float t) {
	return 3 * t * t - 2 * t * t * t;
}
uint8_t traj_t(float x0, float x1, float v0, float v1, float *bnd, joint_phys_t *phys) {
	float vm = x1 - x0;
	float vtf[2] = { fabs(v0/ALPHA), fabs(v1/ALPHA) };
	float lv[2][2] = {{-0.6875*v0, 1.6875*vm}, {v0, 1.6875*vm}};
	float lt[2] = {3.375*(phys->I)*v0*vtf[0], 3.375*(phys->I)*vtf[0]*vm};

	float rv[2][2] = {{1.6875*v1, -0.6875*vm}, {1.6875*v1, vm}};
	float rt[2] = {3.375*(phys->I)*v1*vtf[1], 3.375*(phys->I)*vtf[1]*vm};
	bnd[0] = -1; bnd[1] = -1;
	return populate(lv, lt, bnd, phys, ALPHA / v0) || populate(rv, rt, bnd, phys, ALPHA / v1);
}
float lerp(float a, float b, float av, float bv, float tf, float t) {
	float av_ = fmax(fabs(av), 1 / tf);
	float bv_ = fmax(fabs(bv), 1 / tf);

	float l = (av * t + a);
	float m = (b - a) * t / tf + a;
	float r = (bv * (t - tf) + b);
	if(t < ALPHA / av_ && t >= (tf - ALPHA / bv_)) {
		float s0 = smoothstep(t * av_ / ALPHA);
		float s1 = smoothstep((tf - t) * bv_ / ALPHA);
		float s2 = t / tf;
		return (l * (1 - s0) + m * s0) * (1 - s2) + (r * (1 - s1) + m * s1) * s2;
	}
	else if(t < ALPHA / av_) {
		float s = smoothstep(t * av_ / ALPHA);
		return l * (1 - s) + m * s;
	}
	else if(t >= ALPHA / av_ && t < (tf - ALPHA / bv_)) {
		return m;
	}
	else {
		float s = smoothstep((tf - t) * bv_ / ALPHA);
		return r * (1 - s) + m * s;
	}
}
