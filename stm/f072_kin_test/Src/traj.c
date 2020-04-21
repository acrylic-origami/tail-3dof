/*
 * traj.c
 *
 *  Created on: Apr 19, 2020
 *      Author: derek-lam
 */

#include <math.h>
#include "traj.h"

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
static uint8_t populate(float* vs, float* ts, float* xbnd) {
	float a = ts[0], b = ts[1];
	for(uint8_t i = 0; i < 2; i++) {
		float c = vs[i * 2], d = vs[i * 2 + 1];
		int8_t A[4] = { 1, 1, -1, -1 },
			   B[4] = { 1, -1, -1, 1 };
		for(uint8_t i = 0; i < 4; i++) {
			float pc[3] = {
				A[i] * (b / (d * d)),
				B[i] * T0_HS / SMAX_HS - A[i] * 2 * b * c / (d * d),
				A[i] * (a + b * (c * c) / (d * d)) - T0_HS
			};

			float det = pc[1] * pc[1] - 4 * pc[0] * pc[2];
			if(det > 0) {
				float xs[2];
				uint8_t all_neg = 1;
				for(int8_t j = 0; j < 2; j++) {
					xs[j] = (-pc[1] + (j * 2 - 1) * sgn(d) * sqrt(det)) / 2 / pc[0] * d + c;
					all_neg &= ~(xs[j] > 0);
				}
				if(all_neg && pc[2] > 0) {
					return 1;
				}

				cbnd(xbnd, xs);
			}
			else if(pc[2] > 0) {
				return 1;
			}
		}
	}

	return 0;
}
static float smoothstep(float t) {
	return 3 * t * t - 2 * t * t * t;
}
uint8_t traj_t(float x0, float x1, float v0, float v1, float *bnd) {
	float vm = x1 - x0;
	float lv[2][2] = {{-0.6875*v0, 1.6875*vm}, {v0, 1.6875*vm}};
	float lt[2] = {3.375*I_HS*(v0*v0)/ALPHA, 3.375*I_HS*v0/ALPHA};

	float rv[2][2] = {{1.6875*v1, -0.6875*vm}, {1.6875*v1, vm}};
	float rt[2] = {3.375*I_HS*(v1*v1)/ALPHA, 3.375*I_HS*v1/ALPHA};
	bnd[0] = -1; bnd[1] = -1;
	return populate(lv, lt, bnd) || populate(rv, rt, bnd);
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
		float s2 = smoothstep((t - (tf - ALPHA / bv_)) / (ALPHA / av_ - (tf - ALPHA / bv_)));
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
