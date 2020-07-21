/*
 * traj_qsp.c
 *
 *  Created on: Jul 19, 2020
 *      Author: derek-lam
 */

#include "traj.h"
#include "traj_qsp.h"
#include <math.h>

static void qsp(float *ts, float *q, float t, uint8_t tau, float *r) {
	float T[4][3] = { 0 };
	float Tt[4] = { 0 };
	for(uint8_t i = 0; i < 4; i++) {
		for(uint8_t j = i + 1; j < 4; j++) {
			T[j][i] = 1.0f / (ts[j] - ts[i]);
		}
		Tt[i] = t - ts[i];
	}
	float sp[5] = { 0 }, sp_[5] = { 0 }, sp__[5] = { 0 };
	if(t > ts[2]) {
		sp[2] = (1.0f - Tt[1] * T[3][1]) * (1.0f - Tt[2] * T[3][2]);
		sp[3] = (1.0f - Tt[2] * T[3][2]) * (Tt[1] * T[3][1] + Tt[2] * T[3][2]);
		sp[4] = (Tt[2] * T[3][2]) * (Tt[2] * T[3][2]);
		sp_[2] = -T[3][2] * (1.0f - Tt[1] * T[3][1]) - T[3][1] * (1.0f - Tt[2] * T[3][2]);
		sp_[3] = -Tt[1] * T[3][1] * T[3][2] + T[3][1] * (1.0f - Tt[2] * T[3][2]) + T[3][2] * (1.0f - 2.0f * Tt[2] * T[3][2]) ;
		sp_[4] = 2.0f * Tt[2] * (T[3][2] * T[3][2]);
		if(tau) {
			sp__[2] = 2.0f * T[3][2] * T[3][1];
			sp__[3] = -2.0f * T[3][2] * (T[3][1] + T[3][2]);
			sp__[4] = 2.0f * T[3][2] * T[3][2];
		}
	}
	else if(t > ts[1]) {
		sp[1] = (1.0f - Tt[0] * T[2][0]) * (1.0f - Tt[1] * T[2][1]);
		sp[2] = Tt[0] * T[2][0] * (1.0f - Tt[1] * T[2][1]) + (1.0f - Tt[1] * T[3][1]) * (Tt[1] * T[2][1]);
		sp[3] = (Tt[1] * Tt[1]) * T[3][1] * T[2][1];
		sp_[1] = -T[2][1] * (1.0f - Tt[0] * T[2][0]) - T[2][0] * (1.0f - Tt[1] * T[2][1]) ;
		sp_[2] = -Tt[0] * T[2][0] * T[2][1] + T[2][0] * (1.0f - Tt[1] * T[2][1]) - T[3][1] * Tt[1] * T[2][1] + T[2][1] * (1.0f - Tt[1] * T[3][1]);
		sp_[3] = 2.0f * Tt[1] * T[2][1] * T[3][1];
		if(tau) {
			sp__[1] = 2.0f * T[2][1] * T[2][0];
			sp__[2] = -2.0f * T[2][1] * (T[2][0] + T[3][1]);
			sp__[3] = 2.0f * T[2][1] * T[3][1];
		}
	}
	else {
		sp[0] = (1.0f - Tt[0] * T[1][0]) * (1.0f - Tt[0] * T[1][0]);
		sp[1] = Tt[0] * T[1][0] * (2.0f - Tt[0] * (T[2][0] + T[1][0]));
		sp[2] = (Tt[0] * Tt[0]) * T[2][0] * T[1][0];
		sp_[0] = -2.0f * T[1][0] * (1.0f - Tt[0] * T[1][0]);
		sp_[1] = 2.0f * (1.0f - Tt[0] * (T[1][0] + T[2][0])) * T[1][0];
		sp_[2] = 2.0f * Tt[0] * T[2][0] * T[1][0];
		if(tau) {
			sp__[0] = 2.0f * T[1][0] * T[1][0];
			sp__[1] = -2.0f * T[1][0] * (T[1][0] + T[2][0]);
			sp__[2] = 2.0f * T[2][0] * T[1][0];
		}
	}

	r[0] = r[1] = 0;
	if(tau)
		r[2] = 0;

	for(uint8_t i = 0; i < 5; i++) {
		r[0] += sp[i] * q[i];
		r[1] += sp_[i] * q[i];
		if(tau)
			r[2] += sp__[i] * q[i];
	}
}
static void qsp_params(float x, float v0, float v1, float dd, float *k, float *q) {
	k[0] = 0.0f;
	k[1] = v0 == 0 ? 1.0f / 3.0f : fmin(2.0f * ALPHA_F * x / fabs(v0), 1.0f / 3.0f);
	k[2] = v1 == 0 ? 2.0f / 3.0f : (1.0f - fmin(2.0f * ALPHA_F * x / fabs(v1), 1.0f / 3.0f));
	k[3] = 1.0f;

	q[0] = 0.0f;
	q[1] = fmin(fabs(v0 / 6), ALPHA_F * x) * fsgn(v0);
	q[3] = dd - fmin(fabs(v1 / 6), ALPHA_F * x) * fsgn(v1);
	q[2] = (q[3] - q[1]) / (1 - (k[1] + k[2]) / 2) * k[2] / 2 + q[1]; // watch the order
	q[4] = dd;
}
void lerp_qsp(float a, float b, float v0, float v1, float tf, float t, float *tr) {
	float k[4], q[5];
	qsp_params(1.0f / tf, v0, v1, (b - a) / tf, k, q); // TODO for optimization, solve once at top level instead
	qsp(k, q, t / tf, 0, tr);
	tr[0] = (tr[0] * tf) + a;
//	tr[1] *= tf;
}
static void wc_s_tau(float x, float v0, float v1, float dd, const joint_phys_t *phys, float *r) {
	float k[4], q[5];
	qsp_params(x, v0, v1, dd, k, q); // TODO for optimization, solve once at top level instead
	float tau_[3];
	float xvt[3];
	qsp(k, q, k[1] / 2.0f, 1, xvt);
	tau_[0] = xvt[2];
	qsp(k, q, 0.5f, 1, xvt);
	tau_[1] = xvt[2];
	qsp(k, q, (1.0f + k[2]) / 2.0f, 1, xvt);
	tau_[2] = xvt[2];

	qsp(k, q, k[1], 1, xvt);
	float va = xvt[1];
	qsp(k, q, k[2], 1, xvt);
	float vb = xvt[1];

	float vt[4][2] = {
			{ v0, tau_[0] },
			{ va, fmax(tau_[0], tau_[1]) },
			{ vb, fmax(tau_[1], tau_[2]) },
			{ v1, tau_[2] }
	};

	r[0] = r[1] = -1;
	for(uint8_t i = 0; i < 4; i++) {
		for(uint8_t j = 0; j < 2; j++) {
			if(fabs(vt[i][j]) > r[j]) {
				r[j] = fabs(vt[i][j]);
			}
		}
	}
	r[0] /= (float)phys->SMAX / (1 << _W);
	r[1] *= (x * phys->I) / phys->T0;
}
uint8_t traj_qsp(float p0, float p1, float v0, float v1, const joint_phys_t *phys, float *r) {
	volatile float vmax = fmax(fabs(v0), fabs(v1));
	volatile float dd = p1 - p0;
	volatile float x[2] = { -1, 1.0 }; // vmax / ALPHA_F / 6.0f // ah screw this setpoint, it's too complicated -- just use reasonable value
	volatile float vt[2];
	wc_s_tau(x[1], v0, v1, dd * x[1], phys, vt);
	volatile float R[2] = { -1, vt[0] * vt[0] + vt[1] * vt[1] - 1.0f };
//	if(R[1] > 0) {
		for(uint8_t iter = 0; iter < TIM_ITER_LIM && fabs(R[1]) > NEWTON_THRESH && x[1] >= 0; iter++) {
			volatile float drdx = R[0] >= 0 ? (R[1] - R[0]) / (x[1] - x[0]) : DRDX_INIT;
			x[0] = x[1];
			x[1] -= R[1] / drdx;
			R[0] = R[1];
			wc_s_tau(x[1], v0, v1, dd * x[1], phys, vt);
			R[1] = vt[0] * vt[0] + vt[1] * vt[1] - 1.0f;
		}
		*r = 1 / x[1];
		return R[1] > NEWTON_THRESH;
//	}
//	else {
//		volatile float tau_crit = sqrt(1.0f - vt[0] * vt[0]); // in normalized SMAX-T0 units
//		*r = t0 * (vt[1] / tau_crit);
//		return 0;
//	}
//	else {
//		wc_s_tau(1.0f, v0, v1, dd, phys, vt);
//		if(vt[0] < 1.0f) {
//			*r = vt[1] / sqrt(1.0f - vt[0] * vt[0]);
//			return 0;
//		}
//		else {
//			// already too fast
//			// TODO if this is due to the middle velocities rather than endpoint,
//			// should be able to solve by making the excursion limit tighter
//			return 1;
//		}
//	}
}
