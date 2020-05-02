/*
 * kin.c
 *
 *  Created on: Apr 19, 2020
 *      Author: derek-lam
 */
#include "kin.h"
#include <math.h>
#include "consts.h"

static void f3(float (*f)(float), float *x, float *r) {
    for(uint8_t i = 0; i < 3; i++) {
        r[i] = f(x[i]);
    }
}
static float sq(float x) { return x * x; }

void fwd_kin(float *A, float *q, float *r) {
    float c[3], s[3];
	f3(cos, q, c);
    f3(sin, q, s);
	float v = A[1] + c[1] * (A[2] + A[3] * c[2]);

    r[0] = s[1] * (A[3] * c[2] + A[2]);
    r[1] = c[0] * v - A[3] * s[0] * s[2];
    r[2] = s[0] * v + A[3] * c[0] * s[2];
}
uint8_t inv_kin(float *A, float *T, float *t) {
    float rT = 0;
    for(uint8_t i = 0; i < 3; i++) {
        rT += T[i] * T[i];
    }
    rT = sqrt(rT);
    float r_yz = sqrt(T[1] * T[1] + T[2] * T[2]);
    float c1 = r_yz / rT;

    float r0 = A[1] + c1 * A[2];
	float r1 = c1 * A[3];
	float r2 = A[3];

	float y;

	if(1 - c1 < EPS) {
		// degenerate case: in the y-z plane already, so solve directly
		if(r_yz - r2 > r0 || r_yz + r2 < r0)
			return 1; // outside of range

		y = (r0 * r0 - r1 * r1) / 2 / r_yz;
	}
	else {
	    float pc[3] = {
	        1 / (r1 * r1) - 1 / (r2 * r2),
			-2 * r_yz / (r1 * r1),
			r0 * r0 / (r2 * r2) + (r_yz * r_yz / (r1 * r1)) - 1
	    };
	    float det = pc[1] * pc[1] - 4 * pc[0] * pc[2];
	    if(det < 0)
	        return 1;

	    y = (-pc[1] + sqrt(det)) / 2 / pc[0];
	    if(fabs(y) > r0)
	        y = (-pc[1] - sqrt(det)) / 2 / pc[0];
	    if(fabs(y) > r0)
	        return 1;
	}

    float z = sqrt(r0 * r0 - y * y);

    float m0 = z / r0;
    float m1 = -1 / m0;
    float y_ = (m1 * r_yz) / (m1 - m0);
    float z_ = m0 * y_;
    float r_yz_ = sqrt(y_ * y_ + z_ * z_);

    if(1 - c1 < EPS)
    	t[1] = 0; // optimizing (perhaps prematurely
    else
        t[1] = atan2(T[0], r_yz_ - A[1]);

    t[0] = asin(z / r0) + atan2(T[2], T[1]);
    t[2] = atan2(z, (y - r_yz) / c1) + M_PI;
    if(t[2] > M_PI)
    	t[2] -= 2 * M_PI;
    return 0;
}
