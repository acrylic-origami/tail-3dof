/*
 * kin.c
 *
 *  Created on: Apr 19, 2020
 *      Author: derek-lam
 */
#include "kin.h"
#include <math.h>
#include "consts.h"
#include "math_util.h"

static void f3(int32_t (*f)(int32_t), int32_t *x, int32_t *r) {
    for(uint8_t i = 0; i < 3; i++) {
        r[i] = f(x[i]);
    }
}

void fwd_kin(int32_t *A, int32_t *q, int32_t *r) {
    int32_t c[3], s[3];
	f3(cos, q, c);
    f3(sin, q, s);
	int32_t v = A[1] + (c[1] * (A[2] + (A[3] * c[2] >> _W)) >> _W);

    r[0] = s[1] * ((A[3] * c[2] >> _W) + A[2]) >> _W;
    r[1] = (c[0] * v >> _W) - ((A[3] * s[0] >> _W) * s[2] >> _W);
    r[2] = (s[0] * v >> _W) + ((A[3] * c[0] >> _W) * s[2] >> _W);
}
uint8_t inv_kin(int32_t *A, int32_t *T, int32_t *t) {
    int32_t rT = 0;
    for(uint8_t i = 0; i < 3; i++) {
        rT += T[i] * T[i] >> _W;
    }
    rT = isqrt(rT) << _HW;
    int32_t r_yz = isqrt((T[1] * T[1] >> _W) + (T[2] * T[2] >> _W)) << _HW;
    int32_t c1 = (r_yz << _W) / rT;

    int32_t r0 = A[1] + (c1 * A[2] >> _W);
	int32_t r1 = c1 * A[3] >> _W;
	int32_t r2 = A[3];

	int32_t y;

	if((1 << _W) - c1 < EPS) {
		// degenerate case: in the y-z plane already, so solve directly
		if(r_yz - r2 > r0 || r_yz + r2 < r0)
			return 1; // outside of range

		y = (r0 * r0 - r1 * r1) / 2 / r_yz; // EXEMPT: * followed by /, along with distributivity
	}
	else {
	    int32_t pc[3] = {
	        (1 << _TW) / (r1 * r1 >> _W) - (1 << _TW) / (r2 * r2 >> _W),
			-2 * (r_yz << _W) / (r1 * r1 >> _W),
			r0 * r0 / (r2 * r2 >> _W) + (r_yz * r_yz / (r1 * r1 >> _W)) - (1 << _W)
	    };
	    int32_t det = pc[1] * pc[1] - 4 * pc[0] * pc[2] >> _W;
	    if(det < 0)
	        return 1;

	    y = ((-pc[1] + (isqrt(det) << _HW)) << _W) / 2 / pc[0]; // WATCH: sqrt with divide
	    if(abs(y) > r0)
	        y = ((-pc[1] - (isqrt(det) << _HW)) << _W) / 2 / pc[0];
	    if(abs(y) > r0)
	        return 1;
	}

    int32_t z = isqrt(r0 * r0 - y * y) >> _HW; // WATCH multiply with sqrt

    int32_t m0 = (z << _W) / r0;
    int32_t m1 = -(1 << _TW) / m0;
    int32_t y_ = (m1 * r_yz) / (m1 - m0); // WATCH multiply with divide
    int32_t z_ = m0 * y_ >> _W;
    int32_t r_yz_ = isqrt(y_ * y_ + z_ * z_) >> _HW; // WATCH multiply with sqrt

    if(1 - c1 < EPS)
    	t[1] = 0; // optimizing (perhaps prematurely
    else {
        t[1] = atan2(T[0], r_yz_ - A[1]) * (1 << _W); // WATCH: atan2 ignores scaling // TODO replace with LUT
    }

    t[0] = (asin(((int32_t)((z << _W) / r0)) / (1 << _W)) + atan2((int32_t)T[2], T[1])) * (1 << _W); // WATCH: atan2 ignores scaling
    t[2] = (atan2(z, ((y - r_yz) << _W) / c1) + M_PI) * (1 << _W); // WATCH: atan2 ignores scaling
    if(t[2] > M_PI * (1 << _W))
    	t[2] -= 2 * M_PI * (1 << _W);
    return 0;
}
