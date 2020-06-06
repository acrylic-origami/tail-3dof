/*
 * kin.c
 *
 *  Created on: Apr 19, 2020
 *      Author: derek-lam
 */
#include "kin.h"
#include <math.h>
#include "imath.h"
#include <string.h>
#include "consts.h"
#include "math_util.h"
#include "f2c.h"

extern int sgetrf_(integer *m, integer *n, real *a, integer *lda,
		integer *ipiv, integer *info);
extern int sgetrs_(char *trans, integer *n, integer *nrhs, real *a,
		integer *lda, integer *ipiv, real *b, integer *ldb, integer *info);

//static void f3(int32_t (*f)(int32_t), int32_t *x, int32_t *r) {
//    for(uint8_t i = 0; i < 3; i++) {
//        r[i] = f(x[i]);
//    }
//}
//static int32_t lsh1(int32_t x) {
//	return x << 1;
//}
uint8_t inv_jacob(int32_t *A, int32_t *q, int32_t *tb, float *qb) {
	// all inputs in 256 units
	// qb in natural units
	int32_t c[3], s[3], p[3];
	for(uint8_t i = 0; i < 3; i++) {
		c[i] = icos(q[i]) << 1;
		s[i] = isin(q[i]) << 1;
	}

	fwd_kin(A, q, p);
	real J_[9] = { // note column-major order
		0,         (-p[2]),            (p[1])           ,
		(c[1] * ((A[3] * c[2] >> _W) + A[2]) >> _W),   ((-c[0] * s[1] >> _W) * (A[2] + (A[3] * c[2] >> _W))) >> _W,  ((-s[0] * s[1] >> _W) * (A[2] + (A[3] * c[2] >> _W))) >> _W ,
		((-A[3] * s[1] >> _W) * s[2]) >> _W,    (((-c[0] * c[1] >> _W) * s[2] >> _W) * A[3] >> _W) - ((A[3] * s[0] >> _W) * c[2] >> _W),  (((-s[0] * c[1] >> _W) * s[2] >> _W) * A[3] >> _W) + ((A[3] * c[0] >> _W) * c[2] >> _W)
	};

	for(uint8_t i = 0; i < 3; i++)
		qb[i] = (real)tb[i];

	char trans = 'N';
	integer na = 3;
	integer nb = 1;
	integer err = 0, info = 0;
	integer ipiv[3];

	err = sgetrf_(&na, &na, J_, &na, ipiv, &info);
	if(!err)
		err = sgetrs_(&trans, &na, &nb, J_, &na, ipiv, qb, &na, &info);

	return err;
}

void fwd_kin(int32_t *A, int32_t *q, int32_t *r) {
    int32_t c[3], s[3];
	for(uint8_t i = 0; i < 3; i++) {
		c[i] = icos(q[i]) << 1;
		s[i] = isin(q[i]) << 1;
	}
	int32_t v = A[1] + (c[1] * (A[2] + (A[3] * c[2] >> _W)) >> _W);

    r[0] = s[1] * ((A[3] * c[2] >> _W) + A[2]) >> _W;
    r[1] = (c[0] * v >> _W) - ((A[3] * s[0] >> _W) * s[2] >> _W);
    r[2] = (s[0] * v >> _W) + ((A[3] * c[0] >> _W) * s[2] >> _W);
}
uint8_t inv_kin(int32_t *A, int32_t *T, int32_t *t, int32_t prev_t0) {
    volatile int32_t rT = 0;
    for(uint8_t i = 0; i < 3; i++) {
        rT += T[i] * T[i];
    }
    rT = isqrt(rT);
    volatile int32_t r_yz = isqrt(T[1] * T[1] + T[2] * T[2]); // WATCH multiply with sqrt
    volatile int32_t c1 = (r_yz << _W) / rT;

    volatile int32_t r0 = A[1] + (c1 * A[2] >> _W);
	volatile int32_t r1 = c1 * A[3] >> _W;
	volatile int32_t r2 = A[3];

	volatile int32_t y;

	if(abs((1 << _W) - c1) < EPS) {
		// degenerate case: in the y-z plane already, so solve directly
		if(r_yz - r2 > r0 || r_yz + r2 < r0)
			return 1; // outside of range

		y = (r0 * r0 + r_yz * r_yz - r1 * r1) / 2 / r_yz; // EXEMPT: * followed by /, along with distributivity
	}
	else {
		// TODO: new control flow in full quadratic solution, debug why this isn't working properly.
	    volatile int32_t pc[3] = {
	        (1 << _TW) / (r1 * r1 >> _W) - (1 << _TW) / (r2 * r2 >> _W),
			-2 * (r_yz << _W) / (r1 * r1 >> _W),
			r0 * r0 / (r2 * r2 >> _W) + (r_yz * r_yz / (r1 * r1 >> _W)) - (1 << _W)
	    };
	    volatile int32_t det = pc[1] * pc[1] - 4 * pc[0] * pc[2];
	    if(det < 0)
	        return 1;

	    y = ((-pc[1] + isqrt(det)) << _W) / 2 / pc[0]; // WATCH: sqrt with divide
	    if(abs(y) > r0)
	        y = ((-pc[1] - isqrt(det)) << _W) / 2 / pc[0];
	    if(abs(y) > r0)
	        return 1;
	}

    volatile int32_t z = isqrt(r0 * r0 - y * y); // WATCH multiply with sqrt

    volatile int32_t m0 = (z << _W) / r0;
    volatile int32_t m1 = -(1 << _TW) / m0;
    volatile int32_t y_ = (m1 * r_yz) / (m1 - m0); // WATCH multiply with divide
    volatile int32_t z_ = m0 * y_ >> _W;
    volatile int32_t r_yz_ = isqrt(y_ * y_ + z_ * z_); // WATCH multiply with sqrt

    if(abs(1 - c1) < EPS)
    	t[1] = 0; // optimizing (perhaps prematurely)
    else {
        t[1] = (int32_t)(atan2((float)T[0], r_yz_ - A[1]) * (1 << _W)); // WATCH: atan2 ignores scaling // TODO replace with LUT
    }

    volatile int32_t dt0 = asin((float)z / r0) * (1L << _W);
    volatile int32_t t0_base = atan2((float)T[2], T[1]) * (1L << _W); // WATCH: atan2 ignores scaling, but asin needs to be renorm'd
    int8_t sgns[2] = { 1, -1 };
    int8_t sgn;
    int32_t t0s[2] = {
    	t0_base + sgns[0] * dt0,
		t0_base + sgns[1] * dt0
    };
    if(abs(t0s[0] - prev_t0) < abs(t0s[1] - prev_t0)) {
    	t[0] = t0s[0];
    	sgn = sgns[0];
    }
    else {
    	t[0] = t0s[1];
    	sgn = sgns[1];
    }

    t[2] = (int32_t)((atan2((float)(z * sgn), ((y - r_yz) << _W) / c1) + M_PI) * (1 << _W)); // WATCH: atan2 ignores scaling
    if(t[2] > M_PI * (1 << _W))
    	t[2] -= 2 * M_PI * (1 << _W);
    return 0;
}
