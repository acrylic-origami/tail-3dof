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
		c[i] = (int16_t)icos(q[i] * INV_PI >> (_W + 1)) << 1;
		s[i] = (int16_t)isin(q[i] * INV_PI >> (_W + 1)) << 1;
	}

	fwd_kin(A, q, p);
	int32_t vp[3] = { 0, -s[1] * (A[2] + (A[3] * c[2] >> _W)) >> _W, (-c[1] * A[3] >> _W) * s[2] >> _W };
	real J_[9] = { // note column-major order
		0,         (-p[2]),            (p[1])           ,
		(c[1] * ((A[3] * c[2] >> _W) + A[2]) >> _W),   c[0] * vp[1] >> _W,  s[0] * vp[1] >> _W ,
		((-A[3] * s[1] >> _W) * s[2]) >> _W,    (c[0] * vp[2] >> _W) - ((A[3] * s[0] >> _W) * c[2] >> _W),  (s[0] * vp[2] >> _W) + ((A[3] * c[0] >> _W) * c[2] >> _W)
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
		c[i] = (int16_t)icos(q[i] * INV_PI >> (_W + 1)) << 1;
		s[i] = (int16_t)isin(q[i] * INV_PI >> (_W + 1)) << 1;
	}
	int32_t v = A[1] + (c[1] * (A[2] + (A[3] * c[2] >> _W)) >> _W);

    r[0] = s[1] * ((A[3] * c[2] >> _W) + A[2]) >> _W;
    r[1] = (c[0] * v >> _W) - ((A[3] * s[0] >> _W) * s[2] >> _W);
    r[2] = (s[0] * v >> _W) + ((A[3] * c[0] >> _W) * s[2] >> _W);
}
uint8_t inv_kin(int32_t *A, int32_t *T, int32_t *t, int32_t prev_t0) {
//    volatile int32_t rTi = 0;
//    for(uint8_t i = 0; i < 3; i++) {
//    	rTi += T[i] * T[i];
//    }
//    volatile float rT = (float)isqrt(rTi_1p) / (1 << _W);
    int32_t r_yz_i = isqrt(T[1] * T[1] + T[2] * T[2]);
	volatile float r_yz = (float)r_yz_i / (1 << _W); // WATCH multiply with sqrt
    volatile int32_t r1p_yz_i = r_yz_i - A[1];
    int32_t r1p_i = isqrt(r1p_yz_i * r1p_yz_i + T[0] * T[0]);

    volatile float c1 = (float)r1p_yz_i / r1p_i;

    volatile float y0 = (A[1] + (c1 * A[2])) / (1 << _W);
    volatile float r1 = (c1 * A[3]) / (1 << _W);
    volatile float r2 = (float)A[3] / (1 << _W);
    volatile float y;

    if(r1p_i < FEPS || r_yz < FEPS) {
    	// XZ axis: almost no solutions
    	return 1;
    }
    else if(fabs(1.0 - c1) < FEPS) {
    	// degenerate case: in the y-z plane already, so solve directly
    	if(y0 - r2 > r_yz || y0 + r2 < r_yz)
    		return 1; // outside of range

    	// TODO check if r_yz is correct here. First check is good.
    	y = (y0 * y0 + r_yz * r_yz - r2 * r2) / 2 / y0;
    }
    else {
    	// floats to perfom better around singularities of r1 (hopefully)
    	volatile float pc[3] = {
    			1.0f / (r1 * r1) - 1.0f / (r2 * r2),
				(-2.0f * y0) / (r1 * r1),
				(y0 * y0) / (r1 * r1) + (r_yz * r_yz) / (r2 * r2) - 1.0f
    	};
    	// printf("%.3e\t%.3e\t%.3e\n", pc[0], pc[1], pc[2]);

    	volatile float det = pc[1] * pc[1] - 4 * pc[0] * pc[2];
    	if(det < 0)
    		return 1;

    	float sqrt_det = sqrt(det);
    	y = (-pc[1] + sqrt_det) / 2 / pc[0];
    	volatile float abs_y = fabs(y);
    	if(abs_y > y0 + r1) { // could produce real solutions that are invalid: must be checked against ellipse domain
    		y = (-pc[1] - sqrt_det) / 2 / pc[0];
    		abs_y = fabs(y);
    	}
    	else if(abs_y < y0 - r1)
    		return 1;

    	//	    HAL_UART_Transmit_IT(&huart3, &y, 4);
    	if(abs_y > y0 + r1 || abs_y < y0 - r1)
    		return 1;
    }


    volatile float z = sqrt(r_yz * r_yz - y * y);

    // volatile float m0 = z / r_yz; // TODO check if r_yz is the one I want
    // volatile float m1 = -1.0f / m0;
    // volatile float y_ = (m1 * r_yz) / (m1 - m0); // WATCH multiply with divide
    // volatile float z_ = m0 * y_;
    // volatile float r_yz_ = sqrt(y_ * y_ + z_ * z_); // WATCH multiply with sqrt

    // if(fabs(1 - c1) < EPS / ((float)(1 << _W)))
    // 	t[1] = 0; // optimizing (perhaps prematurely)
    // else {
    //     t[1] = (atan2((float)T[0], (r_yz_ * (1 << _W)) - A[1]) * (1L << _W));
    // }

    // TODO evaluate this r_yz vs. r_yz_ (compensated angle based on re-projection)
    t[1] = (atan2((float)T[0], (r_yz * (1 << _W)) - A[1]) * (1L << _W));

    volatile int32_t dt0 = asin(z / r_yz) * (1L << _W);
    volatile int32_t t0_base = atan2((float)T[2], T[1]) * (1L << _W); // WATCH: atan2 ignores scaling, but asin needs to be renorm'd
    int8_t sgns[2] = { 1, -1 };
    int8_t sgn;
    int32_t t0s[2] = {
    		t0_base - sgns[0] * dt0,
			t0_base - sgns[1] * dt0
    };
    if(abs(t0s[0] - prev_t0) < abs(t0s[1] - prev_t0)) {
    	t[0] = t0s[0];
    	sgn = sgns[0];
    }
    else {
    	t[0] = t0s[1];
    	sgn = sgns[1];
    }

    t[2] = (int32_t)((atan2((z * sgn), (y - y0) / c1)) * (1 << _W)); // WATCH: atan2 ignores scaling
    if(t[2] > M_PI * (1 << _W))
    	t[2] -= 2 * M_PI * (1 << _W);
    return 0;
}
