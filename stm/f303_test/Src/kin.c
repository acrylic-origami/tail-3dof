/*
 * kin.c
 *
 *  Created on: Apr 19, 2020
 *      Author: derek-lam
 */
#include "main.h"
#include "kin.h"
#include <math.h>
#include "imath.h"
#include <string.h>
#include "consts.h"
#include "math_util.h"
#include "lapack/f2c.h"

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
