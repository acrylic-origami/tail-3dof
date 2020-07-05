/*
 * ik_test.c
 *
 *  Created on: Jun 30, 2020
 *      Author: derek-lam
 */

#include <math.h>
#include <complex.h>
#include "ik_test.h"
#include <stdint.h>

void ComputeFk(const float* j, float* eetrans) {
	float x0,x1,x2,x3,x4,x5,x6,x7,x8,x9;
	x0=cos(j[0]);
	x1=cos(j[1]);
	x2=cos(j[2]);
	x3=sin(j[0]);
	x4=sin(j[2]);
	x5=sin(j[1]);
	x6=((0.234)*x2);
	x7=((0.234)*x4);
	x8=((0.195)*x3);
	x9=(x0*x1);
	eetrans[0]=((((0.195)*x0))+(((0.195)*x9))+(((-1.0)*x3*x7))+((x6*x9)));
	eetrans[1]=(((x0*x7))+((x1*x8))+x8+((x1*x3*x6)));
	eetrans[2]=((((-0.195)*x5))+(((-1.0)*x5*x6)));
}

#define EPS 0.0000000001f

static inline void polyroots2(float rawcoeffs[2+1], float rawroots[2], int* numroots) {
	float det = rawcoeffs[1]*rawcoeffs[1]-4*rawcoeffs[0]*rawcoeffs[2];
	if( det < 0 ) {
		*numroots = 0;
	}
	else if( det == 0 ) {
		rawroots[0] = -0.5f*rawcoeffs[1]/rawcoeffs[0];
		*numroots = 1;
	}
	else {
		det = sqrt(det);
		rawroots[0] = (-rawcoeffs[1]+det)/(2*rawcoeffs[0]);
		rawroots[1] = (-rawcoeffs[1]-det)/(2*rawcoeffs[0]);//rawcoeffs[2]/(rawcoeffs[0]*rawroots[0]);
		*numroots = 2;
	}
}
static inline void polyroots3(float rawcoeffs[3+1], float rawroots[3], int* numroots)
{
	if( rawcoeffs[0] == 0 ) {
		// solve with one reduced degree
		polyroots2(&rawcoeffs[1], &rawroots[0], numroots);
		return;
	}
	const float tol = 128.0f*EPS;
	const float tolsqrt = sqrt(EPS);
	_Complex float coeffs[3];
	const int maxsteps = 110;
	for(int i = 0; i < 3; ++i) {
		coeffs[i] = rawcoeffs[i+1]/rawcoeffs[0];
	}
	_Complex float roots[3];
	float err[3];
	roots[0] = 1 + 0 * I;
	roots[1] = 0.4f + 0.9f * I; // any complex number not a root of unity works
	err[0] = 1.0f;
	err[1] = 1.0f;
	for(int i = 2; i < 3; ++i) {
		roots[i] = roots[i-1]*roots[1];
		err[i] = 1.0f;
	}
	for(int step = 0; step < maxsteps; ++step) {
		int changed = 0;
		for(int i = 0; i < 3; ++i) {
			if ( err[i] >= tol ) {
				changed = 1;
				// evaluate
				_Complex float x = roots[i] + coeffs[0];
				for(int j = 1; j < 3; ++j) {
					x = roots[i] * x + coeffs[j];
				}
				for(int j = 0; j < 3; ++j) {
					if( i != j ) {
						if( roots[i] != roots[j] ) {
							x /= (roots[i] - roots[j]);
						}
					}
				}
				roots[i] -= x;
				err[i] = fabs(x);
			}
		}
		if( !changed ) {
			break;
		}
	}

	*numroots = 0;
	int visited[3] = {0};
	for(int i = 0; i < 3; ++i) {
		if( !visited[i] ) {
			// might be a multiple root, in which case it will have more error than the other roots
			// find any neighboring roots, and take the average
			_Complex float newroot=roots[i];
			int n = 1;
			for(int j = i+1; j < 3; ++j) {
				// care about error in creal much more than cimaginary
				if( fabs(creal(roots[i])-creal(roots[j])) < tolsqrt && fabs(cimag(roots[i])-cimag(roots[j])) < 0.002f ) {
					newroot += roots[j];
					n += 1;
					visited[j] = 1;
				}
			}
			if( n > 1 ) {
				newroot /= n;
			}
			// there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
			if( fabs(cimag(newroot)) < tolsqrt ) {
				rawroots[(*numroots)++] = creal(newroot);
			}
		}
	}
}
static inline void polyroots4(float rawcoeffs[4+1], float rawroots[4], int* numroots)
{
	if( rawcoeffs[0] == 0 ) {
		// solve with one reduced degree
		polyroots3(&rawcoeffs[1], &rawroots[0], numroots);
		return;
	}
	const float tol = 128.0f*EPS;
	const float tolsqrt = sqrt(EPS);
	_Complex float coeffs[4];
	const int maxsteps = 110;
	for(int i = 0; i < 4; ++i) {
		coeffs[i] = rawcoeffs[i+1]/rawcoeffs[0];
	}
	_Complex float roots[4];
	float err[4];
	roots[0] = 1 + 0 * I;
	roots[1] = 0.4f + 0.9f * I; // any complex number not a root of unity works
	err[0] = 1.0f;
	err[1] = 1.0f;
	for(int i = 2; i < 4; ++i) {
		roots[i] = roots[i-1]*roots[1];
		err[i] = 1.0f;
	}
	for(int step = 0; step < maxsteps; ++step) {
		int changed = 0;
		for(int i = 0; i < 4; ++i) {
			if ( err[i] >= tol ) {
				changed = 1;
				// evaluate
				_Complex float x = roots[i] + coeffs[0];
				for(int j = 1; j < 4; ++j) {
					x = roots[i] * x + coeffs[j];
				}
				for(int j = 0; j < 4; ++j) {
					if( i != j ) {
						if( roots[i] != roots[j] ) {
							x /= (roots[i] - roots[j]);
						}
					}
				}
				roots[i] -= x;
				err[i] = fabs(x);
			}
		}
		if( !changed ) {
			break;
		}
	}

	*numroots = 0;
	int visited[4] = {0};
	for(int i = 0; i < 4; ++i) {
		if( !visited[i] ) {
			// might be a multiple root, in which case it will have more error than the other roots
			// find any neighboring roots, and take the average
			_Complex float newroot=roots[i];
			int n = 1;
			for(int j = i+1; j < 4; ++j) {
				// care about error in creal much more than cimaginary
				if( fabs(creal(roots[i])-creal(roots[j])) < tolsqrt && fabs(cimag(roots[i])-cimag(roots[j])) < 0.002f ) {
					newroot += roots[j];
					n += 1;
					visited[j] = 1;
				}
			}
			if( n > 1 ) {
				newroot /= n;
			}
			// there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
			if( fabs(cimag(newroot)) < tolsqrt ) {
				rawroots[(*numroots)++] = creal(newroot);
			}
		}
	}
}
static inline void polyroots5(float rawcoeffs[5+1], float rawroots[5], int* numroots)
{
	if( rawcoeffs[0] == 0 ) {
		// solve with one reduced degree
		polyroots4(&rawcoeffs[1], &rawroots[0], numroots);
		return;
	}
	const float tol = 128.0f*EPS;
	const float tolsqrt = sqrt(EPS);
	_Complex float coeffs[5];
	const int maxsteps = 110;
	for(int i = 0; i < 5; ++i) {
		coeffs[i] = rawcoeffs[i+1]/rawcoeffs[0];
	}
	_Complex float roots[5];
	float err[5];
	roots[0] = 1 + 0 * I;
	roots[1] = 0.4f + 0.9f * I; // any complex number not a root of unity works
	err[0] = 1.0f;
	err[1] = 1.0f;
	for(int i = 2; i < 5; ++i) {
		roots[i] = roots[i-1]*roots[1];
		err[i] = 1.0f;
	}
	for(int step = 0; step < maxsteps; ++step) {
		int changed = 0;
		for(int i = 0; i < 5; ++i) {
			if ( err[i] >= tol ) {
				changed = 1;
				// evaluate
				_Complex float x = roots[i] + coeffs[0];
				for(int j = 1; j < 5; ++j) {
					x = roots[i] * x + coeffs[j];
				}
				for(int j = 0; j < 5; ++j) {
					if( i != j ) {
						if( roots[i] != roots[j] ) {
							x /= (roots[i] - roots[j]);
						}
					}
				}
				roots[i] -= x;
				err[i] = fabs(x);
			}
		}
		if( !changed ) {
			break;
		}
	}

	*numroots = 0;
	int visited[5] = {0};
	for(int i = 0; i < 5; ++i) {
		if( !visited[i] ) {
			// might be a multiple root, in which case it will have more error than the other roots
			// find any neighboring roots, and take the average
			_Complex float newroot=roots[i];
			int n = 1;
			for(int j = i+1; j < 5; ++j) {
				// care about error in creal much more than cimaginary
				if( fabs(creal(roots[i])-creal(roots[j])) < tolsqrt && fabs(cimag(roots[i])-cimag(roots[j])) < 0.002f ) {
					newroot += roots[j];
					n += 1;
					visited[j] = 1;
				}
			}
			if( n > 1 ) {
				newroot /= n;
			}
			// there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
			if( fabs(cimag(newroot)) < tolsqrt ) {
				rawroots[(*numroots)++] = creal(newroot);
			}
		}
	}
}
static inline void polyroots6(float rawcoeffs[6+1], float rawroots[6], int* numroots)
{
	if( rawcoeffs[0] == 0 ) {
		// solve with one reduced degree
		polyroots5(&rawcoeffs[1], &rawroots[0], numroots);
		return;
	}
	const float tol = 128.0f*EPS;
	const float tolsqrt = sqrt(EPS);
	_Complex float coeffs[6];
	const int maxsteps = 110;
	for(int i = 0; i < 6; ++i) {
		coeffs[i] = rawcoeffs[i+1]/rawcoeffs[0];
	}
	_Complex float roots[6];
	float err[6];
	roots[0] = 1 + 0 * I;
	roots[1] = 0.4f + 0.9f * I; // any complex number not a root of unity works
	err[0] = 1.0f;
	err[1] = 1.0f;
	for(int i = 2; i < 6; ++i) {
		roots[i] = roots[i-1]*roots[1];
		err[i] = 1.0f;
	}
	for(int step = 0; step < maxsteps; ++step) {
		int changed = 0;
		for(int i = 0; i < 6; ++i) {
			if ( err[i] >= tol ) {
				changed = 1;
				// evaluate
				_Complex float x = roots[i] + coeffs[0];
				for(int j = 1; j < 6; ++j) {
					x = roots[i] * x + coeffs[j];
				}
				for(int j = 0; j < 6; ++j) {
					if( i != j ) {
						if( roots[i] != roots[j] ) {
							x /= (roots[i] - roots[j]);
						}
					}
				}
				roots[i] -= x;
				err[i] = fabs(x);
			}
		}
		if( !changed ) {
			break;
		}
	}

	*numroots = 0;
	int visited[6] = {0};
	for(int i = 0; i < 6; ++i) {
		if( !visited[i] ) {
			// might be a multiple root, in which case it will have more error than the other roots
			// find any neighboring roots, and take the average
			_Complex float newroot=roots[i];
			int n = 1;
			for(int j = i+1; j < 6; ++j) {
				// care about error in creal much more than cimaginary
				if( fabs(creal(roots[i])-creal(roots[j])) < tolsqrt && fabs(cimag(roots[i])-cimag(roots[j])) < 0.002f ) {
					newroot += roots[j];
					n += 1;
					visited[j] = 1;
				}
			}
			if( n > 1 ) {
				newroot /= n;
			}
			// there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
			if( fabs(cimag(newroot)) < tolsqrt ) {
				rawroots[(*numroots)++] = creal(newroot);
			}
		}
	}
}
static inline void polyroots7(float rawcoeffs[7+1], float rawroots[7], int* numroots)
{
	if( rawcoeffs[0] == 0 ) {
		// solve with one reduced degree
		polyroots6(&rawcoeffs[1], &rawroots[0], numroots);
		return;
	}
	const float tol = 128.0f*EPS;
	const float tolsqrt = sqrt(EPS);
	_Complex float coeffs[7];
	const int maxsteps = 110;
	for(int i = 0; i < 7; ++i) {
		coeffs[i] = rawcoeffs[i+1]/rawcoeffs[0];
	}
	_Complex float roots[7];
	float err[7];
	roots[0] = 1 + 0 * I;
	roots[1] = 0.4f + 0.9f * I; // any complex number not a root of unity works
	err[0] = 1.0f;
	err[1] = 1.0f;
	for(int i = 2; i < 7; ++i) {
		roots[i] = roots[i-1]*roots[1];
		err[i] = 1.0f;
	}
	for(int step = 0; step < maxsteps; ++step) {
		int changed = 0;
		for(int i = 0; i < 7; ++i) {
			if ( err[i] >= tol ) {
				changed = 1;
				// evaluate
				_Complex float x = roots[i] + coeffs[0];
				for(int j = 1; j < 7; ++j) {
					x = roots[i] * x + coeffs[j];
				}
				for(int j = 0; j < 7; ++j) {
					if( i != j ) {
						if( roots[i] != roots[j] ) {
							x /= (roots[i] - roots[j]);
						}
					}
				}
				roots[i] -= x;
				err[i] = fabs(x);
			}
		}
		if( !changed ) {
			break;
		}
	}

	*numroots = 0;
	int visited[7] = {0};
	for(int i = 0; i < 7; ++i) {
		if( !visited[i] ) {
			// might be a multiple root, in which case it will have more error than the other roots
			// find any neighboring roots, and take the average
			_Complex float newroot=roots[i];
			int n = 1;
			for(int j = i+1; j < 7; ++j) {
				// care about error in creal much more than cimaginary
				if( fabs(creal(roots[i])-creal(roots[j])) < tolsqrt && fabs(cimag(roots[i])-cimag(roots[j])) < 0.002f ) {
					newroot += roots[j];
					n += 1;
					visited[j] = 1;
				}
			}
			if( n > 1 ) {
				newroot /= n;
			}
			// there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
			if( fabs(cimag(newroot)) < tolsqrt ) {
				rawroots[(*numroots)++] = creal(newroot);
			}
		}
	}
}
static inline void polyroots8(float rawcoeffs[8+1], float rawroots[8], int* numroots)
{
	if( rawcoeffs[0] == 0 ) {
		// solve with one reduced degree
		polyroots7(&rawcoeffs[1], &rawroots[0], numroots);
		return;
	}
	const float tol = 128.0f*EPS;
	const float tolsqrt = sqrt(EPS);
	_Complex float coeffs[8];
	const int maxsteps = 110;
	for(int i = 0; i < 8; ++i) {
		coeffs[i] = rawcoeffs[i+1]/rawcoeffs[0];
	}
	_Complex float roots[8];
	float err[8];
	roots[0] = 1 + 0 * I;
	roots[1] = 0.4f + 0.9f * I; // any complex number not a root of unity works
	err[0] = 1.0f;
	err[1] = 1.0f;
	for(int i = 2; i < 8; ++i) {
		roots[i] = roots[i-1]*roots[1];
		err[i] = 1.0f;
	}
	for(int step = 0; step < maxsteps; ++step) {
		int changed = 0;
		for(int i = 0; i < 8; ++i) {
			if ( err[i] >= tol ) {
				changed = 1;
				// evaluate
				_Complex float x = roots[i] + coeffs[0];
				for(int j = 1; j < 8; ++j) {
					x = roots[i] * x + coeffs[j];
				}
				for(int j = 0; j < 8; ++j) {
					if( i != j ) {
						if( roots[i] != roots[j] ) {
							x /= (roots[i] - roots[j]);
						}
					}
				}
				roots[i] -= x;
				err[i] = fabs(x);
			}
		}
		if( !changed ) {
			break;
		}
	}

	*numroots = 0;
	int visited[8] = {0};
	for(int i = 0; i < 8; ++i) {
		if( !visited[i] ) {
			// might be a multiple root, in which case it will have more error than the other roots
			// find any neighboring roots, and take the average
			_Complex float newroot=roots[i];
			int n = 1;
			for(int j = i+1; j < 8; ++j) {
				// care about error in real much more than imaginary
				if( fabs(creal(roots[i])-creal(roots[j])) < tolsqrt && fabs(cimag(roots[i])-cimag(roots[j])) < 0.002f ) {
					newroot += roots[j];
					n += 1;
					visited[j] = 1;
				}
			}
			if( n > 1 ) {
				newroot /= n;
			}
			// there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
			if( fabs(cimag(newroot)) < tolsqrt ) {
				rawroots[(*numroots)++] = creal(newroot);
			}
		}
	}
}

#define IK2PI  ((float)6.28318530717959f)
#define IKPI  ((float)3.14159265358979f)
#define IKPI_2  ((float)1.57079632679490f)

inline float IKabs(float f) { return fabsf(f); }

inline float IKsqr(float f) { return f*f; }

inline float IKlog(float f) { return logf(f); }

inline float fsgn(float f) { return f > 0 ? 1.0f : -1.0f; }

// allows asin and acos to exceed 1. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_SINCOS_THRESH
#define IKFAST_SINCOS_THRESH ((float)1e-7)
#endif

// used to check input to atan2 for degenerate cases. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_ATAN2_MAGTHRESH
#define IKFAST_ATAN2_MAGTHRESH ((float)1e-7)
#endif

// minimum distance of separate solutions
#ifndef IKFAST_SOLUTION_THRESH
#define IKFAST_SOLUTION_THRESH ((float)1e-6)
#endif

// there are checkpoints in ikfast that are evaluated to make sure they are 0. This threshold speicfies by how much they can deviate
#ifndef IKFAST_EVALCOND_THRESH
#define IKFAST_EVALCOND_THRESH ((float)0.00001f)
#endif

float j0_,cj0,sj0,htj0,j0mul,j1_,cj1,sj1,htj1,j1mul,j2_,cj2,sj2,htj2,j2mul,new_px,px,npx,new_py,py,npy,new_pz,pz,npz,pp;
unsigned char _ij0[2], _nj0,_ij1[2], _nj1,_ij2[2], _nj2;

float j100, cj100, sj100;
unsigned char _ij100[2], _nj100;
int ComputeIk(const float* eetrans, IkSingleDOFSolutionBase (*solutions)[], int *solidxs) {
	j0_=NAN; _ij0[0] = -1; _ij0[1] = -1; _nj0 = -1; j1_=NAN; _ij1[0] = -1; _ij1[1] = -1; _nj1 = -1; j2_=NAN; _ij2[0] = -1; _ij2[1] = -1; _nj2 = -1;
	px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];
	{
		new_px=px;
		new_py=py;
		new_pz=pz;
		px = new_px; py = new_py; pz = new_pz;
		pp=((px*px)+(py*py)+(pz*pz));
		float op[8+1], zeror[8];
		int numroots;
		float x10=((0.39f)*px);
		float x11=((1.0f)*(px*px));
		float x12=((1.0f)*(py*py));
		float x13=((1.0f)*(pz*pz));
		float x14=(x11+x13+x12);
		float x15=((0.146016f)+(((-1.0f)*x14))+x10);
		float x16=((-0.036504f)+(((-1.0f)*x14))+x10);
		float x17=(x10+x14);
		float x18=((0.146016f)+(((-1.0f)*x17)));
		float x19=((-0.036504f)+(((-1.0f)*x17)));
		float gconst0=x15;
		float gconst1=x16;
		float gconst2=x15;
		float gconst3=x16;
		float gconst4=x18;
		float gconst5=x19;
		float gconst6=x18;
		float gconst7=x19;
		float x20=py*py;
		float x21=px*px;
		float x22=((2.0f)*gconst0);
		float x23=((1.0f)*gconst1);
		float x24=((1.0f)*gconst4);
		float x25=((2.0f)*gconst1);
		float x26=((1.0f)*gconst0);
		float x27=(gconst5*gconst7);
		float x28=((2.0f)*gconst4);
		float x29=(gconst1*gconst6);
		float x30=(gconst3*gconst4);
		float x31=(gconst2*gconst5);
		float x32=((0.17083872f)*py);
		float x33=(gconst0*gconst3);
		float x34=((0.34167744f)*py);
		float x35=(gconst3*gconst5);
		float x36=((2.0f)*gconst5);
		float x37=(gconst4*gconst6);
		float x38=(gconst2*gconst4);
		float x39=(gconst0*gconst6);
		float x40=(gconst5*gconst6);
		float x41=(gconst4*gconst7);
		float x42=(gconst1*gconst2);
		float x43=(gconst0*gconst2);
		float x44=(gconst0*gconst7);
		float x45=((0.219024f)*x40);
		float x46=(gconst6*x32);
		float x47=(gconst5*x32);
		float x48=((0.1332542016f)*x20);
		float x49=((0.219024f)*x31);
		float x50=((0.219024f)*x29);
		float x51=(gconst2*x32);
		float x52=(gconst1*x32);
		float x53=((0.219024f)*x42);
		float x54=(gconst3*x20);
		float x55=(gconst6*x20);
		float x56=(gconst2*x20);
		float x57=((4.0f)*px*py);
		float x58=((4.0f)*x21);
		float x59=(gconst7*x20);
		float x60=(x20*x41);
		float x61=(x20*x40);
		float x62=(x20*x31);
		float x63=(x20*x30);
		float x64=(x20*x44);
		float x65=(x20*x29);
		float x66=(x20*x42);
		float x67=(x20*x33);
		float x68=(gconst1*x57);
		float x69=(x27*x57);
		float x70=(x37*x57);
		float x71=(x41*x57);
		float x72=(x40*x57);
		float x73=((1.0f)*x20*x27);
		float x74=(x24*x55);
		float x75=(x38*x57);
		float x76=(gconst7*x68);
		float x77=(x35*x57);
		float x78=(x39*x57);
		float x79=(x30*x57);
		float x80=(x44*x57);
		float x81=(x31*x57);
		float x82=(x29*x57);
		float x83=(x23*x59);
		float x84=(x24*x56);
		float x85=(x26*x55);
		float x86=((1.0f)*x20*x35);
		float x87=(gconst3*x68);
		float x88=(x43*x57);
		float x89=(x33*x57);
		float x90=(x42*x57);
		float x91=(x26*x56);
		float x92=(x23*x54);
		float x93=(x51+x52);
		float x94=(x46+x47);
		float x95=(x66+x67);
		float x96=(x60+x61);
		float x97=(x89+x90);
		float x98=(x70+x69);
		float x99=(x88+x87);
		float x100=(x72+x71);
		float x101=(x74+x73+x45);
		float x102=(x53+x91+x92);
		float x103=(x62+x63+x64+x65);
		float x104=(x77+x76+x75+x78);
		float x105=(x79+x82+x80+x81);
		float x106=(x48+x49+x50+x83+x86+x84+x85);
		op[0]=((((-1.0f)*x101))+x96);
		op[1]=(x100+(((-1.0f)*x94))+(((-1.0f)*x98)));
		op[2]=((((-0.438048f)*x40))+(((-1.0f)*x106))+x103+(((-1.0f)*x27*x58))+(((-1.0f)*x37*x58))+(((-1.0f)*x28*x59))+((x40*x58))+(((-1.0f)*x36*x55))+((x41*x58))+(((2.0f)*x20*x27))+((x28*x55)));
		op[3]=((((-1.0f)*gconst5*x34))+(((-1.0f)*gconst6*x34))+(((-1.0f)*x100))+(((-1.0f)*x104))+x105+x98+(((-1.0f)*x93)));
		op[4]=((((-1.0f)*x35*x58))+(((-1.0f)*gconst1*gconst7*x58))+(((-0.438048f)*x31))+(((-1.0f)*x25*x55))+((x44*x58))+(((-1.0f)*x101))+(((-1.0f)*x102))+(((2.0f)*x20*x35))+(((-2.0f)*x62))+(((-1.0f)*x39*x58))+(((-1.0f)*x38*x58))+((x25*x59))+(((-0.438048f)*x29))+x95+x96+(((-0.2665084032f)*x20))+((x31*x58))+(((-1.0f)*x28*x54))+(((-1.0f)*x22*x59))+((x30*x58))+((x28*x56))+((x22*x55))+((x29*x58)));
		op[5]=((((-1.0f)*gconst1*x34))+(((-1.0f)*x105))+x104+x97+(((-1.0f)*x94))+(((-1.0f)*x99))+(((-1.0f)*gconst2*x34)));
		op[6]=((((-1.0f)*x25*x56))+(((-0.438048f)*x42))+(((-1.0f)*x106))+x103+((x25*x54))+((x33*x58))+((x42*x58))+(((-1.0f)*gconst1*gconst3*x58))+(((-1.0f)*x22*x54))+((x22*x56))+(((-1.0f)*x43*x58)));
		op[7]=(x99+(((-1.0f)*x93))+(((-1.0f)*x97)));
		op[8]=((((-1.0f)*x102))+x95);
		polyroots8(op,zeror,&numroots);
		float j0array[8], cj0array[8], sj0array[8], tempj0array[1];
		int numsolutions = 0;
		for(int ij0 = 0; ij0 < numroots; ++ij0)
		{
			float htj0 = zeror[ij0];
			tempj0array[0]=((2.0f)*(atan(htj0)));
			for(int kj0 = 0; kj0 < 1; ++kj0)
			{
				j0array[numsolutions] = tempj0array[kj0];
				if( j0array[numsolutions] > IKPI )
				{
					j0array[numsolutions]-=IK2PI;
				}
				else if( j0array[numsolutions] < -IKPI )
				{
					j0array[numsolutions]+=IK2PI;
				}
				sj0array[numsolutions] = sin(j0array[numsolutions]);
				cj0array[numsolutions] = cos(j0array[numsolutions]);
				numsolutions++;
			}
		}
		int j0valid[8]={1,1,1,1,1,1,1,1};
		_nj0 = 8;
		for(int ij0 = 0; ij0 < numsolutions; ++ij0)
		{
			if( !j0valid[ij0] )
			{
				continue;
			}
			j0_ = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];
			htj0 = tan(j0_/2);

			_ij0[0] = ij0; _ij0[1] = -1;
			for(int iij0 = ij0+1; iij0 < numsolutions; ++iij0)
			{
				if( j0valid[iij0] && fabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && fabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
				{
					j0valid[iij0]=0; _ij0[1] = iij0; break;
				}
			}
			{
				float j2array[1], cj2array[1], sj2array[1];
				int j2valid[1]={0};
				_nj2 = 1;
				float x107=((4.27350427350427f)*px);
				float x108=((4.27350427350427f)*py);
				if( fabs((((cj0*x108))+(((-1.0f)*sj0*x107)))) < IKFAST_ATAN2_MAGTHRESH && fabs(((-0.6f)+(((10.9577032653956f)*(pz*pz)))+(((10.9577032653956f)*(px*px)))+(((-1.0f)*cj0*x107))+(((10.9577032653956f)*(py*py)))+(((-1.0f)*sj0*x108)))) < IKFAST_ATAN2_MAGTHRESH && fabs(IKsqr((((cj0*x108))+(((-1.0f)*sj0*x107))))+IKsqr(((-0.6f)+(((10.9577032653956f)*(pz*pz)))+(((10.9577032653956f)*(px*px)))+(((-1.0f)*cj0*x107))+(((10.9577032653956f)*(py*py)))+(((-1.0f)*sj0*x108))))-1) <= IKFAST_SINCOS_THRESH )
					continue;
				j2array[0]=atan2((((cj0*x108))+(((-1.0f)*sj0*x107))), ((-0.6f)+(((10.9577032653956f)*(pz*pz)))+(((10.9577032653956f)*(px*px)))+(((-1.0f)*cj0*x107))+(((10.9577032653956f)*(py*py)))+(((-1.0f)*sj0*x108))));
				sj2array[0]=sin(j2array[0]);
				cj2array[0]=cos(j2array[0]);
				if( j2array[0] > IKPI )
				{
					j2array[0]-=IK2PI;
				}
				else if( j2array[0] < -IKPI )
				{    j2array[0]+=IK2PI;
				}
				j2valid[0] = 1;
				for(int ij2 = 0; ij2 < 1; ++ij2)
				{
					if( !j2valid[ij2] )
					{
						continue;
					}
					_ij2[0] = ij2; _ij2[1] = -1;
					for(int iij2 = ij2+1; iij2 < 1; ++iij2)
					{
						if( j2valid[iij2] && fabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && fabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
						{
							j2valid[iij2]=0; _ij2[1] = iij2; break;
						}
					}
					j2_ = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
					{
						float evalcond[2];
						evalcond[0]=((((0.234f)*(sin(j2_))))+((px*sj0))+(((-1.0f)*cj0*py)));
						evalcond[1]=((0.054756f)+(((-1.0f)*(px*px)))+(((0.39f)*cj0*px))+(((0.09126f)*(cos(j2_))))+(((0.39f)*py*sj0))+(((-1.0f)*(pz*pz)))+(((-1.0f)*(py*py))));
						if( fabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
						{
							continue;
						}
					}

					{
						float j1eval[3];
						j1eval[0]=((1.0f)+(((1.2f)*cj2)));
						j1eval[1]=((fabs(pz))+(fabs(((-0.195f)+((cj0*px))+((py*sj0))))));
						j1eval[2]=fsgn(((0.195f)+(((0.234f)*cj2))));
						if( fabs(j1eval[0]) < 0.0000010000000000f  || fabs(j1eval[1]) < 0.0000010000000000f  || fabs(j1eval[2]) < 0.0000010000000000f  )
						{
							{
								float j1eval[2];
								float x109=(py*sj0);
								float x110=((6.15384615384615f)*cj2);
								float x111=(cj0*px);
								float x112=((1.0f)+(((1.2f)*cj2)));
								j1eval[0]=x112;
								j1eval[1]=((((-5.12820512820513f)*x111))+(((-5.12820512820513f)*x109))+(((-1.0f)*x110*x111))+x112+(((-1.0f)*x109*x110)));
								if( fabs(j1eval[0]) < 0.0000010000000000f  || fabs(j1eval[1]) < 0.0000010000000000f  )
								{
									{
										float evalcond[2];
										int bgotonextstatement = 1;
										do
										{
											evalcond[0]=((-3.14159265358979f)+(fmod(((3.14159265358979f)+(fabs(((0.833333333333333f)+cj2)))+(fabs(((-1.0f)+(fsgn(sj2)))))), 6.28318530717959f)));
											evalcond[1]=pz;
											if( fabs(evalcond[0]) < 0.0000050000000000f  && fabs(evalcond[1]) < 0.0000050000000000f  )
											{
												bgotonextstatement=0;
												{
													float j1eval[1];
													sj2=0.5527708f;
													cj2=-0.833333333333333f;
													j2_=2.55590708081597f;
													j1eval[0]=((-1.0f)+(((5.12820512820513f)*cj0*px))+(((5.12820512820513f)*py*sj0)));
													if( fabs(j1eval[0]) < 0.0000010000000000f  )
													{
														{
															float evalcond[1];
															int bgotonextstatement = 1;
															do
															{
																float x113=((((26.2984878369494f)*(py*py)))+(((26.2984878369494f)*(px*px))));
																float x119 = x113;
																if(fabs(x119)==0){
																	continue;
																}
																float x114=pow(x119,-0.5f);
																if((x113) < -0.00001f)
																	continue;
																float x115=fabs(sqrt(x113));
																float x116=((5.12820512820513f)*x114);
																float x120=pow(x115,-1);
																if(isnan(x120)){
																	continue;
																}
																float x117=x120;
																if((((1.0f)+(((-1.0f)*(x117*x117))))) < -0.00001f)
																	continue;
																float x118=sqrt(((1.0f)+(((-1.0f)*(x117*x117)))));
																if( (x117) < -1-IKFAST_SINCOS_THRESH || (x117) > 1+IKFAST_SINCOS_THRESH )
																	continue;
																float x121 = atan2((float)(((5.12820512820513f)*px)),(float)(((5.12820512820513f)*py)));
																if(isnan(x121)){
																	continue;
																}
																float gconst24=((asin(x117))+(((-1.0f)*(x121))));
																float gconst25=((((-1.0f)*px*x116*x118))+((py*x116*x117)));
																float gconst26=(((px*x116*x117))+((py*x116*x118)));
																float x122 = atan2((float)(((5.12820512820513f)*px)),(float)(((5.12820512820513f)*py)));
																if(isnan(x122)){
																	continue;
																}
																if((((((26.2984878369494f)*(py*py)))+(((26.2984878369494f)*(px*px))))) < -0.00001f)
																	continue;
																float x123=pow(fabs(sqrt(((((26.2984878369494f)*(py*py)))+(((26.2984878369494f)*(px*px)))))),-1);
																if(isnan(x123)){
																	continue;
																}
																if( (x123) < -1-IKFAST_SINCOS_THRESH || (x123) > 1+IKFAST_SINCOS_THRESH )
																	continue;
																evalcond[0]=((-3.14159265358979f)+(fmod(((3.14159265358979f)+(fabs(((x122)+(((-1.0f)*(asin(x123))))+j0_)))), 6.28318530717959f)));
																if( fabs(evalcond[0]) < 0.0000050000000000f  )
																{
																	bgotonextstatement=0;
																	{
																		float j1array[2], cj1array[2], sj1array[2];
																		int j1valid[2]={0};
																		_nj1 = 2;
																		float x124=((0.39f)*gconst26*px);
																		float x125=((0.39f)*gconst25*py);
																		float x127=pow(((-0.07605f)+x124+x125),-1);
																		if(isnan(x127)){
																			continue;
																		}
																		float x126=x127;
																		cj1array[0]=(((x126*(px*px)))+(((-1.0f)*x124*x126))+(((-1.0f)*x125*x126))+((x126*(py*py)))+(((0.021294f)*x126)));
																		if( cj1array[0] >= -1-IKFAST_SINCOS_THRESH && cj1array[0] <= 1+IKFAST_SINCOS_THRESH )
																		{
																			j1valid[0] = j1valid[1] = 1;
																			j1array[0] = acos(cj1array[0]);
																			sj1array[0] = sin(j1array[0]);
																			cj1array[1] = cj1array[0];
																			j1array[1] = -j1array[0];
																			sj1array[1] = -sj1array[0];
																		}
																		else if( isnan(cj1array[0]) )
																		{
																			// probably any will work
																			j1valid[0] = 1;
																			cj1array[0] = 1; sj1array[0] = 0; j1array[0] = 0;
																		}
																		for(int ij1 = 0; ij1 < 2; ++ij1)
																		{
																			if( !j1valid[ij1] )
																			{
																				continue;
																			}
																			_ij1[0] = ij1; _ij1[1] = -1;
																			for(int iij1 = ij1+1; iij1 < 2; ++iij1)
																			{
																				if( j1valid[iij1] && fabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && fabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
																				{
																					j1valid[iij1]=0; _ij1[1] = iij1; break;
																				}
																			}
																			j1_ = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];
																			{
																				float evalcond[2];
																				float x128=sin(j1_);
																				float x129=cos(j1_);
																				float x130=((1.0f)*gconst26*px);
																				float x131=((1.0f)*gconst25*py);
																				evalcond[0]=((((-1.0f)*x128*x130))+(((-1.0f)*x128*x131))+(((0.195f)*x128)));
																				evalcond[1]=((((-1.0f)*x129*x130))+(((-1.0f)*x129*x131))+(((0.195f)*x129)));
																				if( fabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
																				{
																					continue;
																				}
																			}

																			{
																				IkSingleDOFSolutionBase vinfos;
																				vinfos.sols[0].jointtype = 1;
																				vinfos.sols[0].foffset = j0_;
																				vinfos.sols[0].indices[0] = _ij0[0];
																				vinfos.sols[0].indices[1] = _ij0[1];
																				vinfos.sols[0].maxsolutions = _nj0;
																				vinfos.sols[1].jointtype = 1;
																				vinfos.sols[1].foffset = j1_;
																				vinfos.sols[1].indices[0] = _ij1[0];
																				vinfos.sols[1].indices[1] = _ij1[1];
																				vinfos.sols[1].maxsolutions = _nj1;
																				vinfos.sols[2].jointtype = 1;
																				vinfos.sols[2].foffset = j2_;
																				vinfos.sols[2].indices[0] = _ij2[0];
																				vinfos.sols[2].indices[1] = _ij2[1];
																				vinfos.sols[2].maxsolutions = _nj2;
																				(*solutions)[0] = vinfos;
																				solidxs[0] = 1;
																			}
																		}
																	}

																}
															} while(0);
															if( bgotonextstatement )
															{
																int bgotonextstatement = 1;
																do
																{
																	float x132=((((26.2984878369494f)*(py*py)))+(((26.2984878369494f)*(px*px))));
																	if((x132) < -0.00001f)
																		continue;
																	float x133=fabs(sqrt(x132));
																	float x139 = x132;
																	if(fabs(x139)==0){
																		continue;
																	}
																	float x134=pow(x139,-0.5f);
																	float x140=pow(x133,-1);
																	if(isnan(x140)){
																		continue;
																	}
																	float x135=x140;
																	if((((1.0f)+(((-1.0f)*(x135*x135))))) < -0.00001f)
																		continue;
																	float x136=sqrt(((1.0f)+(((-1.0f)*(x135*x135)))));
																	float x137=((5.12820512820513f)*x134*x135);
																	float x138=((5.12820512820513f)*x134*x136);
																	if( (x135) < -1-IKFAST_SINCOS_THRESH || (x135) > 1+IKFAST_SINCOS_THRESH )
																		continue;
																	float x141 = atan2((float)(((5.12820512820513f)*px)),(float)(((5.12820512820513f)*py)));
																	if(isnan(x141)){
																		continue;
																	}
																	float gconst27=((3.14159265358979f)+(((-1.0f)*(asin(x135))))+(((-1.0f)*(x141))));
																	float gconst28=(((px*x138))+((py*x137)));
																	float gconst29=((((-1.0f)*py*x138))+((px*x137)));
																	float x142 = atan2((float)(((5.12820512820513f)*px)),(float)(((5.12820512820513f)*py)));
																	if(isnan(x142)){
																		continue;
																	}
																	if((((((26.2984878369494f)*(py*py)))+(((26.2984878369494f)*(px*px))))) < -0.00001f)
																		continue;
																	float x143=pow(fabs(sqrt(((((26.2984878369494f)*(py*py)))+(((26.2984878369494f)*(px*px)))))),-1);
																	if(isnan(x143)){
																		continue;
																	}
																	if( (x143) < -1-IKFAST_SINCOS_THRESH || (x143) > 1+IKFAST_SINCOS_THRESH )
																		continue;
																	evalcond[0]=((-3.14159265358979f)+(fmod(((3.14159265358979f)+(fabs(((-3.14159265358979f)+(x142)+(asin(x143))+j0_)))), 6.28318530717959f)));
																	if( fabs(evalcond[0]) < 0.0000050000000000f  )
																	{
																		bgotonextstatement=0;
																		{
																			float j1array[2], cj1array[2], sj1array[2];
																			int j1valid[2]={0};
																			_nj1 = 2;
																			float x144=((0.39f)*gconst28*py);
																			float x145=((0.39f)*gconst29*px);
																			float x147=pow(((-0.07605f)+x144+x145),-1);
																			if(isnan(x147)){
																				continue;
																			}
																			float x146=x147;
																			cj1array[0]=((((-1.0f)*x144*x146))+(((0.021294f)*x146))+(((-1.0f)*x145*x146))+((x146*(px*px)))+((x146*(py*py))));
																			if( cj1array[0] >= -1-IKFAST_SINCOS_THRESH && cj1array[0] <= 1+IKFAST_SINCOS_THRESH )
																			{
																				j1valid[0] = j1valid[1] = 1;
																				j1array[0] = acos(cj1array[0]);
																				sj1array[0] = sin(j1array[0]);
																				cj1array[1] = cj1array[0];
																				j1array[1] = -j1array[0];
																				sj1array[1] = -sj1array[0];
																			}
																			else if( isnan(cj1array[0]) )
																			{
																				// probably any will work
																				j1valid[0] = 1;
																				cj1array[0] = 1; sj1array[0] = 0; j1array[0] = 0;
																			}
																			for(int ij1 = 0; ij1 < 2; ++ij1)
																			{
																				if( !j1valid[ij1] )
																				{
																					continue;
																				}
																				_ij1[0] = ij1; _ij1[1] = -1;
																				for(int iij1 = ij1+1; iij1 < 2; ++iij1)
																				{
																					if( j1valid[iij1] && fabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && fabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
																					{
																						j1valid[iij1]=0; _ij1[1] = iij1; break;
																					}
																				}
																				j1_ = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];
																				{
																					float evalcond[2];
																					float x148=sin(j1_);
																					float x149=cos(j1_);
																					float x150=((1.0f)*gconst28*py);
																					float x151=((1.0f)*gconst29*px);
																					evalcond[0]=((((-1.0f)*x148*x150))+(((-1.0f)*x148*x151))+(((0.195f)*x148)));
																					evalcond[1]=((((0.195f)*x149))+(((-1.0f)*x149*x151))+(((-1.0f)*x149*x150)));
																					if( fabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
																					{
																						continue;
																					}
																				}

																				{
																					IkSingleDOFSolutionBase vinfos;
																					vinfos.sols[0].jointtype = 1;
																					vinfos.sols[0].foffset = j0_;
																					vinfos.sols[0].indices[0] = _ij0[0];
																					vinfos.sols[0].indices[1] = _ij0[1];
																					vinfos.sols[0].maxsolutions = _nj0;
																					vinfos.sols[1].jointtype = 1;
																					vinfos.sols[1].foffset = j1_;
																					vinfos.sols[1].indices[0] = _ij1[0];
																					vinfos.sols[1].indices[1] = _ij1[1];
																					vinfos.sols[1].maxsolutions = _nj1;
																					vinfos.sols[2].jointtype = 1;
																					vinfos.sols[2].foffset = j2_;
																					vinfos.sols[2].indices[0] = _ij2[0];
																					vinfos.sols[2].indices[1] = _ij2[1];
																					vinfos.sols[2].maxsolutions = _nj2;
																					(*solutions)[1] = vinfos;
																					solidxs[1] = 1;
																				}
																			}
																		}

																	}
																} while(0);
																if( bgotonextstatement )
																{
																	int bgotonextstatement = 1;
																	do
																	{
																		if( 1 )
																		{
																			bgotonextstatement=0;
																			continue; // branch miss [j1]

																		}
																	} while(0);
																	if( bgotonextstatement )
																	{
																	}
																}
															}
														}

													} else
													{
														{
															float j1array[2], cj1array[2], sj1array[2];
															int j1valid[2]={0};
															_nj1 = 2;
															float x152=((0.39f)*py*sj0);
															float x153=((0.39f)*cj0*px);
															float x155=pow(((-0.07605f)+x153+x152),-1);
															if(isnan(x155)){
																continue;
															}
															float x154=x155;
															cj1array[0]=((((-1.0f)*x153*x154))+((x154*(py*py)))+(((-1.0f)*x152*x154))+((x154*(px*px)))+(((0.021294f)*x154)));
															if( cj1array[0] >= -1-IKFAST_SINCOS_THRESH && cj1array[0] <= 1+IKFAST_SINCOS_THRESH )
															{
																j1valid[0] = j1valid[1] = 1;
																j1array[0] = acos(cj1array[0]);
																sj1array[0] = sin(j1array[0]);
																cj1array[1] = cj1array[0];
																j1array[1] = -j1array[0];
																sj1array[1] = -sj1array[0];
															}
															else if( isnan(cj1array[0]) )
															{
																// probably any will work
																j1valid[0] = 1;
																cj1array[0] = 1; sj1array[0] = 0; j1array[0] = 0;
															}
															for(int ij1 = 0; ij1 < 2; ++ij1)
															{
																if( !j1valid[ij1] )
																{
																	continue;
																}
																_ij1[0] = ij1; _ij1[1] = -1;
																for(int iij1 = ij1+1; iij1 < 2; ++iij1)
																{
																	if( j1valid[iij1] && fabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && fabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
																	{
																		j1valid[iij1]=0; _ij1[1] = iij1; break;
																	}
																}
																j1_ = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];
																{
																	float evalcond[2];
																	float x156=sin(j1_);
																	float x157=cos(j1_);
																	float x158=((1.0f)*py*sj0);
																	float x159=((1.0f)*cj0*px);
																	evalcond[0]=((((-1.0f)*x156*x159))+(((-1.0f)*x156*x158))+(((0.195f)*x156)));
																	evalcond[1]=((((-1.0f)*x157*x158))+(((-1.0f)*x157*x159))+(((0.195f)*x157)));
																	if( fabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
																	{
																		continue;
																	}
																}

																{
																	IkSingleDOFSolutionBase vinfos;
																	vinfos.sols[0].jointtype = 1;
																	vinfos.sols[0].foffset = j0_;
																	vinfos.sols[0].indices[0] = _ij0[0];
																	vinfos.sols[0].indices[1] = _ij0[1];
																	vinfos.sols[0].maxsolutions = _nj0;
																	vinfos.sols[1].jointtype = 1;
																	vinfos.sols[1].foffset = j1_;
																	vinfos.sols[1].indices[0] = _ij1[0];
																	vinfos.sols[1].indices[1] = _ij1[1];
																	vinfos.sols[1].maxsolutions = _nj1;
																	vinfos.sols[2].jointtype = 1;
																	vinfos.sols[2].foffset = j2_;
																	vinfos.sols[2].indices[0] = _ij2[0];
																	vinfos.sols[2].indices[1] = _ij2[1];
																	vinfos.sols[2].maxsolutions = _nj2;
																	(*solutions)[2] = vinfos;
																	solidxs[2] = 1;
																}
															}
														}

													}

												}

											}
										} while(0);
										if( bgotonextstatement )
										{
											int bgotonextstatement = 1;
											do
											{
												evalcond[0]=((-3.14159265358979f)+(fmod(((3.14159265358979f)+(fabs(((0.833333333333333f)+cj2)))+(fabs(((1.0f)+(fsgn(sj2)))))), 6.28318530717959f)));
												evalcond[1]=pz;
												if( fabs(evalcond[0]) < 0.0000050000000000f  && fabs(evalcond[1]) < 0.0000050000000000f  )
												{
													bgotonextstatement=0;
													{
														float j1eval[1];
														sj2=-0.5527708f;
														cj2=-0.833333333333333f;
														j2_=-2.55590708081597f;
														j1eval[0]=((-1.0f)+(((5.12820512820513f)*cj0*px))+(((5.12820512820513f)*py*sj0)));
														if( fabs(j1eval[0]) < 0.0000010000000000f  )
														{
															{
																float evalcond[1];
																int bgotonextstatement = 1;
																do
																{
																	float x160=((((26.2984878369494f)*(py*py)))+(((26.2984878369494f)*(px*px))));
																	float x166 = x160;
																	if(fabs(x166)==0){
																		continue;
																	}
																	float x161=pow(x166,-0.5f);
																	if((x160) < -0.00001f)
																		continue;
																	float x162=fabs(sqrt(x160));
																	float x163=((5.12820512820513f)*x161);
																	float x167=pow(x162,-1);
																	if(isnan(x167)){
																		continue;
																	}
																	float x164=x167;
																	if((((1.0f)+(((-1.0f)*(x164*x164))))) < -0.00001f)
																		continue;
																	float x165=sqrt(((1.0f)+(((-1.0f)*(x164*x164)))));
																	if( (x164) < -1-IKFAST_SINCOS_THRESH || (x164) > 1+IKFAST_SINCOS_THRESH )
																		continue;
																	float x168 = atan2((float)(((5.12820512820513f)*px)),(float)(((5.12820512820513f)*py)));
																	if(isnan(x168)){
																		continue;
																	}
																	float gconst30=((asin(x164))+(((-1.0f)*(x168))));
																	float gconst31=((((-1.0f)*px*x163*x165))+((py*x163*x164)));
																	float gconst32=(((px*x163*x164))+((py*x163*x165)));
																	float x169 = atan2((float)(((5.12820512820513f)*px)),(float)(((5.12820512820513f)*py)));
																	if(isnan(x169)){
																		continue;
																	}
																	if((((((26.2984878369494f)*(py*py)))+(((26.2984878369494f)*(px*px))))) < -0.00001f)
																		continue;
																	float x170=pow(fabs(sqrt(((((26.2984878369494f)*(py*py)))+(((26.2984878369494f)*(px*px)))))),-1);
																	if(isnan(x170)){
																		continue;
																	}
																	if( (x170) < -1-IKFAST_SINCOS_THRESH || (x170) > 1+IKFAST_SINCOS_THRESH )
																		continue;
																	evalcond[0]=((-3.14159265358979f)+(fmod(((3.14159265358979f)+(fabs(((x169)+(((-1.0f)*(asin(x170))))+j0_)))), 6.28318530717959f)));
																	if( fabs(evalcond[0]) < 0.0000050000000000f  )
																	{
																		bgotonextstatement=0;
																		{
																			float j1array[2], cj1array[2], sj1array[2];
																			int j1valid[2]={0};
																			_nj1 = 2;
																			float x171=((0.39f)*gconst32*px);
																			float x172=((0.39f)*gconst31*py);
																			float x174=pow(((-0.07605f)+x171+x172),-1);
																			if(isnan(x174)){
																				continue;
																			}
																			float x173=x174;
																			cj1array[0]=(((x173*(py*py)))+((x173*(px*px)))+(((0.021294f)*x173))+(((-1.0f)*x171*x173))+(((-1.0f)*x172*x173)));
																			if( cj1array[0] >= -1-IKFAST_SINCOS_THRESH && cj1array[0] <= 1+IKFAST_SINCOS_THRESH )
																			{
																				j1valid[0] = j1valid[1] = 1;
																				j1array[0] = acos(cj1array[0]);
																				sj1array[0] = sin(j1array[0]);
																				cj1array[1] = cj1array[0];
																				j1array[1] = -j1array[0];
																				sj1array[1] = -sj1array[0];
																			}
																			else if( isnan(cj1array[0]) )
																			{
																				// probably any will work
																				j1valid[0] = 1;
																				cj1array[0] = 1; sj1array[0] = 0; j1array[0] = 0;
																			}
																			for(int ij1 = 0; ij1 < 2; ++ij1)
																			{
																				if( !j1valid[ij1] )
																				{
																					continue;
																				}
																				_ij1[0] = ij1; _ij1[1] = -1;
																				for(int iij1 = ij1+1; iij1 < 2; ++iij1)
																				{
																					if( j1valid[iij1] && fabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && fabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
																					{
																						j1valid[iij1]=0; _ij1[1] = iij1; break;
																					}
																				}
																				j1_ = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];
																				{
																					float evalcond[2];
																					float x175=sin(j1_);
																					float x176=cos(j1_);
																					float x177=(gconst31*py);
																					float x178=(gconst32*px);
																					float x179=((1.0f)*x175);
																					float x180=((1.0f)*x176);
																					evalcond[0]=((((-1.0f)*x177*x179))+(((0.195f)*x175))+(((-1.0f)*x178*x179)));
																					evalcond[1]=((((-1.0f)*x178*x180))+(((-1.0f)*x177*x180))+(((0.195f)*x176)));
																					if( fabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
																					{
																						continue;
																					}
																				}

																				{
																					IkSingleDOFSolutionBase vinfos;
																					vinfos.sols[0].jointtype = 1;
																					vinfos.sols[0].foffset = j0_;
																					vinfos.sols[0].indices[0] = _ij0[0];
																					vinfos.sols[0].indices[1] = _ij0[1];
																					vinfos.sols[0].maxsolutions = _nj0;
																					vinfos.sols[1].jointtype = 1;
																					vinfos.sols[1].foffset = j1_;
																					vinfos.sols[1].indices[0] = _ij1[0];
																					vinfos.sols[1].indices[1] = _ij1[1];
																					vinfos.sols[1].maxsolutions = _nj1;
																					vinfos.sols[2].jointtype = 1;
																					vinfos.sols[2].foffset = j2_;
																					vinfos.sols[2].indices[0] = _ij2[0];
																					vinfos.sols[2].indices[1] = _ij2[1];
																					vinfos.sols[2].maxsolutions = _nj2;
																					(*solutions)[3] = vinfos;
																					solidxs[3] = 1;
																				}
																			}
																		}

																	}
																} while(0);
																if( bgotonextstatement )
																{
																	int bgotonextstatement = 1;
																	do
																	{
																		float x181=((((26.2984878369494f)*(py*py)))+(((26.2984878369494f)*(px*px))));
																		if((x181) < -0.00001f)
																			continue;
																		float x182=fabs(sqrt(x181));
																		float x188 = x181;
																		if(fabs(x188)==0){
																			continue;
																		}
																		float x183=pow(x188,-0.5f);
																		float x189=pow(x182,-1);
																		if(isnan(x189)){
																			continue;
																		}
																		float x184=x189;
																		if((((1.0f)+(((-1.0f)*(x184*x184))))) < -0.00001f)
																			continue;
																		float x185=sqrt(((1.0f)+(((-1.0f)*(x184*x184)))));
																		float x186=((5.12820512820513f)*x183*x184);
																		float x187=((5.12820512820513f)*x183*x185);
																		float x190 = atan2((float)(((5.12820512820513f)*px)),(float)(((5.12820512820513f)*py)));
																		if(isnan(x190)){
																			continue;
																		}
																		if( (x184) < -1-IKFAST_SINCOS_THRESH || (x184) > 1+IKFAST_SINCOS_THRESH )
																			continue;
																		float gconst33=((3.14159265358979f)+(((-1.0f)*(x190)))+(((-1.0f)*(asin(x184)))));
																		float gconst34=(((px*x187))+((py*x186)));
																		float gconst35=((((-1.0f)*py*x187))+((px*x186)));
																		float x191 = atan2((float)(((5.12820512820513f)*px)),(float)(((5.12820512820513f)*py)));
																		if(isnan(x191)){
																			continue;
																		}
																		if((((((26.2984878369494f)*(py*py)))+(((26.2984878369494f)*(px*px))))) < -0.00001f)
																			continue;
																		float x192=pow(fabs(sqrt(((((26.2984878369494f)*(py*py)))+(((26.2984878369494f)*(px*px)))))),-1);
																		if(isnan(x192)){
																			continue;
																		}
																		if( (x192) < -1-IKFAST_SINCOS_THRESH || (x192) > 1+IKFAST_SINCOS_THRESH )
																			continue;
																		evalcond[0]=((-3.14159265358979f)+(fmod(((3.14159265358979f)+(fabs(((-3.14159265358979f)+(x191)+(asin(x192))+j0_)))), 6.28318530717959f)));
																		if( fabs(evalcond[0]) < 0.0000050000000000f  )
																		{
																			bgotonextstatement=0;
																			{
																				float j1array[2], cj1array[2], sj1array[2];
																				int j1valid[2]={0};
																				_nj1 = 2;
																				float x193=((0.39f)*gconst35*px);
																				float x194=((0.39f)*gconst34*py);
																				float x196=pow(((-0.07605f)+x193+x194),-1);
																				if(isnan(x196)){
																					continue;
																				}
																				float x195=x196;
																				cj1array[0]=(((x195*(px*px)))+(((-1.0f)*x194*x195))+(((0.021294f)*x195))+((x195*(py*py)))+(((-1.0f)*x193*x195)));
																				if( cj1array[0] >= -1-IKFAST_SINCOS_THRESH && cj1array[0] <= 1+IKFAST_SINCOS_THRESH )
																				{
																					j1valid[0] = j1valid[1] = 1;
																					j1array[0] = acos(cj1array[0]);
																					sj1array[0] = sin(j1array[0]);
																					cj1array[1] = cj1array[0];
																					j1array[1] = -j1array[0];
																					sj1array[1] = -sj1array[0];
																				}
																				else if( isnan(cj1array[0]) )
																				{
																					// probably any will work
																					j1valid[0] = 1;
																					cj1array[0] = 1; sj1array[0] = 0; j1array[0] = 0;
																				}
																				for(int ij1 = 0; ij1 < 2; ++ij1)
																				{
																					if( !j1valid[ij1] )
																					{
																						continue;
																					}
																					_ij1[0] = ij1; _ij1[1] = -1;
																					for(int iij1 = ij1+1; iij1 < 2; ++iij1)
																					{
																						if( j1valid[iij1] && fabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && fabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
																						{
																							j1valid[iij1]=0; _ij1[1] = iij1; break;
																						}
																					}
																					j1_ = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];
																					{
																						float evalcond[2];
																						float x197=sin(j1_);
																						float x198=cos(j1_);
																						float x199=(gconst34*py);
																						float x200=(gconst35*px);
																						float x201=((1.0f)*x197);
																						float x202=((1.0f)*x198);
																						evalcond[0]=((((-1.0f)*x200*x201))+(((0.195f)*x197))+(((-1.0f)*x199*x201)));
																						evalcond[1]=((((-1.0f)*x200*x202))+(((0.195f)*x198))+(((-1.0f)*x199*x202)));
																						if( fabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
																						{
																							continue;
																						}
																					}

																					{
																						IkSingleDOFSolutionBase vinfos;
																						vinfos.sols[0].jointtype = 1;
																						vinfos.sols[0].foffset = j0_;
																						vinfos.sols[0].indices[0] = _ij0[0];
																						vinfos.sols[0].indices[1] = _ij0[1];
																						vinfos.sols[0].maxsolutions = _nj0;
																						vinfos.sols[1].jointtype = 1;
																						vinfos.sols[1].foffset = j1_;
																						vinfos.sols[1].indices[0] = _ij1[0];
																						vinfos.sols[1].indices[1] = _ij1[1];
																						vinfos.sols[1].maxsolutions = _nj1;
																						vinfos.sols[2].jointtype = 1;
																						vinfos.sols[2].foffset = j2_;
																						vinfos.sols[2].indices[0] = _ij2[0];
																						vinfos.sols[2].indices[1] = _ij2[1];
																						vinfos.sols[2].maxsolutions = _nj2;
																						(*solutions)[4] = vinfos;
																						solidxs[4] = 1;
																					}
																				}
																			}

																		}
																	} while(0);
																	if( bgotonextstatement )
																	{
																		int bgotonextstatement = 1;
																		do
																		{
																			if( 1 )
																			{
																				bgotonextstatement=0;
																				continue; // branch miss [j1]

																			}
																		} while(0);
																		if( bgotonextstatement )
																		{
																		}
																	}
																}
															}

														} else
														{
															{
																float j1array[2], cj1array[2], sj1array[2];
																int j1valid[2]={0};
																_nj1 = 2;
																float x203=((0.39f)*py*sj0);
																float x204=((0.39f)*cj0*px);
																float x206=pow(((-0.07605f)+x204+x203),-1);
																if(isnan(x206)){
																	continue;
																}
																float x205=x206;
																cj1array[0]=((((-1.0f)*x204*x205))+(((-1.0f)*x203*x205))+(((0.021294f)*x205))+((x205*(py*py)))+((x205*(px*px))));
																if( cj1array[0] >= -1-IKFAST_SINCOS_THRESH && cj1array[0] <= 1+IKFAST_SINCOS_THRESH )
																{
																	j1valid[0] = j1valid[1] = 1;
																	j1array[0] = acos(cj1array[0]);
																	sj1array[0] = sin(j1array[0]);
																	cj1array[1] = cj1array[0];
																	j1array[1] = -j1array[0];
																	sj1array[1] = -sj1array[0];
																}
																else if( isnan(cj1array[0]) )
																{
																	// probably any will work
																	j1valid[0] = 1;
																	cj1array[0] = 1; sj1array[0] = 0; j1array[0] = 0;
																}
																for(int ij1 = 0; ij1 < 2; ++ij1)
																{
																	if( !j1valid[ij1] )
																	{
																		continue;
																	}
																	_ij1[0] = ij1; _ij1[1] = -1;
																	for(int iij1 = ij1+1; iij1 < 2; ++iij1)
																	{
																		if( j1valid[iij1] && fabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && fabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
																		{
																			j1valid[iij1]=0; _ij1[1] = iij1; break;
																		}
																	}
																	j1_ = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];
																	{
																		float evalcond[2];
																		float x207=sin(j1_);
																		float x208=cos(j1_);
																		float x209=((1.0f)*py*sj0);
																		float x210=((1.0f)*cj0*px);
																		evalcond[0]=((((0.195f)*x207))+(((-1.0f)*x207*x210))+(((-1.0f)*x207*x209)));
																		evalcond[1]=((((0.195f)*x208))+(((-1.0f)*x208*x209))+(((-1.0f)*x208*x210)));
																		if( fabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
																		{
																			continue;
																		}
																	}

																	{
																		IkSingleDOFSolutionBase vinfos;
																		vinfos.sols[0].jointtype = 1;
																		vinfos.sols[0].foffset = j0_;
																		vinfos.sols[0].indices[0] = _ij0[0];
																		vinfos.sols[0].indices[1] = _ij0[1];
																		vinfos.sols[0].maxsolutions = _nj0;
																		vinfos.sols[1].jointtype = 1;
																		vinfos.sols[1].foffset = j1_;
																		vinfos.sols[1].indices[0] = _ij1[0];
																		vinfos.sols[1].indices[1] = _ij1[1];
																		vinfos.sols[1].maxsolutions = _nj1;
																		vinfos.sols[2].jointtype = 1;
																		vinfos.sols[2].foffset = j2_;
																		vinfos.sols[2].indices[0] = _ij2[0];
																		vinfos.sols[2].indices[1] = _ij2[1];
																		vinfos.sols[2].maxsolutions = _nj2;
																		(*solutions)[5] = vinfos;
																		solidxs[5] = 1;
																	}
																}
															}

														}

													}

												}
											} while(0);
											if( bgotonextstatement )
											{
												int bgotonextstatement = 1;
												do
												{
													if( 1 )
													{
														bgotonextstatement=0;
														continue; // branch miss [j1]

													}
												} while(0);
												if( bgotonextstatement )
												{
												}
											}
										}
									}

								} else
								{
									{
										float j1array[1], cj1array[1], sj1array[1];
										int j1valid[1]={0};
										_nj1 = 1;
										float x211=((0.234f)*cj2);
										float x212=(py*sj0);
										float x213=(cj0*px);
										float x214=pow(((0.195f)+x211),-1);
										if(isnan(x214)){
											continue;
										}
										float x215=pow(((0.038025f)+(((-0.195f)*x212))+(((-0.195f)*x213))+(((-1.0f)*x211*x212))+(((-1.0f)*x211*x213))+(((0.04563f)*cj2))),-1);
										if(isnan(x215)){
											continue;
										}
										if( fabs(((-1.0f)*pz*(x214))) < IKFAST_ATAN2_MAGTHRESH && fabs(((x215)*(((-0.038025f)+(((-0.054756f)*(cj2*cj2)))+(pz*pz)+(((-0.09126f)*cj2)))))) < IKFAST_ATAN2_MAGTHRESH && fabs(IKsqr(((-1.0f)*pz*(x214)))+IKsqr(((x215)*(((-0.038025f)+(((-0.054756f)*(cj2*cj2)))+(pz*pz)+(((-0.09126f)*cj2))))))-1) <= IKFAST_SINCOS_THRESH )
											continue;
										j1array[0]=atan2(((-1.0f)*pz*(x214)), ((x215)*(((-0.038025f)+(((-0.054756f)*(cj2*cj2)))+(pz*pz)+(((-0.09126f)*cj2))))));
										sj1array[0]=sin(j1array[0]);
										cj1array[0]=cos(j1array[0]);
										if( j1array[0] > IKPI )
										{
											j1array[0]-=IK2PI;
										}
										else if( j1array[0] < -IKPI )
										{    j1array[0]+=IK2PI;
										}
										j1valid[0] = 1;
										for(int ij1 = 0; ij1 < 1; ++ij1)
										{
											if( !j1valid[ij1] )
											{
												continue;
											}
											_ij1[0] = ij1; _ij1[1] = -1;
											for(int iij1 = ij1+1; iij1 < 1; ++iij1)
											{
												if( j1valid[iij1] && fabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && fabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
												{
													j1valid[iij1]=0; _ij1[1] = iij1; break;
												}
											}
											j1_ = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];
											{
												float evalcond[5];
												float x216=sin(j1_);
												float x217=cos(j1_);
												float x218=((0.234f)*cj2);
												float x219=(py*sj0);
												float x220=((0.195f)*x216);
												float x221=((0.195f)*x217);
												float x222=((1.0f)*cj0*px);
												float x223=((1.0f)*x217);
												float x224=((0.39f)*cj0*px);
												float x225=(pz*x216);
												float x226=((0.195f)+x221);
												evalcond[0]=(((x216*x218))+x220+pz);
												evalcond[1]=(x226+(((-1.0f)*x219))+((x217*x218))+(((-1.0f)*x222)));
												evalcond[2]=((((-1.0f)*x216*x222))+(((-1.0f)*x216*x219))+x220+(((-1.0f)*pz*x223)));
												evalcond[3]=((((-1.0f)*x219*x223))+x218+x225+x226+(((-1.0f)*x217*x222)));
												evalcond[4]=((-0.021294f)+(((-1.0f)*(px*px)))+(((-0.39f)*x225))+(((0.39f)*x217*x219))+(((0.39f)*x219))+x224+(((-0.07605f)*x217))+(((-1.0f)*(pz*pz)))+((x217*x224))+(((-1.0f)*(py*py))));
												if( fabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
												{
													continue;
												}
											}

											{
												IkSingleDOFSolutionBase vinfos;
												vinfos.sols[0].jointtype = 1;
												vinfos.sols[0].foffset = j0_;
												vinfos.sols[0].indices[0] = _ij0[0];
												vinfos.sols[0].indices[1] = _ij0[1];
												vinfos.sols[0].maxsolutions = _nj0;
												vinfos.sols[1].jointtype = 1;
												vinfos.sols[1].foffset = j1_;
												vinfos.sols[1].indices[0] = _ij1[0];
												vinfos.sols[1].indices[1] = _ij1[1];
												vinfos.sols[1].maxsolutions = _nj1;
												vinfos.sols[2].jointtype = 1;
												vinfos.sols[2].foffset = j2_;
												vinfos.sols[2].indices[0] = _ij2[0];
												vinfos.sols[2].indices[1] = _ij2[1];
												vinfos.sols[2].maxsolutions = _nj2;
												(*solutions)[6] = vinfos;
												solidxs[6] = 1;
											}
										}
									}

								}

							}

						} else
						{
							{
								float j1array[1], cj1array[1], sj1array[1];
								int j1valid[1]={0};
								_nj1 = 1;
								float x227=pow(fsgn(((0.195f)+(((0.234f)*cj2)))),-1);
								if(isnan(x227)){
									continue;
								}
								float x228 = atan2((float)(((-1.0f)*pz)),(float)(((-0.195f)+((cj0*px))+((py*sj0)))));
								if(isnan(x228)){
									continue;
								}
								j1array[0]=((-1.5707963267949f)+(((1.5707963267949f)*(x227)))+(x228));
								sj1array[0]=sin(j1array[0]);
								cj1array[0]=cos(j1array[0]);
								if( j1array[0] > IKPI )
								{
									j1array[0]-=IK2PI;
								}
								else if( j1array[0] < -IKPI )
								{    j1array[0]+=IK2PI;
								}
								j1valid[0] = 1;
								for(int ij1 = 0; ij1 < 1; ++ij1)
								{
									if( !j1valid[ij1] )
									{
										continue;
									}
									_ij1[0] = ij1; _ij1[1] = -1;
									for(int iij1 = ij1+1; iij1 < 1; ++iij1)
									{
										if( j1valid[iij1] && fabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && fabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
										{
											j1valid[iij1]=0; _ij1[1] = iij1; break;
										}
									}
									j1_ = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];
									{
										float evalcond[5];
										float x229=sin(j1_);
										float x230=cos(j1_);
										float x231=((0.234f)*cj2);
										float x232=(py*sj0);
										float x233=((0.195f)*x229);
										float x234=((0.195f)*x230);
										float x235=((1.0f)*cj0*px);
										float x236=((1.0f)*x230);
										float x237=((0.39f)*cj0*px);
										float x238=(pz*x229);
										float x239=((0.195f)+x234);
										evalcond[0]=(((x229*x231))+x233+pz);
										evalcond[1]=(((x230*x231))+(((-1.0f)*x235))+x239+(((-1.0f)*x232)));
										evalcond[2]=((((-1.0f)*pz*x236))+x233+(((-1.0f)*x229*x232))+(((-1.0f)*x229*x235)));
										evalcond[3]=((((-1.0f)*x230*x235))+x238+x239+x231+(((-1.0f)*x232*x236)));
										evalcond[4]=((-0.021294f)+(((-1.0f)*(px*px)))+((x230*x237))+x237+(((0.39f)*x232))+(((-0.39f)*x238))+(((-1.0f)*(pz*pz)))+(((-1.0f)*(py*py)))+(((0.39f)*x230*x232))+(((-0.07605f)*x230)));
										if( fabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || fabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
										{
											continue;
										}
									}

									{
										IkSingleDOFSolutionBase vinfos;
										vinfos.sols[0].jointtype = 1;
										vinfos.sols[0].foffset = j0_;
										vinfos.sols[0].indices[0] = _ij0[0];
										vinfos.sols[0].indices[1] = _ij0[1];
										vinfos.sols[0].maxsolutions = _nj0;
										vinfos.sols[1].jointtype = 1;
										vinfos.sols[1].foffset = j1_;
										vinfos.sols[1].indices[0] = _ij1[0];
										vinfos.sols[1].indices[1] = _ij1[1];
										vinfos.sols[1].maxsolutions = _nj1;
										vinfos.sols[2].jointtype = 1;
										vinfos.sols[2].foffset = j2_;
										vinfos.sols[2].indices[0] = _ij2[0];
										vinfos.sols[2].indices[1] = _ij2[1];
										vinfos.sols[2].maxsolutions = _nj2;
										(*solutions)[7] = vinfos;
										solidxs[7] = 1;
									}
								}
							}

						}

					}
				}
			}
		}
	}
	return 0;
}
