/*
 * math_util.c
 *
 *  Created on: May 1, 2020
 *      Author: derek-lam
 */

#include "math_util.h"
#include "consts.h"
#include <stdint.h>
#include <math.h>

uint32_t cantor_pair(uint32_t a, uint32_t b) {
	return (a + b) * (a + b + 1) / 2 + b;
}

uint32_t isqrt(uint32_t a_nInput)
{
	// thanks https://stackoverflow.com/a/1101217/3925507
    uint32_t op  = a_nInput;
    uint32_t res = 0;
    uint32_t one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type


    // "one" starts at the highest power of four <= than the argument.
    while (one > op)
    {
        one >>= 2;
    }

    while (one != 0)
    {
        if (op >= res + one)
        {
            op = op - (res + one);
            res = res +  2 * one;
        }
        res >>= 1;
        one >>= 2;
    }
    return res;
}

float cubic(float a, float b, float c, float d) {
	// find the largest real root if it exists
	// thanks https://stackoverflow.com/questions/27176423/
	if (fabs(a) < EPS) { // Quadratic case, ax^2+bx+c=0
		a = b; b = c; c = d;
		if (fabs(a) < EPS) { // Linear case, ax+b=0
			a = b; b = c;
			if (fabs(a) < EPS) // Degenerate case
				return nan("");
			return -b / a;
		}

		float D = b*b - 4*a*c;
		if(fabs(D) < EPS)
			return -b/(2*a);
		else if (D > 0)
			return (-b+sqrt(D))/(2*a); // , (-b-sqrt(D))/(2*a);
		return nan("");
	}

	// Convert to depressed cubic t^3+pt+q = 0 (subst x = t - b/3a)
	float p = (3*a*c - b*b)/(3*a*a);
	float q = (2*b*b*b - 9*a*b*c + 27*a*a*d)/(27*a*a*a);
	float root;

	if (fabs(p) < EPS) { // p = 0 -> t^3 = -q -> t = -q^1/3
		root = cbrt(-q);
	} else if (fabs(q) < EPS) { // q = 0 -> t^3 + pt = 0 -> t(t^2+p)=0
		if(p < 0)
			root = sqrt(-p); //, -sqrt(-p)];
		else
			root = 0;
	} else {
		float D = q*q/4 + p*p*p/27;
		if (fabs(D) < EPS) {       // D = 0 -> two roots
			root = fmax(-1.5*q/p, 3*q/p);
		} else if (D > 0) {             // Only one real root
			float u = cbrt(-q/2 - sqrt(D));
			root = u - p/(3*u);
		} else {                        // D < 0, three roots, but needs to use complex numbers/trigonometric solution
			// really hope we don't have to end up here
			float u = 2*sqrt(-p/3);
			float t = acos(3*q/p/u)/3;  // D < 0 implies p < 0 and acos argument in [-1..1]
			float k = 2*M_PI/3;
			root = fmax(u*cos(t), fmax(u*cos(t-k), u*cos(t-2*k)));
		}
	}

	// Convert back from depressed cubic
	root -= b/(3*a);

	return root;
}
