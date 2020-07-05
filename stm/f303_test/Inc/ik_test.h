/*
 * ik_test.h
 *
 *  Created on: Jun 30, 2020
 *      Author: derek-lam
 */

#ifndef IK_TEST_H_
#define IK_TEST_H_

typedef struct sol {
    float fmul, foffset; ///< joint is fmul*sol[freeind]+foffset
    signed char freeind; ///< if >= 0, mimics another joint
    unsigned char jointtype; ///< joint type, 0x01 is revolute, 0x11 is slider
    unsigned char maxsolutions; ///< max possible indices, 0 if controlled by free index or a free joint itself
    unsigned char indices[5];
} sol_t;
typedef struct IkSingleDOFSolutionBase {
    sol_t sols[3];
} IkSingleDOFSolutionBase;

void ComputeFk(const float* j, float* eetrans);
int ComputeIk(const float* eetrans, IkSingleDOFSolutionBase (*solutions)[], int *solidxs);

#endif /* IK_TEST_H_ */
