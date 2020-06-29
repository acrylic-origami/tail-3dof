/*
 * imath.c
 *
 *  Created on: Mar 30, 2020
 *      Author: derek-lam
 */

#include "imath.h"

const int8_t SIN[64] = {
		3,6,9,12,15,18,21,24,28,31,34,37,40,43,46,48,
		51,54,57,60,63,65,68,71,73,76,78,81,83,85,88,90,
		92,94,96,98,100,102,104,106,108,109,111,112,114,115,117,118,
		119,120,121,122,123,124,124,125,126,126,127,127,127,127,127,127
};
// TODO accidentally generated 127, prepended 0. ACTUALLY regenerate this table
const int8_t ASIN[128] = {
		0,0,1,1,2,3,3,4,5,5,6,7,7,8,8,9,10,10,11,12,12,13,14,14,15,16,16,17,17,18,19,19,20,
		21,21,22,23,23,24,25,25,26,27,27,28,29,29,30,31,32,32,33,34,34,35,36,36,37,38,39,39,40,41,41,42,
		43,44,44,45,46,47,47,48,49,50,51,51,52,53,54,55,55,56,57,58,59,60,61,61,62,63,64,65,66,67,68,69,
		70,71,72,73,74,75,76,77,78,79,81,82,83,84,85,87,88,90,91,93,94,96,98,100,102,104,106,109,112,116,127
};
const int8_t CSC[64] = {
		85,51,36,28,23,19,17,15,13,12,11,10,9,9,8,7,
		7,7,6,6,6,5,5,5,5,5,5,4,4,4,4,4,
		4,4,4,4,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3
};

int8_t isin(int8_t x) {
	if(0 <= x && x < 0x40)
		return SIN[x];
	else if(0x40 <= x)
		return SIN[0x7F - x];
	else if(x < -64)
		return -SIN[x & 0x7F];
	else
		return -SIN[-x - 1];
}
int8_t icos(int8_t x) {
	return isin(x + 64); // take advantage of wraparound
}
int8_t iasin(int8_t x) {
	if(x >= 0)
		return ASIN[x];
	else
		return -ASIN[-x];
}
int8_t iacos(uint8_t x) { // contrast: 0 -> 255 = 0 -> pi
	return iasin(x - 128);
}
int8_t icsc(int8_t x) {
	if(0 <= x && x < 64)
		return CSC[x];
	else if(64 <= x)
		return CSC[127 - x];
	else if(x < -64)
		return -CSC[x + 0x80];
	else
		return -CSC[-x];
}
int8_t isec(int8_t x) {
	return icsc(x + 64); // take advantage of wraparound
}
