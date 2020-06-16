/*
 * as1130.h
 *
 *  Created on: May 14, 2020
 *      Author: derek-lam
 */

#ifndef AS1130_H_
#define AS1130_H_

#include <stdint.h>

#define AS1130_SETUP_NUM_STEPS 16
#define AS1130_ADDR            0b01101110
#define NUM_PXS 132
void AS1130_Init(void);
uint8_t AS1130_blit(uint8_t *A, uint16_t ncols, uint16_t nrows, uint16_t *loc, uint16_t *scale);
uint8_t AS1130_xy2idx(uint8_t x, uint8_t y);

// hardcoded bitmaps
#define NBUMP 32

#endif /* AS1130_H_ */
