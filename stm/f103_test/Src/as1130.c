/*
 * as1130.c
 *
 *  Created on: May 14, 2020
 *      Author: derek-lam
 */
#include "stm32f1xx_hal.h"
#include "as1130.h"
#include "consts.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

const uint8_t AS1130_setup_sizes[AS1130_SETUP_NUM_STEPS] = {
  2, 2, // CONFIG
  2, 2, // CLK
  2, 2, // CURR
  2, 2, // DSP/
  2, 2, // SHDN
  2, 2, // PIC_EN
  2, 25, // ON-OFF
  2, 157, // PX0
}; // configuration problem: can't change frames after either on-off register or pic-en it seems
const uint8_t AS1130_setup[] = {
  0xFD, 0xC0,
  0x06, 0b10100001, // CONFIG

  0xFD, 0xC0,
  0x0B, 0b1110, // CLK

  0xFD, 0xC0,
  0x05, 0x40, // CURR

  0xFD, 0xC0,
  0x04, 0xEB, // DSP

  0xFD, 0xC0,
  0x09, 0b10111, // SHDN

  0xFD, 0xC0,
  0x00, 0x40, // PIC_EN

  0xFD, 0x01, // ON-OFF
  0x00,
    0xFF, 0x07, // select PWM frame 0
    0xFF, 0x07, 0xFF, 0x07, 0xFF, 0x07, 0xFF, 0x07, 0xFF, 0x07, 0xFF, 0x07, 0xFF, 0x07, 0xFF, 0x07, 0xFF, 0x07, 0xFF, 0x07, 0xFF, 0x07,

  0xFD, 0x40,
  0x00, // PX
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // no blink
	// unset all pxs
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,

};
uint8_t e;
void AS1130_Init(void) {
	HAL_Delay(10); // give AS1130 >5ms to setup I2C address (as spec'd)

	uint8_t step = 0;
	for(uint8_t i = 0; i < AS1130_SETUP_NUM_STEPS; i++) {
		e = HAL_I2C_Master_Transmit(&hi2c1, AS1130_ADDR, &(AS1130_setup[step]), AS1130_setup_sizes[i], -1);
		step += AS1130_setup_sizes[i];
	}
}

const uint8_t AS1130_xy2idx_lookup[ARRAY_ROWS][ARRAY_COLS] = {
		{ 63,52,41,30,119,8 },
		{ 131,120,109,98,87,76 },
		{ 64,53,42,31,20,9 },
		{ 122,111,100,89,78,67 },
		{ 55,44,33,22,11,0 },
		{ 123,112,101,90,79,68 },
		{ 56,45,34,23,12,1 },
		{ 124,113,102,91,80,69 },
		{ 57,46,35,24,13,2 },
		{ 125,114,103,92,81,70 },
		{ 58,47,36,25,14,3 },
		{ 126,115,104,93,82,71 },
};

const uint8_t ARRAY_DIMS[2] = { ARRAY_COLS, ARRAY_ROWS };
uint8_t blit_buf[NUM_PXS + 1];
uint8_t AS1130_blit(uint8_t *A, uint16_t *Adims, uint16_t *loc, uint16_t *scale) {
	// expect row-major A, stride is # cols
	// scale[2] is XY nomalized of the image size, 256-units
	// loc[2] is XY normalized of the array size, 256-units, centered at top left
	// centering is center-aligned, antialiasing is top-left-corner aligned
	if(hi2c1.State == HAL_I2C_STATE_READY) {
		memset(blit_buf, 0, NUM_PXS + 1);
		uint8_t tdims[2];
		int16_t t0[2];
		int16_t tcent[2];
		int16_t dmid_a[2]; // fractions of a-pixels relative to nearest top-left t-pixel, 0~1
		for(uint8_t i = 0; i < 2; i++) {
			tdims[i] = scale[i] * Adims[i] >> _W;
			tcent[i] = (loc[i] * ARRAY_DIMS[i] >> _W);
			t0[i] = tcent[i] - tdims[i] / 2;
			dmid_a[i] = ((loc[i] * ARRAY_DIMS[i] - ((loc[i] * ARRAY_DIMS[i]) & 0xFFFFFF00)) << _W) / scale[i];
		}

		for(int8_t tx = t0[0] + tdims[0]; tx >= t0[0]; tx--) {
			for(int8_t ty = t0[1] + tdims[1]; ty >= t0[1]; ty--) {
				int8_t ts[2] = { tx, ty };
				if(
					tx >= 0 && tx < ARRAY_COLS &&
					ty >= 0 && ty < ARRAY_ROWS
				) {
					int16_t as[2];
					int16_t _go = 1;
					for(uint8_t i = 0; i < 2; i++) {
						as[i] = (((int16_t)ts[i] - tcent[i]) << _W) / scale[i] + Adims[i] / 2 - (dmid_a[i] >> _W);
						if(as[i] < 0 || as[i] >= Adims[i])
							_go = 0;
					}

					if(_go) {
						volatile uint8_t idx = AS1130_xy2idx_lookup[ty][tx]; // AS1130_xy2idx(tx, ty) + 1;
						blit_buf[idx + 1] = A[as[1] * Adims[0] + as[0]];
					}
				}
			}
		}
		blit_buf[0] = 0x18;
		HAL_I2C_Master_Transmit(&hi2c1, AS1130_ADDR, blit_buf, NUM_PXS + 1, -1);
		return 0;
	}
	else {
		return 1;
	}
}

uint8_t AS1130_xy2idx(uint8_t x, uint8_t y) {
	y += 10;
	uint8_t a = y >> 1;
	uint8_t b = ((y & 1) ? 6 : 0) + x;
	return b * 11 + a;
}
