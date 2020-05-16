/*
 * as1130.c
 *
 *  Created on: May 14, 2020
 *      Author: derek-lam
 */
#include "stm32f1xx_hal.h"
#include "as1130.h"

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
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

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
