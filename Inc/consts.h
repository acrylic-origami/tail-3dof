/*
 * consts.h
 *
 *  Created on: Mar 26, 2020
 *      Author: derek-lam
 */

#ifndef CONSTS_H_
#define CONSTS_H_

#define AS1130_ADDR             0b01101110
#define AS1130_ADDR_W           (AS1130_ADDR_R | 1)
#define AS1130_ADDR_R           AS1130_ADDR
#define AS1130_REG_SEL_ADDR     0xFD
#define AS1130_ONOFF_FR0_REG    0x01
#define AS1130_PWM_SET0_REG     0x40
#define AS1130_DATA_LOAD_ADDR   0x18
#define PX_SIZE 24 // 24 data frames of 8/3 LEDs each, 12 rows
// parsed into this strange format by main, not interrupt

// Physics, time and scale
// TIM2 ticks per ms
#define TIM2_MS 100
// 20ms * 8M / 13 (aka TIM3_PSC)
#define TIM3_BASE_STEPS 12308
// 51 (gear ratio) * 200 (steps/rev) / 256 (nat units) * 256 (division scaling)
#define NAT_TO_STEP_256 10200

#endif /* CONSTS_H_ */
