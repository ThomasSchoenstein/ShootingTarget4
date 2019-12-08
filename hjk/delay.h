/*
 * delay.h
 *
 *  Created on: Dec 7, 2019
 *      Author: jdoe8
 */

#include "stdint.h"

#ifndef DELAY_H_
#define DELAY_H_

// counts 1ms timeTicks
volatile uint32_t msTicks;


void Delay(uint32_t dlyTicks);
void SysTick_Handler(void);

#endif /* DELAY_H_ */
