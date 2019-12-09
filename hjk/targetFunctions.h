/*
 * targetFunctions.h
 *
 *  Created on: Oct 10, 2019
 *      Author: jdoe8
 */
#include "stdint.h"


#ifndef TARGETFUNCTIONS_H_
#define TARGETFUNCTIONS_H_

#define pinNumb 11

void initADC(void);
void initGPIO(void);
void ADCoutputLED(uint32_t dataIn);
int32_t ADCvalue();
void adcMeasure();
void temperatureMeasure();
void initGPIO(void);


#endif /* TARGETFUNCTIONS_H_ */
