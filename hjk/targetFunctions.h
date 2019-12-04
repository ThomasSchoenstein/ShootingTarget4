/*
 * targetFunctions.h
 *
 *  Created on: Oct 10, 2019
 *      Author: jdoe8
 */

#ifndef TARGETFUNCTIONS_H_
#define TARGETFUNCTIONS_H_

void initADC(void);
void initGPIO(void);
void ADCoutputLED(uint32_t dataIn);
int32_t ADCvalue();
void adcMeasure();


#endif /* TARGETFUNCTIONS_H_ */
