/*
 * targetFunctions.c
 *
 *  Created on: Oct 10, 2019
 *      Author: jdoe8
 */

#include "em_system.h"
#include "em_adc.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "hal-config.h"
#include "infrastructure.h"
#include "si7013.h"
#include "native_gecko.h"
#include "gatt_db.h"

#include "delay.h"
#include "targetFunctions.h"

#define adcFreq   16000000

int32_t i;
int32_t value=1;
int32_t t=0;

void initGPIO(void){
	//initApp function uses a GPIO command so GPIO is already initialized?
	//initBoard function enables GPIO clock

	GPIO_PinModeSet(gpioPortC, pinNumb, gpioModePushPull, 0);

	GPIO_PinOutClear(gpioPortC, pinNumb);
}

void initSingleADC(void){

	  // Enable ADC0 clock
	  CMU_ClockEnable(cmuClock_ADC0, true);

	  // Declare init structs
	  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

	  // Modify init structs and initialize
	  init.prescale = ADC_PrescaleCalc(adcFreq, 0); // Init to max ADC clock for Series 1

	  initSingle.diff       = false;        // single ended
	  initSingle.reference  = adcRefVDD;    // internal 2.5V reference
	  initSingle.resolution = adcRes12Bit;  // 12-bit resolution
	  initSingle.acqTime    = adcAcqTime4;  // set acquisition time to meet minimum requirement

	  // Select ADC input. See README for corresponding EXP header pin.
	  initSingle.posSel = adcPosSelAPORT1XCH6;
	  init.timebase = ADC_TimebaseCalc(0);

	  ADC_Init(ADC0, &init);
	  ADC_InitSingle(ADC0, &initSingle);
}

int32_t ADCvalue(){
//	value=i;

//	i++;
    // Start ADC conversion
    ADC_Start(ADC0, adcStartSingle);

    // Wait for conversion to be complete
    while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));

    // Get ADC result
    value = ADC_DataSingleGet(ADC0);


    GPIO_PinOutClear(gpioPortC, pinNumb);

    return value;

}

//This function handles the bluetooth capability and sends out the data read in from ADCvalue()
void adcMeasure(){
  uint8_t ADCBuffer[5]; /* Stores the ADC data in the Health Thermometer (HTM) format. */
uint8_t flags = 0x00;   /* HTM flags set as 0 for Celsius, no time stamp and no temperature type. */
  int32_t adcData;     /* Stores the ADC data read from the Piezo sensor. */
  uint32_t pressure;   /* Stores the ADC data read from the sensor in the correct format */
  uint8_t *p = ADCBuffer; /* Pointer to ADC buffer needed for converting values to bitstream. */

  /* Convert flags to bitstream and append them in the ADC data buffer (ADCBuffer) */
  UINT8_TO_BITSTREAM(p, flags);

  /* Sensor pressure measurement */
  adcData=ADCvalue();

  /* Convert sensor data to correct format */
  pressure = FLT_TO_UINT32(adcData, -3);
  /* Convert pressure to bitstream and place it in the ADC data buffer (ADCBuffer) */
  UINT32_TO_BITSTREAM(p, pressure);

  /* Send indication of the pressure in ADCBuffer to all "listening" clients.
   * This enables the Piezo to display the pressure.
   *  0xFF as connection ID will send indications to all connections. */
  gecko_cmd_gatt_server_send_characteristic_notification(
    0xFF, gattdb_temperature_measurement, 5, ADCBuffer);
}


