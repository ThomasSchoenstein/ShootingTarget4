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

int32_t i;
int32_t value=12;   //want pin C11

void initGPIO(void){
	//initApp function uses a GPIO command so GPIO is already initialized?
	//initBoard function enables GPIO clock

	GPIO_PinModeSet(gpioPortC, pinNumb, gpioModePushPull, 0);

	GPIO_PinOutClear(gpioPortC, pinNumb);
}

void initADC(void){

	CMU_ClockEnable(cmuClock_ADC0, true);  //enable ADC clock
	CMU_ClockEnable(cmuClock_PRS, true);   //enable PRS clock
	ADC_IntEnable(ADC0, ADC_IEN_SINGLE); //enable ADC interrupts

	ADC_Init_TypeDef adcConfig=ADC_INIT_DEFAULT;                         //initialization variables for ADC
		adcConfig.em2ClockConfig=adcEm2ClockAlwaysOn;
		adcConfig.ovsRateSel=adcOvsRateSel2;
		adcConfig.prescale=ADC_PrescaleCalc(16000000,40000000);
		adcConfig.tailgate=true;
		adcConfig.timebase=ADC_TimebaseCalc(0);
		adcConfig.warmUpMode=adcWarmupKeepADCWarm;

	ADC_Init(ADC0,&adcConfig);  //function to init ADC

	//PINS ARE FOUND ON PAGE 72 OF BGM111 DATA SHEET
	ADC_InitSingle_TypeDef adcSingleConfigPC6=ADC_INITSINGLE_DEFAULT;  //initializaion variables for single conversion on Pin PC6
		adcSingleConfigPC6.posSel=adcPosSelAPORT1XCH6;       //Positive is Pin PC6

		adcSingleConfigPC6.acqTime=adcAcqTime1;   //aquire after 1 clock cycle
		adcSingleConfigPC6.diff=false;            //single ended input
		adcSingleConfigPC6.fifoOverwrite=false;   //excess data is thrown out
		adcSingleConfigPC6.leftAdjust=false;      //right adjusted
		adcSingleConfigPC6.negSel=adcNegSelVSS;   //Negative select is Vss
		adcSingleConfigPC6.prsEnable=false;       //prs is disabled
		adcSingleConfigPC6.prsSel=adcPRSSELCh0;  //prs channel 0
		adcSingleConfigPC6.reference=adcRefVDD;   //referance is VDD should be ~3V
		adcSingleConfigPC6.rep=false;              //will not repeat
		adcSingleConfigPC6.resolution=adcRes8Bit;  //8 bit resolution
		adcSingleConfigPC6.singleDmaEm2Wu=true;    //DMA is enabled when in EM2

/*	ADC_InitSingle_TypeDef adcSingleConfigPC7=ADC_INITSINGLE_DEFAULT;  //initializaion variables for single conversion on Pin PC7
		adcSingleConfigPC7.posSel=adcPosSelAPORT1YCH7;       //Positive is Pin PC7

		adcSingleConfigPC7.acqTime=adcAcqTime1;   //aquire after 1 clock cycle
		adcSingleConfigPC7.diff=false;            //single ended input
		adcSingleConfigPC7.fifoOverwrite=false;   //excess data is thrown out
		adcSingleConfigPC7.leftAdjust=false;      //right adjusted
		adcSingleConfigPC7.negSel=adcNegSelVSS;   //Negative select is Vss
		adcSingleConfigPC7.prsEnable=false;       //prs is disabled
		adcSingleConfigPC7.prsSel=adcPRSSELCh0;  //prs channel 0
		adcSingleConfigPC7.reference=adcRefVDD;   //referance is VDD should be ~3V
		adcSingleConfigPC7.rep=false;              //will not repeat
		adcSingleConfigPC7.resolution=adcRes8Bit;  //8 bit resolution
		adcSingleConfigPC7.singleDmaEm2Wu=true;    //DMA is enabled when in EM2

	ADC_InitSingle_TypeDef adcSingleConfigPC8=ADC_INITSINGLE_DEFAULT;  //initializaion variables for single conversion on Pin C8
		adcSingleConfigPC8.posSel=adcPosSelAPORT1XCH8;       //Positive is Pin C8

		adcSingleConfigPC8.acqTime=adcAcqTime1;   //aquire after 1 clock cycle
		adcSingleConfigPC8.diff=false;            //single ended input
		adcSingleConfigPC8.fifoOverwrite=false;   //excess data is thrown out
		adcSingleConfigPC8.leftAdjust=false;      //right adjusted
		adcSingleConfigPC8.negSel=adcNegSelVSS;   //Negative select is Vss
		adcSingleConfigPC8.prsEnable=false;       //prs is disabled
		adcSingleConfigPC8.prsSel=adcPRSSELCh0;  //prs channel 0
		adcSingleConfigPC8.reference=adcRefVDD;   //referance is VDD should be ~3V
		adcSingleConfigPC8.rep=false;              //will not repeat
		adcSingleConfigPC8.resolution=adcRes8Bit;  //8 bit resolution
		adcSingleConfigPC8.singleDmaEm2Wu=true;    //DMA is enabled when in EM2

	ADC_InitSingle_TypeDef adcSingleConfigPC9=ADC_INITSINGLE_DEFAULT;  //initializaion variables for single conversion on Pin C9
		adcSingleConfigPC9.posSel=adcPosSelAPORT1YCH9;       //Positive is Pin C9

		adcSingleConfigPC9.acqTime=adcAcqTime1;   //aquire after 1 clock cycle
		adcSingleConfigPC9.diff=false;            //single ended input
		adcSingleConfigPC9.fifoOverwrite=false;   //excess data is thrown out
		adcSingleConfigPC9.leftAdjust=false;      //right adjusted
		adcSingleConfigPC9.negSel=adcNegSelVSS;   //Negative select is Vss
		adcSingleConfigPC9.prsEnable=false;       //prs is disabled
		adcSingleConfigPC9.prsSel=adcPRSSELCh0;  //prs channel 0
		adcSingleConfigPC9.reference=adcRefVDD;   //referance is VDD should be ~3V
		adcSingleConfigPC9.rep=false;              //will not repeat
		adcSingleConfigPC9.resolution=adcRes8Bit;  //8 bit resolution
		adcSingleConfigPC9.singleDmaEm2Wu=true;    //DMA is enabled when in EM2

	ADC_InitSingle_TypeDef adcSingleConfigPC10=ADC_INITSINGLE_DEFAULT;  //initializaion variables for single conversion on Pin C10
		adcSingleConfigPC10.posSel=adcPosSelAPORT1XCH10;       //Positive is Pin C10

		adcSingleConfigPC10.acqTime=adcAcqTime1;   //aquire after 1 clock cycle
		adcSingleConfigPC10.diff=false;            //single ended input
		adcSingleConfigPC10.fifoOverwrite=false;   //excess data is thrown out
		adcSingleConfigPC10.leftAdjust=false;      //right adjusted
		adcSingleConfigPC10.negSel=adcNegSelVSS;   //Negative select is Vss
		adcSingleConfigPC10.prsEnable=false;       //prs is disabled
		adcSingleConfigPC10.prsSel=adcPRSSELCh0;  //prs channel 0
		adcSingleConfigPC10.reference=adcRefVDD;   //referance is VDD should be ~3V
		adcSingleConfigPC10.rep=false;              //will not repeat
		adcSingleConfigPC10.resolution=adcRes8Bit;  //8 bit resolution
		adcSingleConfigPC10.singleDmaEm2Wu=true;    //DMA is enabled when in EM2
*/
	ADC_InitSingle(ADC0, &adcSingleConfigPC6);  //conversion init for pin C6
/*	ADC_InitSingle(ADC0, &adcSingleConfigPC7);  //conversion init for pin C7
	ADC_InitSingle(ADC0, &adcSingleConfigPC8);  //conversion init for C8
	ADC_InitSingle(ADC0, &adcSingleConfigPC9);  //conversion init for C9
	ADC_InitSingle(ADC0, &adcSingleConfigPC10);  //conversion init for C10   */
}

int32_t ADCvalue(){
	value=i;

	i++;

/*
	// Start ADC conversion
    ADC_Start(ADC0, adcStartSingle);

    // Wait for conversion to be complete

    CHECK:

    if((ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK)){
    	goto CHECK;
    }
    else if((ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK)){
		value=ADC_DataSingleGet(ADC0);
    }

    GPIO_PinOutToggle(gpioPortC, 11);

    return value;
    */
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
  GPIO_PinOutSet(gpioPortC, 11);

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


