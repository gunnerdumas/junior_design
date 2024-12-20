/*
 * adcConversion.h
 *
 * Created: 10/25/2024 6:46:19 PM
 *  Author: Gunner Cook-Dumas
 */ 


#ifndef ADCCONVERSION_H_
#define ADCCONVERSION_H_

void adcInit();
void adcPinEnable(uint8_t pin);
void adcPinDisable(uint8_t pin);
void adcPinSelect(uint8_t pin);
uint16_t adcConversion(void);

#define ADC_VOLT(x) ( x*(5/1024)) //get voltage of input


enum{
	ADC0PIN,
	ADC1PIN,
	ADC2PIN,
	ADC3PIN,
	ADC4PIN,
	ADC5PIN,
	ADC6PIN,
	ADC7PIN,
	ADC1V1 = 14,
	ADCGND
	};



#endif /* ADCCONVERSION_H_ */