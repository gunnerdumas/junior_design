/*
 * adcConversion.c
 *
 * Created: 10/25/2024 6:40:35 PM
 *  Author: Gunner Cook-Dumas
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "adcConversion.h"

//1 for ready, 0 for busy
volatile static uint8_t g_ADCConDone=1;

//ADC initiation
void adcInit()
{
	//AVcc with external capacitor at AREF pin
	
	//enable AVcc with external capacitor at AREF pin
	ADMUX = 0x40;
	ADCSRA = 0x9B;
	//int stuff = 0;
}

//enable ADC to chosen pin by disabling digital
void adcPinEnable(uint8_t pin)
{
	DIDR0 |= (1<<pin);
}
//disable ADC to chosen pin by enable digital
void adcPinDisable(uint8_t pin)
{
	DIDR0 &= ~(1<<pin);;
}
void adcPinSelect(uint8_t pin)
{
	//clear lower 4 bits while keeping settings
	ADMUX &= 0xF0;
	ADMUX |= pin;
}
//ADC conversion complete
ISR(ADC_vect)
{
	g_ADCConDone = 1;	
}

//ADC conversion
uint16_t adcConversion(void)
{
	uint8_t tempH=0; //temporary vars for conversion
	uint8_t tempL=0;
	
	//currently converting
	g_ADCConDone=0;
	//start conversion
	ADCSRA |= (1<<ADSC);
	//wait for conversion to complete
	while(g_ADCConDone == 0);
	
	//read adcl and shift over to read adch
	tempL = ADCL;
	tempH = ADCH;
	// uint16_t adValue = ADCL;
	// adValue |= (uint16_t)(ADCH << 8);

	return(tempH|tempL);
}


