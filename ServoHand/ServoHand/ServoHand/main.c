#include <avr/interrupt.h>
#include <util/delay.h>
#include "servo.h"
#include "adcConversion.h"
#include "uart.h"
#include <stdio.h>


uint16_t voltMap180(uint16_t);


int main()
{

	DDRB = 0xFF;
	adcInit();
	adcPinEnable(ADC0PIN);
	adcPinSelect(ADC0PIN);
	sei();
_
	while (1)
	{
		uint16_t tempV = adcConversion();
		float volts = ((float)tempV +0.5)/1024.0 * 5.0;
		if(tempV > 250)
			PORTB = 1<<5;
		else
		PORTB = 0;
	}
}
uint16_t voltMap180(uint16_t deg)
{
	return 0+((deg-0)*(180)/(deg));
}