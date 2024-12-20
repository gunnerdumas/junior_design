/*
 * uart.c
 *
 * Created: 11/11/2024 3:55:01 PM
 *  Author: gunner Cook-Dumas
 */ 
#include "uart.h"

void uartInit(void)
{
	//enable txen
	UCSR0B = 0x04;
	UCSR0C = 0x06;
	UBRR0H = (BRC>>8);
	UBRR0L = BRC;
}
void uartTX(char c)
{
	//while((UCSR0A & 0x10) != 0);
	UDR0=c;
}

