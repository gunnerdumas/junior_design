/*
 * uart.h
 *
 * Created: 11/11/2024 3:54:51 PM
 *  Author: gtcdu
 */ 


#ifndef UART_H_
#define UART_H_
#include <avr/interrupt.h>
#include <util/delay.h>
#include "servo.h"
#include "adcConversion.h"
#include <stdio.h>

#define F_CPU 16000000
#define BAUD 9600
#define BRC 103

void uartInit(void);
void uartTX(char c);




#endif /* UART_H_ */