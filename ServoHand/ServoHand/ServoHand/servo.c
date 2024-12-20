/*
 * servo.c
 *
 * Created: 10/25/2024 6:26:00 PM
 *  Author: Gunner Cook-Dumas
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "servo.h"

//servo initiation for OC1A
void servoInit1A(void){
	//set pin high as its connected to OC1A
	DDRB |= (1<<DDB1);
	
	//interrupt enable for OC1A and overflow
	TIMSK1 |= (1<<TOIE1) | (1<<OCIE1A);

	//set top value
	ICR1H = (PWM_TOP & 0xFF00)>>8;
	ICR1L = (PWM_TOP & 0x00FF);
	
	//set top value to 50Hz
	OCR1AH = (SERVO_MIN & 0xFF00)>>8;
	OCR1AL = (SERVO_MIN & 0x00FF);
	
	//Set mode=14 enable OC1B pre-scaler 8
	TCCR1A |= (1<<WGM11) | (0<<WGM10) | (1<<COM1A1) | (0<<COM1A0);
	//enable timer starting
	TCCR1B |= (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	
}

//servo initiation for OC1B
void servoInit1B(void){
	//set pin high as its connected to OC1B
	DDRB |= (1<<DDB2);
	
	//interrupt enable for OC1B and overflow
	TIMSK1 |= (1<<TOIE1) | (1<<OCIE1B);

	//set top value
	ICR1H = (PWM_TOP & 0xFF00)>>8;
	ICR1L = (PWM_TOP & 0x00FF);
	
	//set top value to 50Hz
	OCR1BH = (SERVO_MIN & 0xFF00)>>8;
	OCR1BL = (SERVO_MIN & 0x00FF);
	
	//Set mode=14 enable OC1A and OC1B pre-scaler 8
	TCCR1A |= (1<<WGM11) | (0<<WGM10) | (1<<COM1B1) | (0<<COM1B0);
	//enable timer starting
	TCCR1B |= (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	
}

//incomplete OC0A
void servoInit0A(void){
	//PD6 or D6
	DDRD |= (1<<6);
	//fast PWM TOP = OCR0A
	TCCR0A |= (1<<COM0A1) | (0<<COM0A0) | (1<<WGM01) | (1<<WGM00);
	//set clock to 1024 pre-scaler
	TCCR0B |= (0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);
	//interrupt mask for ocie0a and overflow vector
	TIMSK0 |= (1<<OCIE0A) | (1<<TOIE0);
	
}
//!in progress OC0B
void servoInit0B(void){
	//PD6 or D6
	DDRD |= (1<<5);
	TIMSK0 |= (1<<OCIE0B) | (1<<TOIE0);
	//fast PWM TOP = OCR0B
	TCCR0A |= (1<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00);
	//set clock to 1024 pre-scaler
	TCCR0B |= (0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);
	//interrupt mask for ocie0b and overflow vector
}




//!in progress
void updatePWM0A(uint16_t i){
	g_update_pwm0A_ready =1;
	while(g_update_pwm0A_ready != 0);
	OCR0A = (i & 0xFF);
}

void updatePWM0B(uint16_t i){
	g_update_pwm0B_ready =1;
	while(g_update_pwm0B_ready != 0);
	OCR0B = (i & 0xFF);
}

//updates the PWM OC1A for position
void updatePWM1A(uint16_t i){
	g_update_pwm1A_ready = 1;
	while(g_update_pwm1A_ready != 0);
	OCR1AH = (i & 0xFF00)>>8;
	OCR1AL = (i & 0x00FF);
}

//updates the PWM OC1B for position
void updatePWM1B(uint16_t i){
	g_update_pwm1B_ready = 1;
	while(g_update_pwm1B_ready != 0);
	OCR1BH = (i & 0xFF00)>>8;
	OCR1BL = (i & 0x00FF);
}





void servoSet0A(uint16_t deg, uint16_t maxDeg){
	deg=deg;
	deg=(deg*32/180)+16;
	//double dutyCycle = deg/1000;
	//OCR0A=angle;
	updatePWM0A(deg);
}

void servoSet0B(uint16_t deg, uint16_t maxDeg){
	deg=deg;
	deg=(deg*32/180)+16;
	//double dutyCycle = deg/1000;
	//OCR0A=angle;
	updatePWM0B(deg);
}




//takes a value and sets it to 0-180
void servoSet1A(uint16_t deg, uint16_t maxDeg){
	float set = (float)deg/(float)maxDeg;
	set=(((float)SERVO_MAX-(float)SERVO_MIN)*set)+(float)SERVO_MIN;
	updatePWM1A((uint16_t)set);
}

//takes a value and sets it to 0-180
void servoSet1B(uint16_t deg, uint16_t maxDeg){
	float set = (float)deg/(float)maxDeg;
	set=(((float)SERVO_MAX-(float)SERVO_MIN)*set)+(float)SERVO_MIN;
	updatePWM1B((uint16_t)set);
}

ISR(TIMER1_COMPA_vect){
//int temp = g_update_pwm1A_ready;
}
ISR(TIMER1_COMPB_vect){
	
}
ISR(TIMER1_OVF_vect){
	g_update_pwm1A_ready = 0;
	g_update_pwm1B_ready = 0;
}
ISR(TIMER0_COMPA_vect){
	
}
ISR(TIMER0_COMPB_vect){
	
}
ISR(TIMER0_OVF_vect){
	g_update_pwm0A_ready = 0;
	g_update_pwm0B_ready = 0;
}



