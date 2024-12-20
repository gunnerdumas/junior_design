#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#define F_CPU 16000000UL
#define PWM_TOP (39999u)
#define SERVO_MIN (1999u)
#define SERVO_MAX (4999u)
volatile static uint8_t g_update_pwm2B_ready = 0;
volatile static uint8_t g_update_pwm0A_ready = 0;
volatile static uint8_t g_update_pwm0B_ready = 0;
volatile static uint8_t g_update_pwm1A_ready = 0;
volatile static uint8_t g_update_pwm1B_ready = 0;

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

void servoInit0A(void){
	//PD6 or D6
	DDRD |= (1<<6);
	//fast PWM TOP = OCR0A
	TCCR0A |= (1<<COM0A1) | (0<<COM0A0) | (1<<WGM01) | (1<<WGM00);
	//set clock to 128 pre-scaler
	TCCR0B |= (0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);
	//interrupt mask for ocie0a and overflow vector
	TIMSK0 |= (1<<OCIE0A) | (1<<TOIE0);
	
}

void servoInit0B(void){
	//PD6 or D6
	DDRD |= (1<<5);
	TIMSK0 |= (1<<OCIE0B) | (1<<TOIE0);
	//fast PWM TOP = OCR0B
	TCCR0A |= (1<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00);
	//set clock to 128 pre-scaler	come back and change precaler to 1024
	TCCR0B |= (0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);
	//interrupt mask for ocie0b and overflow vector
}

void servoInit2B(){
	DDRD |= (1<<3);
	//fast pem and top=ocra2B
	TCCR2A |= (1<<COM2B1) | (0<<COM2B0) | (1<<WGM21) | (1<<WGM20);
	TCCR2B |= (0<<WGM22) | (1<<CS22) | (0<<CS21) | (1<<CS20);
	TIMSK2 |= (1<<OCIE2B) | (1<<TOIE2);
}


void updatePWM2B(uint16_t i){
	g_update_pwm2B_ready = 1;
	while(g_update_pwm2B_ready != 0);
	OCR2B = (i & 0xFF);
}

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




void servoSet2B(uint16_t deg, uint16_t maxDeg){
		deg=deg;
		deg=(deg*32/180)+16;
		updatePWM2B(deg);
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

ISR(TIMER2_COMPB_vect){
	
}
ISR(TIMER2_OVF_vect){
	g_update_pwm2B_ready = 0;
}

ISR(TIMER1_COMPA_vect){
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

void adcInit();
void adcPinEnable(uint8_t pin);
void adcPinDisable(uint8_t pin);
void adcPinSelect(uint8_t pin);
uint16_t adcConversion(void);

#define ADC_VOLT(x) ((float)x +0.5)/1024.0 * 5.0;//get voltage of input


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



//1 for ready, 0 for busy
volatile static uint8_t g_ADCConDone=1;

//ADC initiation
void adcInit()
{
	//AVcc with external capacitor at AREF pin
	
	//enable AVcc with external capacitor at AREF pin
	ADMUX = 0x40;
	ADCSRA = 0x9F;
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
	DIDR0 &= ~(1<<pin);
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
	uint32_t tempH=0; //temporary vars for conversion
	uint32_t tempL=0;
	
	//currently converting
	g_ADCConDone=0;
	//start conversion
	ADCSRA |= (1<<ADSC);
	//wait for conversion to complete
	while(g_ADCConDone == 0);
	
	//read adcl and shift over to read adch
	tempL = ADCL;
	tempH = ADCH;

	return(tempH<<8|tempL);
}

int main()
{
	uint16_t  servo_volts[5]={};
	DDRB = 0xFF;
	DDRC = 0x00;//C0-4 enabled as input
	adcInit();
	servoInit2B();
	servoInit1A();
	servoInit1B();
	servoInit0A();
	servoInit0B();
	servoInit0A();
	
	sei();
	servoSet2B(0, 180);
	servoSet1A(0, 180);
	servoSet1B(0, 180);
	servoSet0A(0, 180);
	servoSet0B(0, 180);
	while (1)
	{
		for (int i = 0; i<=4; i++)//loop through each ADC input and place it into its respective servo signal
		{
			adcPinEnable(i);
			adcPinSelect(i);
			uint16_t tempV = adcConversion();
      float volts = (((float)tempV)/1024.0) *5.0;
			volts = volts / (5.0/90.0); 		//will convert the voltage and rotation into a angle
			servo_volts[i]=volts;
			adcPinDisable(i);
		}

		servoSet0A(servo_volts[0], 180);//THUMB
		servoSet0B(servo_volts[0], 180);//POINTer
		servoSet1A(servo_volts[0], 180);//MIDDLE
		servoSet1B(servo_volts[0], 180);//RING
		servoSet2B(servo_volts[0], 180);//PINKY
	
	}
}

