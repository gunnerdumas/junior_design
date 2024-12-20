/*
 * servo1A.h
 *
 * Created: 10/19/2024 10:06:35 AM
 *  Author: Gunner Cook-Dumas
 */ 


#ifndef SERVO1A_H_
#define SERVO1A_H_

void servoInit1A(void);
void servoInit1B(void);
void servoInit0A(void);
void servoInit0B(void);


void updatePWM0A(uint16_t);
void updatePWM0B(uint16_t);
void updatePWM1A(uint16_t);
void updatePWM1B(uint16_t);

void servoSet0A(uint16_t, uint16_t);
void servoSet0B(uint16_t, uint16_t);
void servoSet1A(uint16_t, uint16_t);
void servoSet1B(uint16_t, uint16_t);


//definitions
#define F_CPU 16000000UL
#define PWM_TOP (39999u)
#define SERVO_MIN (1999u)
#define SERVO_MAX (4999u)

//pwm ready bit
volatile static uint8_t g_update_pwm0A_ready = 0;
volatile static uint8_t g_update_pwm0B_ready = 0;
volatile static uint8_t g_update_pwm1A_ready = 0;
volatile static uint8_t g_update_pwm1B_ready = 0;

#endif /* SERVO1A_H_ */