#ifndef _MOTORCONTROL_H_
#define _MOTORCONTROL_H_

#include "stm32f10x.h"


#define PWMfrequency 15000
#define PWMprescalerValue ((uint16_t) (SystemCoreClock / 24000000) - 1 )
#define PWMperiod  ((24000000 / PWMfrequency) - 1)
#define maxPulse PWMperiod



void configMotorControlPins(void);
void configMotorTimerChannels(void);
void setRightMotorDutyCycle(int16_t pulse);
void setLeftMotorDutyCycle(int16_t pulse) ;

void lockLeftMotor(void);
void lockRightMotor(void);


#endif
