#ifndef _CONTROL_H_
#define _CONTROL_H_


#include "stm32f10x.h"


int16_t computePID(float angle, float velocity, float position,  float dt);
void stabilizeMotorOutput(float pwm, float dt);

#endif

