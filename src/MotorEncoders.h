#ifndef _MOTORENCODERS_H_
#define _MOTORENCODERS_H_

#include "stm32f10x.h"


#define SAMPLE_FREQ 10 //Hz
// 2*pi*4/(4*330)
#define SCALING_FACTOR 0.0190399555


extern volatile uint32_t msTicks;

// direct readings from encoder 
extern int16_t leftCount;
extern int32_t leftTotal;
extern int16_t rightCount;
extern int32_t rightTotal;


// velocity and displacement in [m] and [m/s]
extern float velocityLeft;
extern float velocityRight;

extern float displacementLeft;
extern float displacementRight;


void configMotorEncoder(void);
void readEncoders(void);


#endif

