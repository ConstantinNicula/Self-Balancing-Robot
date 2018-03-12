#ifndef _COMPLEMENTARY_FILTER_
#define _COMPLEMENTARY_FILTER_

#include "stm32f10x.h"


extern volatile uint32_t msTicks;

// data read during imu interupt
extern volatile uint8_t dataReady;

// buffers for acceleration and gyro data.
static int16_t accData[3], gyroData[3];


// offsets calculated on startup
static int16_t gyroOffsets[3];
static int16_t accOffsets[3];


#define  accScale (1./16384);
#define  gyroScale  (1./131);


void configureMPU6050(void);


void computeOffsets(uint16_t numSamples) ;
float computeAngle(float dt);
float complementaryFilter(float dt);
void printCorrectedValues(void);
void printRawValues(void);

void stabilizeAngle(float tol);

#endif
