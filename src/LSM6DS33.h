#ifndef _LSM6DS33_H_ 
#define _LSM6DS33_H_

#include "stm32f10x.h"

/*Default device address(AD0 low)*/
#define LSM6DS33_ADR ((0x6B) << 1)

/*
 * Device state, used to signal errors to higher level functions.
 */
 
typedef enum 
{
	LSM6DS33_Error = 0,
	LSM6DS33_Ok = 1
}LSM6DS33_Status;


LSM6DS33_Status LSM6DS33_Init(void);
LSM6DS33_Status LSM6DS33_GetRawGyroData(int16_t* dest);
LSM6DS33_Status LSM6DS33_GetRawAccelData(int16_t* dest);
uint8_t LSM6DS33_NewDataAvailable(void);
#endif
