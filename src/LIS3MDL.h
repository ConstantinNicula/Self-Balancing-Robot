#ifndef _LIS3MDL_H_ 
#define _LIS3MDL_H_

#include "stm32f10x.h"

/*Default device address(AD0 low)*/
#define LIS3MDL_ADR ((0x1E) << 1)

/*
 * Device state, used to signal errors to higher level functions.
 */
 
typedef enum 
{
	LIS3MDL_Error = 0,
	LIS3MDL_Ok = 1
}LIS3MDL_Status;


LIS3MDL_Status LIS3MDL_Init(void);
LIS3MDL_Status LIS3MDL_GetRawMagData(int16_t* dest);
uint8_t LIS3MDL_NewDataAvailable(void);
#endif
