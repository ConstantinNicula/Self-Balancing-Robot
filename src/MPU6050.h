#ifndef _MPU6050_H_ 
#define _MPU6050_H_

#include "stm32f10x.h"
#include "I2CRoutines.h"

/*Default device address(AD0 low)*/
#define MPU6050_ADR ((0x68)<< 1)

/*Refrence to the IC2 peripheral*/
#define I2C_Periph I2C1



/*Uncomment this if I2C1 peripheral should be enable during init routine*/
#define Init_Periph

/*This is only available fore I2C1, only needed for(initPeriph)*/
#define Enable_Remap 1

/*Set the desired speed for the peripheral bus, only needed for(initPeriph)*/
#define I2C_Speed 400000



/*
 * Device state, used to signal errors to higher level functions.
 */
 
typedef enum 
{
	MPU6050_Error = 0,
	MPU6050_Ok = 1
}MPU6050_Status;


MPU6050_Status MPU6050_Init(void);
MPU6050_Status MPU6050_GetRawGyroData(int16_t dest[3]);
MPU6050_Status MPU6050_GetRawAccelData(int16_t dest[3]);
MPU6050_Status MPU6050_ClearDataReady(void);

#endif
