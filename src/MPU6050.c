#include "MPU6050.h"
#include "retarget.h"



#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define SMPRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define GYRO_XOUT_H 0x43
#define ACCEL_XOUT_H 0x3B
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define INT_STATUS 0x3A


/*
 * Sets up the MPU6050 peripheral, loads a basic configuration.
 */
MPU6050_Status MPU6050_Init(void)
{
	Status res;
	uint8_t temp;
	
	#ifdef Init_Periph
		I2C_ConfigurePeripheral(I2C_Periph, 0, I2C_Speed, Enable_Remap);
	#endif
	
	// Set the clock source to PLL with X axis gyroscope reference as indicated in manual
	res = I2C_WriteRegByte(I2C_Periph, PWR_MGMT_1, 0x01, MPU6050_ADR);
	
	printf("result:%d\r\n", res);
	// Configure gyro and accelerometer
	// Disable FSYNC & set accelerometer and gyro filter
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	res &= I2C_WriteRegByte(I2C_Periph, CONFIG, 0x04, MPU6050_ADR);
	
	
	// Set the sample rate 
	// sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	res &= I2C_WriteRegByte(I2C_Periph, SMPRT_DIV, 0x09, MPU6050_ADR);
	
	// Set gyroscope full scale range to 250 deg/s
	res &= I2C_ReadRegByte(I2C_Periph,  GYRO_CONFIG, &temp,  MPU6050_ADR);
	temp = (temp & 0x07) | (0x00) << 3; // select scale in this case 00
	res &= I2C_WriteRegByte(I2C_Periph,  GYRO_CONFIG, temp,  MPU6050_ADR);
	
	
	// Set the accelerometer configuration to 
	res &= I2C_ReadRegByte(I2C_Periph,  ACCEL_CONFIG, &temp,  MPU6050_ADR);
	temp = (temp & 0x07) | (0x00) << 3; // select scale in this case 00
	res &= I2C_WriteRegByte(I2C_Periph,  ACCEL_CONFIG, temp,  MPU6050_ADR);

	// Enable interrupt 	
	res &= I2C_ReadRegByte(I2C_Periph,  INT_ENABLE, &temp,  MPU6050_ADR);
	temp = (temp & 0xE6) | (0x01);
	res &= I2C_WriteRegByte(I2C_Periph, INT_ENABLE, temp,  MPU6050_ADR);
		
		
	if(res == I2C_ERROR)
		return MPU6050_Error;
	else 
		return MPU6050_Ok;
}

/*
 * Reads the GYRO_[]OUT_H and GYRO_[]OUT_L and combines the data into 
 * a single int16_t value for each axis.
 */

MPU6050_Status MPU6050_GetRawGyroData(int16_t dest[3])
{
	uint8_t data[6];
	Status res = I2C_ReadMultipleBytes(I2C_Periph, GYRO_XOUT_H, data, 6, MPU6050_ADR);
	// combine the MSB and LSB into a signed 16-bit value
	dest[0] = (int16_t)(((int16_t)data[0]<<8) | data[1]);
	dest[1] = (int16_t)(((int16_t)data[2]<<8) | data[3]);
	dest[2] = (int16_t)(((int16_t)data[4]<<8) | data[5]);
	
	if(res == I2C_ERROR)
		return MPU6050_Error;
	else 
		return MPU6050_Ok;
}

/*
 * Reads the ACCEL_[]OUT_H and ACCEL_[]OUT_L and combines the data into 
 * a single int16_t value for each axis.
 */

MPU6050_Status MPU6050_GetRawAccelData(int16_t dest[3])
{
	uint8_t data[6];
	Status res = I2C_ReadMultipleBytes(I2C_Periph, ACCEL_XOUT_H, data, 6, MPU6050_ADR);
	// combine the MSB and LSB into a signed 16-bit value
	dest[0] = (int16_t)(((int16_t)data[0]<<8) | data[1]);
	dest[1] = (int16_t)(((int16_t)data[2]<<8) | data[3]);
	dest[2] = (int16_t)(((int16_t)data[4]<<8) | data[5]);
	
	if(res == I2C_ERROR)
		return MPU6050_Error;
	else 
		return MPU6050_Ok;
}



/**
	*	 Clear MPU6050 Data ready interrupt bit
	*/

MPU6050_Status MPU6050_ClearDataReady()
{
	uint8_t temp;
	Status res = I2C_ReadRegByte(I2C_Periph,  INT_STATUS, &temp,  MPU6050_ADR);
	
	if(res == I2C_ERROR)
		return MPU6050_Error;
	else 
		return MPU6050_Ok;
	
}
