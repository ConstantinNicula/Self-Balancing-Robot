#include "LSM6DS33.h"
#include "I2CRoutines.h"
#include "retarget.h"

#define CTRL1_XL 0x10
#define CTRL2_G  0x11
#define CTRL3_C  0x12
#define OUTX_L_G 0x22
#define OUTX_L_XL 0x28
#define STATUS_REG 0x1E


/*
 * Sets up the LSM6DS33 peripheral, loads a basic configuration.
 */
LSM6DS33_Status LSM6DS33_Init(void)
{
//	uint8_t reg;
	// configure accelerometer  
	// scale to +- 4g FS_XL[1:0] = 10b
  // set otput data rate to 100Hz  ODR_XL[3:0] = 0100b	
	// the bandwidth is set to 50Hz 
	Status res = I2C_WriteRegByte(I2C1, CTRL1_XL, 0x48, LSM6DS33_ADR);
	
	// configure gyroscope 
	// set scale to 2000 dps  FS_G[1:0] = 11
	// set otput data rate to 100Hz ODR_G[3:0] = 0100b
	res &= I2C_WriteRegByte(I2C1, CTRL2_G, 0x4C, LSM6DS33_ADR);
	
	
//// set block data output
//res &= I2C_ReadRegByte(I2C1,  CTRL3_C, &reg, LSM6DS33_ADR);
//// set BDU bit
//reg |= (1<<6);
//// write control rgister
//res &= I2C_WriteRegByte(I2C1, CTRL3_C, reg, LSM6DS33_ADR);
	
	
	
	if(res == I2C_ERROR)
		return LSM6DS33_Error;
	else 
		return LSM6DS33_Ok;
}

/*
 * Reads the OUT[]_L_G and OUT[]_H_G and combines the data into 
 * a single int16_t value for each axis.
 */

LSM6DS33_Status LSM6DS33_GetRawGyroData(int16_t* dest)
{
	uint8_t data[6];
	Status res = I2C_ReadMultipleBytes(I2C1, OUTX_L_G, data, 6, LSM6DS33_ADR);
	// combine the MSB and LSB into a signed 16-bit value
	dest[0] = (int16_t)(((int16_t)data[1]<<8) | data[0]);
	dest[1] = (int16_t)(((int16_t)data[3]<<8) | data[2]);
	dest[2] = (int16_t)(((int16_t)data[5]<<8) | data[4]);
	
	if(res == I2C_ERROR)
		return LSM6DS33_Error;
	else 
		return LSM6DS33_Ok;
}

/*
 * Reads the ACCEL_[]OUT_H and ACCEL_[]OUT_L and combines the data into 
 * a single int16_t value for each axis.
 */

LSM6DS33_Status LSM6DS33_GetRawAccelData(int16_t* dest)
{
	uint8_t data[6];
	Status res = I2C_ReadMultipleBytes(I2C1, OUTX_L_XL, data, 6, LSM6DS33_ADR);
	// combine the MSB and LSB into a signed 16-bit value
	dest[0] = (int16_t)(((int16_t)data[1]<<8) | data[0]);
	dest[1] = (int16_t)(((int16_t)data[3]<<8) | data[2]);
	dest[2] = (int16_t)(((int16_t)data[5]<<8) | data[4]);
	
	if(res == I2C_ERROR)
		return LSM6DS33_Error;
	else 
		return LSM6DS33_Ok;
}

/*
 * Checks status register to see if GDA and XLDA bits are high indicating
 * new data can be read.
 */

uint8_t LSM6DS33_NewDataAvailable()
{
	uint8_t regVal;
	Status res = I2C_ReadRegByte(I2C1, STATUS_REG, &regVal, LSM6DS33_ADR);
	//printf("%x\r\n", regVal);
	if(res == I2C_ERROR)
		return 0;
	else 
		return ((regVal&0x3) == 0x3);
}
