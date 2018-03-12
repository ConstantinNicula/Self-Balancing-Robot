#include "LIS3MDL.h"
#include "I2CRoutines.h"
#include "retarget.h"

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

#define STATUS_REG 0x27
#define OUT_X_L 0x28



/*
 * Sets up the LIS3MDL peripheral, loads a basic configuration.
 */
LIS3MDL_Status LIS3MDL_Init(void)
{
	// set magnetometer output data rate to 80Hz
	// set DO[2:0] bits in CTRL_REG1 to 111b
	// set magnetometer XY axis to ultra high performance mode 
	// set OM[1:0] bits in CTRL_REG1 to 11b
	Status res = I2C_WriteRegByte(I2C1, CTRL_REG1, 0x7C, LIS3MDL_ADR);
	
	// set scale to +- 4 gauss 
	res &= I2C_WriteRegByte(I2C1, CTRL_REG2, 0x00, LIS3MDL_ADR);
	
	// set device to continous conversion
	res &= I2C_WriteRegByte(I2C1, CTRL_REG3, 0x00, LIS3MDL_ADR);
	
	//  set magnetometer XY axis to ultra high performance mode 
	res &= I2C_WriteRegByte(I2C1, CTRL_REG4, 0x0C, LIS3MDL_ADR);
	
	if(res == I2C_ERROR)
		return LIS3MDL_Error;
	else 
		return LIS3MDL_Ok;
}

/*
 * Read output data registers from LIS3MDL and convert the low and high
 * registers into a single int16_t value.
 */

LIS3MDL_Status LIS3MDL_GetRawMagData(int16_t* dest)
{
	uint8_t data[6];
	Status res = I2C_ReadMultipleBytes(I2C1, OUT_X_L, data, 6, LIS3MDL_ADR);
	// combine the MSB and LSB into a signed 16-bit value
	dest[0] = (int16_t)(((int16_t)data[1]<<8) | data[0]);
	dest[1] = (int16_t)(((int16_t)data[3]<<8) | data[2]);
	dest[2] = (int16_t)(((int16_t)data[5]<<8) | data[4]);
	
	if(res == I2C_ERROR)
		return LIS3MDL_Error;
	else 
		return LIS3MDL_Ok;
}



/*
 * Checks status register to see if GDA and XLDA bits are high indicating
 * new data can be read.
 */

uint8_t LIS3MDL_NewDataAvailable()
{
	uint8_t regVal;
	Status res = I2C_ReadRegByte(I2C1, STATUS_REG, &regVal, LIS3MDL_ADR);
	
	if(res == I2C_ERROR)
		return 0;
	else 
		return ((regVal&0x07) == 0x07);
}
