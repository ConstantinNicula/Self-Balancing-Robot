#ifndef _I2CROUTINES_H_
#define _I2CROUTINES_H_

#include "stm32f10x.h"

typedef enum {
	I2C_ERROR = 0, 
	I2C_Success 
}Status;

void I2C_ConfigurePeripheral(I2C_TypeDef* I2Cx, uint16_t ownAddress, uint32_t clockSpeed, uint8_t remapPeriphPins);

/*
 * This function should not be called explicitly, it is used by other methods to
 * set up the DMA stream.
 */
void I2C_ConfigureDMA(I2C_TypeDef* I2Cx, uint8_t* pBuf, uint32_t bufSize, uint8_t dir);


Status I2C_BufferWrite(I2C_TypeDef* I2Cx, uint8_t* pBuf,  uint32_t bufSize, uint8_t slaveAddress);
Status I2C_BufferRead(I2C_TypeDef* I2Cx, uint8_t* pBuf,  uint32_t bufSize, uint8_t slaveAddress);

Status I2C_WriteRegByte(I2C_TypeDef* I2Cx, uint8_t reg, uint8_t value, uint8_t slaveAddress);
Status I2C_ReadRegByte(I2C_TypeDef* I2Cx, uint8_t reg, uint8_t* value, uint8_t slaveAddress);


Status I2C_ReadMultipleBytes(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* pBuf, uint32_t bufSize, uint8_t slaveAddress); 

#endif
