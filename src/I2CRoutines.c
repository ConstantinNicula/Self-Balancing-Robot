#include "I2CRoutines.h"
#include "retarget.h"


/* I2C SPE mask */
#define CR1_PE_Set              ((uint16_t)0x0001)
#define CR1_PE_Reset            ((uint16_t)0xFFFE)

/* I2C START mask */
#define CR1_START_Set           ((uint16_t)0x0100)
#define CR1_START_Reset         ((uint16_t)0xFEFF)

#define CR1_POS_Set           ((uint16_t)0x0800)
#define CR1_POS_Reset         ((uint16_t)0xF7FF)

/* I2C STOP mask */
#define CR1_STOP_Set            ((uint16_t)0x0200)
#define CR1_STOP_Reset          ((uint16_t)0xFDFF)

/* I2C ACK mask */
#define CR1_ACK_Set             ((uint16_t)0x0400)
#define CR1_ACK_Reset           ((uint16_t)0xFBFF)

/* I2C ENARP mask */
#define CR1_ENARP_Set           ((uint16_t)0x0010)
#define CR1_ENARP_Reset         ((uint16_t)0xFFEF)

/* I2C NOSTRETCH mask */
#define CR1_NOSTRETCH_Set       ((uint16_t)0x0080)
#define CR1_NOSTRETCH_Reset     ((uint16_t)0xFF7F)

/* I2C registers Masks */
#define CR1_CLEAR_Mask          ((uint16_t)0xFBF5)

/* I2C DMAEN mask */
#define CR2_DMAEN_Set           ((uint16_t)0x0800)
#define CR2_DMAEN_Reset         ((uint16_t)0xF7FF)

/* I2C LAST mask */
#define CR2_LAST_Set            ((uint16_t)0x1000)
#define CR2_LAST_Reset          ((uint16_t)0xEFFF)

/* I2C FREQ mask */
#define CR2_FREQ_Reset          ((uint16_t)0xFFC0)

/* I2C ADD0 mask */
#define OAR1_ADD0_Set           ((uint16_t)0x0001)
#define OAR1_ADD0_Reset         ((uint16_t)0xFFFE)

/* I2C ENDUAL mask */
#define OAR2_ENDUAL_Set         ((uint16_t)0x0001)
#define OAR2_ENDUAL_Reset       ((uint16_t)0xFFFE)

/* I2C ADD2 mask */
#define OAR2_ADD2_Reset         ((uint16_t)0xFF01)

/* I2C F/S mask */
#define CCR_FS_Set              ((uint16_t)0x8000)

/* I2C CCR mask */
#define CCR_CCR_Set             ((uint16_t)0x0FFF)

/* I2C FLAG mask */
#define FLAG_Mask               ((uint32_t)0x00FFFFFF)

/* I2C Interrupt Enable mask */
#define ITEN_Mask               ((uint32_t)0x07000000)


#define I2C1_TX_DMA_CHANNEL DMA1_Channel6
#define I2C1_RX_DMA_CHANNEL DMA1_Channel7

#define I2C2_TX_DMA_CHANNEL DMA1_Channel4
#define I2C2_RX_DMA_CHANNEL DMA1_Channel5

#define I2C1_DR_Address              0x40005410
#define I2C2_DR_Address              0x40005810

typedef enum {
	I2C_TX_DIR = 0,
	I2C_RX_DIR
}I2C_TRANSFER_DIR;


/*
 * Used for configuring dma stream, only one structure is defined for both peripherals. 
 * This approach is sufficent since the library uses polling to read/write data to the peripherals.
 */
DMA_InitTypeDef I2C_DMA_InitStruct;

/*
 * Reads a buffer of bytes from slave 
 * pBuf - pointer to the buffer 
 * bufSize - number of bytes to read
 * slaveAddress - 7 bit address already shifted
 */


Status I2C_BufferRead(I2C_TypeDef* I2Cx, uint8_t* pBuf,  uint32_t bufSize, uint8_t slaveAddress)
{
	uint32_t timeout = 0, temp;
	
	//Enable I2C errors interrupts
	I2Cx->CR2 |= I2C_IT_ERR;
	if(bufSize > 1)
	{
		// Configure I2Cx DMA channel 
		I2C_ConfigureDMA(I2Cx, pBuf, bufSize, I2C_RX_DIR);
		// Set Last bit to have a NACK on the last received byte 
		I2Cx->CR2 |= CR2_LAST_Set;
		// Enable I2C DMA requests
		I2Cx->CR2 |= CR2_DMAEN_Set;
		timeout = 0xFFFF;
		// Send START condition
		I2Cx->CR1 |= CR1_START_Set;
		// Wait until SB flag is set: EV5
		while ((I2Cx->SR1&0x0001) != 0x0001)
		{
			if (timeout-- == 0)
					return I2C_ERROR;
		}
		timeout = 0xFFFF;
		// Send slave address

		I2Cx->DR = (slaveAddress |OAR1_ADD0_Set);
		// Wait until ADDR is set: EV6 
		while ((I2Cx->SR1&0x0002) != 0x0002)
		{
			if (timeout-- == 0)
					return I2C_ERROR;
		}
		// Clear ADDR flag by reading SR2 register 
		temp = I2Cx->SR2;
		if (I2Cx == I2C1)
		{
			// Wait until DMA end of transfer
			while (!DMA_GetFlagStatus(DMA1_FLAG_TC7));
			// Disable DMA Channel
			DMA_Cmd(I2C1_RX_DMA_CHANNEL, DISABLE);
			// Clear the DMA Transfer Complete flag
			DMA_ClearFlag(DMA1_FLAG_TC7);

		}
		else
		{
			// Wait until DMA end of transfer
			while (!DMA_GetFlagStatus(DMA1_FLAG_TC5));
			// Disable DMA Channel
			DMA_Cmd(I2C2_RX_DMA_CHANNEL, DISABLE);
			// Clear the DMA Transfer Complete flag
			DMA_ClearFlag(DMA1_FLAG_TC5);
		}
			// Program the STOP */
			I2Cx->CR1 |= CR1_STOP_Set;
			// Make sure that the STOP bit is cleared by Hardware before CR1 write access 
			while ((I2Cx->CR1&0x200) == 0x200);
	}
	else 
	{
		timeout = 0xFFFF;
		// Send START condition
		I2Cx->CR1 |= CR1_START_Set;
		// Wait until SB flag is set: EV5
		while ((I2Cx->SR1&0x0001) != 0x0001)
		{
				if (timeout-- == 0)
						return I2C_ERROR;
		}
		// Send slave address 
		I2Cx->DR =( slaveAddress|OAR1_ADD0_Set);
		// Wait until ADDR is set: EV6_3, then program ACK = 0, clear ADDR
		//and program the STOP just after ADDR is cleared. The EV6_3 
		//software sequence must complete before the current byte end of transfer
		// Wait until ADDR is set */
		timeout = 0xFFFF;
		while ((I2Cx->SR1&0x0002) != 0x0002)
		{
				if (timeout-- == 0)
						return I2C_ERROR;
		}
		// Clear ACK bit 
		I2Cx->CR1 &= CR1_ACK_Reset;
		// Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
		// software sequence must complete before the current byte end of transfer 
		__disable_irq();
		//Clear ADDR flag
		temp = I2Cx->SR2;
		// Program the STOP
		I2Cx->CR1 |= CR1_STOP_Set;
		// Re-enable IRQs
		__enable_irq();
		// Wait until a data is received in DR register (RXNE = 1) EV7
		while ((I2Cx->SR1 & 0x00040) != 0x000040);
		// Read the data
		*pBuf = I2Cx->DR;
		// Make sure that the STOP bit is cleared by Hardware before CR1 write access 
		while ((I2Cx->CR1&0x200) == 0x200);
		//Enable Acknowledgement to be ready for another reception
		I2Cx->CR1 |= CR1_ACK_Set;
	
	}
	
	return I2C_Success;
}

/*
 * Writes a buffer of bytes. 
 * pBuf - pointer to the buffer you want to send
 * bufSize - number of bytes to send
 * slaveAddress - 7 bit address already shifted
 */

Status I2C_BufferWrite(I2C_TypeDef* I2Cx, uint8_t* pBuf,  uint32_t bufSize, uint8_t slaveAddress)
{
	uint32_t timeout = 0, temp;
	
	// enable error IT
	I2Cx->CR2 |= I2C_IT_ERR;
	
	// configure DMA channel for transmission
	I2C_ConfigureDMA(I2Cx, pBuf, bufSize, I2C_TX_DIR);
	// enable the I2Cx DMA request
	I2Cx->CR2 |= CR2_DMAEN_Set;
	// send the start command
	I2Cx->CR1 |= CR1_START_Set;
	
	timeout = 0xFFFF;
	// wait for SB flag set
	while((I2Cx->SR1&0x0001) != 0x0001)
	{
		if(timeout-- == 0)
			return I2C_ERROR;
	}
	
	// send slave address 
	I2Cx->DR = (slaveAddress & OAR1_ADD0_Reset);
	
	timeout = 0xFFFF;
	// wait untill ADDR is set
	while ((I2Cx->SR1&0x0002) != 0x0002)
	{
		if (timeout-- == 0)
				return I2C_ERROR;
	}
	
	// clear ADDR flag by reading SR2 register
	temp = I2Cx->SR2;
	if (I2Cx == I2C1)
	{
			// Wait until DMA end of transfer
			while (!DMA_GetFlagStatus(DMA1_FLAG_TC6));
			// Disable the DMA1 Channel 6
			DMA_Cmd(I2C1_TX_DMA_CHANNEL, DISABLE);
			// Clear the DMA Transfer complete flag
			DMA_ClearFlag(DMA1_FLAG_TC6);
	}
	else
	{
			// Wait until DMA end of transfer
			while (!DMA_GetFlagStatus(DMA1_FLAG_TC4));
			// Disable the DMA1 Channel 4
			DMA_Cmd(I2C2_TX_DMA_CHANNEL, DISABLE);
			// Clear the DMA Transfer complete flag
			DMA_ClearFlag(DMA1_FLAG_TC4);
	}
	
	// EV8_2: Wait until BTF is set before programming the STOP
	while ((I2Cx->SR1 & 0x00004) != 0x000004);
	// Program the STOP 
	I2Cx->CR1 |= CR1_STOP_Set;
	// Make sure that the STOP bit is cleared by Hardware
	while ((I2Cx->CR1&0x200) == 0x200);
	
	
	return I2C_Success;
}

/**
 * Initializes peripheral I2Cx and the corresponding GPIO and DMA channel
 */

void I2C_ConfigurePeripheral(I2C_TypeDef* I2Cx, uint16_t ownAddress, uint32_t clockSpeed, uint8_t remapPeriphPins)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	
	
	//Enable GPIO clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	//Enable the DMA1 clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	
	// Set I2Cx configuration
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = ownAddress;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStruct.I2C_ClockSpeed = clockSpeed;
	
	
	// Configure common paramters of DMA for I2Cx
	I2C_DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)0;   // This parameter will be configured durig communication 
	I2C_DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;    // This parameter will be configured durig communication 
	I2C_DMA_InitStruct.DMA_BufferSize = 0xFFFF;            // This parameter will be configured durig communication 
	I2C_DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	I2C_DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	I2C_DMA_InitStruct.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	I2C_DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	I2C_DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	I2C_DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	I2C_DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	
	
	
	if(I2Cx == I2C1)
	{
		// Enable I2C1 clock 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

		if(!remapPeriphPins) 
		{
			
			// Configure SDA(PB7) and SCL(PB6) as Open drain
			GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOB, &GPIO_InitStruct);
			
			GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
			GPIO_Init(GPIOB, &GPIO_InitStruct);
		}
		else 
		{
				
			GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
		
			// Configure SDA(PB8) and SCL(PB9) as Open drain
			GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOB, &GPIO_InitStruct);
			
			GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
			GPIO_Init(GPIOB, &GPIO_InitStruct);
		}
		// Enable I2C1 reset state
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
		// Release I2c1 from reset state
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
		
		// Init I2C1 perripheral
		I2C_Init(I2C1, &I2C_InitStruct);
		
		
		// Configure I2C1 TX DMA Channel
		I2C_DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
		
		DMA_DeInit(I2C1_TX_DMA_CHANNEL);
		DMA_Init(I2C1_TX_DMA_CHANNEL, &I2C_DMA_InitStruct);
		
		// Configure I2C1 RX DMA Channel
		DMA_DeInit(I2C1_RX_DMA_CHANNEL);
		DMA_Init(I2C1_RX_DMA_CHANNEL, &I2C_DMA_InitStruct);
		
	} 
	else 
	{
		// Enable I2C2 clock 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		
		// Configure SDA(PB11) and SCL(PB10) as Open drain
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		// Enable I2C2 reset state
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
		// Release I2C2 from reset state
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);
	
		// Init I2C2 perripheral
		I2C_Init(I2C2, &I2C_InitStruct);
		
		
		// Configure I2C2 TX DMA Channel
		I2C_DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)I2C2_DR_Address;
		
		DMA_DeInit(I2C2_TX_DMA_CHANNEL);
		DMA_Init(I2C2_TX_DMA_CHANNEL, &I2C_DMA_InitStruct);
		
		// Configure I2C2 RX DMA Channel
		DMA_DeInit(I2C2_RX_DMA_CHANNEL);
		DMA_Init(I2C2_RX_DMA_CHANNEL, &I2C_DMA_InitStruct);
	}
	
	
}

/*
 * Initializez DMA channel used by the I2C read/write routines.
*/
void I2C_ConfigureDMA(I2C_TypeDef* I2Cx, uint8_t* pBuf, uint32_t bufSize, uint8_t dir)
{
	
	if(dir == I2C_TX_DIR)
	{
		I2C_DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t) pBuf;
		I2C_DMA_InitStruct.DMA_BufferSize = (uint32_t) bufSize;
		I2C_DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;

		if (I2Cx == I2C1)
		{
			I2C_DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
			DMA_Cmd(I2C1_TX_DMA_CHANNEL, DISABLE);
			DMA_Init(I2C1_TX_DMA_CHANNEL, &I2C_DMA_InitStruct);
			DMA_Cmd(I2C1_TX_DMA_CHANNEL, ENABLE);
		}
		else
		{
			I2C_DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)I2C2_DR_Address;
			DMA_Cmd(I2C2_TX_DMA_CHANNEL, DISABLE);
			DMA_Init(I2C2_TX_DMA_CHANNEL, &I2C_DMA_InitStruct);
			DMA_Cmd(I2C2_TX_DMA_CHANNEL, ENABLE);
		}
	}
	else  // dir RX
	{
		I2C_DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t) pBuf;
		I2C_DMA_InitStruct.DMA_BufferSize = (uint32_t) bufSize;
		I2C_DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
		
		if (I2Cx == I2C1)
		{
			I2C_DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
			DMA_Cmd(I2C1_RX_DMA_CHANNEL, DISABLE);
			DMA_Init(I2C1_RX_DMA_CHANNEL, &I2C_DMA_InitStruct);
			DMA_Cmd(I2C1_RX_DMA_CHANNEL, ENABLE);
		}
		else
		{
			I2C_DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)I2C2_DR_Address;
			DMA_Cmd(I2C2_RX_DMA_CHANNEL, DISABLE);
			DMA_Init(I2C2_RX_DMA_CHANNEL, &I2C_DMA_InitStruct);
			DMA_Cmd(I2C2_RX_DMA_CHANNEL, ENABLE);
		}
	
	}

}

/*
 * Configures a 8bit( 1 byte) register on the slave device
 * reg - register address
 * value - new register value
 * slaveAddress - 7bit (left aligned) salve address
 */

Status I2C_WriteRegByte(I2C_TypeDef* I2Cx, uint8_t reg, uint8_t value, uint8_t slaveAddress)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	return I2C_BufferWrite(I2Cx, data, 2, slaveAddress);
}

/*
 * Read the state of a 8bit( 1 byte) register on the slave device
 * reg - register address
 * value - new register value
 * slaveAddress - 7bit (left aligned) salve address
 */
Status I2C_ReadRegByte(I2C_TypeDef* I2Cx, uint8_t reg, uint8_t* value, uint8_t slaveAddress)
{
	Status res;
	res = I2C_BufferWrite(I2Cx, &reg, 1, slaveAddress);
	res &= I2C_BufferRead(I2Cx, value, 1, slaveAddress);
	return res;
}

/*
 * Read a sequence of bytes starting form the base-address (check if supported by I2C peripheral device before using)
 * reg - register address
 * value - new register value
 * slaveAddress - 7bit (left aligned) salve address
 */
Status I2C_ReadMultipleBytes(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* pBuf, uint32_t bufSize, uint8_t slaveAddress)
{
	Status res;
	res = I2C_BufferWrite(I2Cx, &address, 1, slaveAddress);
	res &= I2C_BufferRead(I2Cx, pBuf, bufSize, slaveAddress);
	return res;
}
