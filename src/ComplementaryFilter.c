#include "ComplementaryFilter.h"
#include "math.h"
#include "MadgwickAHRS.h"
#include "retarget.h"
#include "MPU6050.h"

static float lastAngle = 0;
static float angle = 0;

void computeOffsets(uint16_t numSamples) 
{
	// data buffers
	uint16_t i, j;
	int32_t accAcum[3] = {0, 0, 0}, gyroAcum[3] = {0, 0, 0};
	
	
	// delete first 100 measurements.
	while( i < 100 )
	{
		if(dataReady)
		i++;
	}

	i = 0;
	while(i<numSamples)
	{
		if(dataReady)
		{
			// clear data ready bit 
			dataReady = 0;

			// accumulate data
			for (j = 0; j<3; j++)
			{
				accAcum[j] += accData[j];
				gyroAcum[j] += gyroData[j];
			}			
			// increment counter 
			i++;
		}
	}

	gyroOffsets[0] =  gyroAcum[0]/numSamples;
	gyroOffsets[1] =  gyroAcum[1]/numSamples;
	gyroOffsets[2] =  gyroAcum[2]/numSamples;
	
	printf("Accum: ax:%d,ay:%d,ay:%d, gx:%d, gy:%d, gz:%d, angle:90",accAcum[0], accAcum[1],accAcum[2],gyroAcum[0], gyroAcum[1], gyroAcum[2]  );

	
	
	// determine which side is down 
	if (accAcum[0]/numSamples > 5000)
	{		
		// Compute accelrometer and gyro offsets
		accOffsets[0] =  accAcum[0]/numSamples - 16384;
		accOffsets[1] =  accAcum[1]/numSamples;
		accOffsets[2] =  accAcum[2]/numSamples;
		
		printf("offsets: ax:%d,ay:%d,ay:%d, gx:%d, gy:%d, gz:%d, angle:90",accOffsets[0], accOffsets[1],accOffsets[2],gyroOffsets[0], gyroOffsets[1], gyroOffsets[2]  );
	}
	else if (accAcum[0]/numSamples <-5000)
	{
		// Compute accelrometer and gyro offsets
		accOffsets[0] =  accAcum[0]/numSamples + 16384;
		accOffsets[1] =  accAcum[1]/numSamples;
		accOffsets[2] =  accAcum[2]/numSamples;
		printf("offsets: ax:%d,ay:%d,ay:%d, gx:%d, gy:%d, gz:%d, angle:-90",accOffsets[0], accOffsets[1],accOffsets[2],gyroOffsets[0], gyroOffsets[1], gyroOffsets[2]  );
	}

	
}
	

float computeAngle(float dt)
{
	// Compute scaled acceleration 
	float xAcc = (float) (accData[0] - accOffsets[0]) * accScale;
	float yAcc = (float) (accData[1] - accOffsets[1]) * accScale;
	float zAcc = (float) (accData[2] - accOffsets[2]) * accScale;
	
	// Compute scaled angular velocity in deg/sec
	float xGyro = 0.0174533 * (gyroData[0] - gyroOffsets[0]) * gyroScale;
	float yGyro = 0.0174533 * (gyroData[1] - gyroOffsets[1]) * gyroScale;// * 0.0174533;
	float zGyro = 0.0174533 * (gyroData[2] - gyroOffsets[2]) * gyroScale;// * 0.0174533;
	
	MadgwickAHRSupdateIMU(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);
	
	//float pitch = acosf(q0/ sqrt(q0*q0 + q2*q2))*2.0f ;
	float pitch = asinf(2*(q0*q2-q3*q1));
	
//	printf("%f\r\n",pitch*180/3.14159265359);
	return pitch*180/3.14159265359;

}

float complementaryFilter(float dt)
{
	static uint8_t firstEntry = 1;
	
	// Compute scaled acceleration 
	float xAcc = (float) (accData[0] - accOffsets[0]) * accScale;
	float yAcc = (float) (accData[2] - accOffsets[2]) * accScale;
	
	// Compute scaled angular velocity in deg/sec
	float gyro =  (float) (gyroData[1] - gyroOffsets[1]) * gyroScale;
	
	// Aproximate angle from acceleration 
	float accAngleAprox = -atan2(xAcc, yAcc) * 57.295779; 
	
	if(firstEntry)
	{
		lastAngle = accAngleAprox;
		firstEntry = 0;
		return lastAngle;
	}
	else 
	{
		angle = (0.99 *(lastAngle + gyro*dt) )+ (0.01)*accAngleAprox;
		// store current value
		lastAngle = angle;
	}
	
	return angle;
}

void stabilizeAngle(float tol)
{
	uint32_t prevMsTicks, dt;
	
	// wait for data ready
	while(!dataReady){}
	
	// Compute scaled acceleration 
	float xAcc = (float) (accData[0] - accOffsets[0]) * accScale;
	float yAcc = (float) (accData[2] - accOffsets[2]) * accScale;
	
	// Aproximate angle from acceleration 
	lastAngle = -atan2(xAcc, yAcc) * 57.295779; 
	
	// wait for angle to stablize
	do {
		// wait for data ready.
		prevMsTicks = msTicks;
		while(!dataReady){}
		dt = msTicks - prevMsTicks;
			
		// compute new angle 
		angle = complementaryFilter(dt/1000.);		
			
	}while( fabs(angle - lastAngle) > tol);
	

}

void printRawValues()
{
	float xAcc = (float) (accData[0] + 0);// * accScale;
	float yAcc = (float) (accData[1] + 0);// * accScale;
	float zAcc = (float) (accData[2] + 0);// * accScale;
	
	// Compute scaled angular velocity in deg/sec
	float xGyro = (gyroData[0] + 0);// * gyroScale;
	float yGyro = (gyroData[1] + 0);// * gyroScale;// * 0.0174533;
	float zGyro = (gyroData[2] + 0);// * gyroScale;// * 0.0174533;
	
	printf("RAW! gyro (%.3f, %.3f, %.3f); acc ( %.3f, %.3f, %.3f)\r\n",xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);

}
void printCorrectedValues()
{
	// Compute scaled acceleration 
	float xAcc = (float) (accData[0] - accOffsets[0]);// * accScale;
	float yAcc = (float) (accData[1] - accOffsets[1]);// * accScale;
	float zAcc = (float) (accData[2] - accOffsets[2]);// * accScale;
	
	// Compute scaled angular velocity in deg/sec
	float xGyro = (gyroData[0] - gyroOffsets[0]);// * gyroScale;
	float yGyro = (gyroData[1] - gyroOffsets[1]);// * gyroScale;// * 0.0174533;
	float zGyro = (gyroData[2] - gyroOffsets[2]);// * gyroScale;// * 0.0174533;
	printRawValues();
	printf("CORRECTED! gyro (%.3f, %.3f, %.3f); acc ( %.3f, %.3f, %.3f)\r\n",xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);
}



void configureMPU6050()
{
	EXTI_InitTypeDef   EXTI_InitStruct;
	GPIO_InitTypeDef   GPIO_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStruct;
		
	uint8_t status ;
	
	/*Configure interrupt pin*/
	// Enable RCC_GPIOC && AFIO clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);


	// Configure PC9 as input pin
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	

	// Connect EXTI 9 Line to PC9 pin 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource9);
	
	// Configure EXTI 9 Line 
	EXTI_InitStruct.EXTI_Line = EXTI_Line9;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	
	// Configure NVIC with highest priority 
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStruct);
	
	
	/*Configure I2C and set up MPU6050 registers*/
	
	printf("Setting up MPU6050!");
	status = MPU6050_Init();
	if(status ==  MPU6050_Ok)
		printf("Init successful!\r\n");
	else 
	{
		printf("Init failed!\r\n");
		while(1)
		{
		
		}
	}
	
	
	
}

void EXTI9_5_IRQHandler(void)
{

	if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
		uint8_t status;
		// Clear interupt flag 
		status = MPU6050_ClearDataReady();
		
		status &= MPU6050_GetRawGyroData(gyroData);
		status &=  MPU6050_GetRawAccelData(accData);
		
		// set data Ready flag 
		dataReady = 1;
		
		// Clear EXTI line 3 pending bit
		EXTI_ClearITPendingBit(EXTI_Line9);
	}
}

