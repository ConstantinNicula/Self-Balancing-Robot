#include "stm32f10x.h"
#include "retarget.h"
#include "MotorControl.h"
#include "MotorEncoders.h"
#include "ComplementaryFilter.h"
#include "Utils.h"
#include "Control.h"



/*Holds time in milliseconds since the lest reset.*/
volatile uint32_t msTicks;
void delay(uint32_t);

volatile uint8_t dataReady;

int main()
{
	uint32_t prevMsTicks;
	uint16_t dt;
	
	float angle = 0;
	int32_t pwm=0;
	
	// set up retargeting to use printf()	
	InitDebugUSART();
	
	// configure systick to triger every millisecond
	SysTick_Config(SystemCoreClock/1000);
	
	// init motors
	configMotorControlPins();
	configMotorTimerChannels();
	
	setRightMotorDutyCycle(0);
	setLeftMotorDutyCycle(0);
	
	
	// Set up IMU
	configureMPU6050();
	
	// Set up Debug led
	configLED();

	//configure encoders
	configMotorEncoder();
	
	// compute offest 
	computeOffsets(1024);
		
	// Turn on indicator led
	GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
	
	
	while(1)
	{
		// read encoder data
		readEncoders();
			
		if(dataReady)
		{
			// clear data ready flag
			dataReady = 0;
			
			// compute duration
			dt = msTicks - prevMsTicks;
			prevMsTicks = msTicks;
			
			// feedback loop
			angle = complementaryFilter((float)dt/1000);		
			
			if (angle < 20 && angle > -20)
			{
				// compute pwm 
				pwm  = computePID(angle, (velocityLeft + velocityRight)/2,(displacementLeft+displacementRight)/2 , dt/1000.);
				
				// update motor pwm values
				stabilizeMotorOutput(pwm, dt);
			} 
			else 
			{
				// no chance of recovery stop motors.
				setRightMotorDutyCycle(0);
				setLeftMotorDutyCycle(0);
			}
		}
	}
	
}



void delay(uint32_t dt)
{
	uint32_t prevMsTicks = msTicks;
	
	while(msTicks - prevMsTicks < dt)
	{
		__NOP();
	}
}

void SysTick_Handler()
{
	msTicks++;
}

