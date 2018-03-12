#include "MotorEncoders.h"
#include "retarget.h"


// absolute encoder data.
int16_t leftCount;
int16_t rightCount;

int32_t leftTotal;
int32_t rightTotal;


// velocity and displacement in [m] and [m/s]
float velocityLeft;
float velocityRight;

float displacementLeft;
float displacementRight;


static int16_t oldLeftEncoder;
static int16_t oldRightEncoder;
static int16_t leftEncoder;
static int16_t rightEncoder;


void configMotorEncoder()
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;

	// Enable clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	// Set PA6 and PA7 as input pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// set PA9 and PA8 as input pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Enable Timers
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Configure timebase 
	TIM_TimeBaseStruct.TIM_Period = 0xFFFF;
	TIM_TimeBaseStruct.TIM_Prescaler = 0x0;
	TIM_TimeBaseStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
	
	// Set up timebase 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);
	
	// Configure encoder mode 
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	// Set count register to zero
	TIM_SetCounter (TIM3, 0);
	TIM_SetCounter (TIM1, 0);
	
	// Enable timers
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
}

void readEncoders()
{
	static uint32_t prevMsTicks;
	
	
	if (msTicks - prevMsTicks> (1000)/SAMPLE_FREQ)
	{
		prevMsTicks = msTicks;
		
		// store previous values
		oldLeftEncoder = leftEncoder;
		oldRightEncoder = rightEncoder;
		
		// read new value 
		leftEncoder = TIM_GetCounter(TIM1);
		rightEncoder = -TIM_GetCounter(TIM3);
		
		// compute difference 
		leftCount = leftEncoder - oldLeftEncoder;
		rightCount = rightEncoder - oldRightEncoder;
		
		// update totals
		leftTotal += leftCount;
		rightTotal += rightCount;
		
		// compute velocity in readable format
		velocityLeft = leftCount * SCALING_FACTOR * SAMPLE_FREQ;
		velocityRight = rightCount *SCALING_FACTOR * SAMPLE_FREQ;
		
		// compute displacement in readable format
		displacementLeft = leftTotal * SCALING_FACTOR;
		displacementRight = rightTotal * SCALING_FACTOR;
	}
}

