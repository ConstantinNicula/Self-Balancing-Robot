#include "MotorControl.h"


#define LM_EN_PIN GPIO_Pin_1
#define LM_EN_PORT GPIOC

#define LM_CW_PIN GPIO_Pin_0
#define LM_CW_PORT GPIOB

#define LM_CCW_PIN GPIO_Pin_4
#define LM_CCW_PORT GPIOA


#define RM_EN_PIN GPIO_Pin_3
#define RM_EN_PORT GPIOB

#define RM_CCW_PIN GPIO_Pin_4
#define RM_CCW_PORT GPIOB

#define RM_CW_PIN GPIO_Pin_5
#define RM_CW_PORT GPIOB

void configMotorControlPins() 
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// Enable GPIOA and GPIOB clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC| RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOA, ENABLE);
	
	/*Configure PWM pins */
	
	//PB10 (TIM2-CH3) - PWM Pin right motor 
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	//PA1 (TIM2-CH2) - PWM Pin left motor
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	/*Configure enable and direction pins for left motor*/

	GPIO_InitStruct.GPIO_Pin = LM_CW_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LM_CW_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = LM_CCW_PIN;
	GPIO_Init(LM_CCW_PORT, &GPIO_InitStruct);
	
	
	
	/*Configure enable and direction pins for left motor*/
	GPIO_InitStruct.GPIO_Pin = RM_CW_PIN;
	GPIO_Init(RM_CW_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = RM_CCW_PIN;
	GPIO_Init(RM_CCW_PORT, &GPIO_InitStruct);
	
	
	// set enable pins to high 
	GPIO_WriteBit(LM_EN_PORT, LM_EN_PIN, Bit_SET);
	
	// set direction pins to low
	GPIO_WriteBit(RM_CW_PORT, RM_CW_PIN, Bit_SET);
	GPIO_WriteBit(RM_CCW_PORT, RM_CCW_PIN, Bit_RESET);
	GPIO_WriteBit(LM_CW_PORT, LM_CW_PIN, Bit_SET);
	GPIO_WriteBit(LM_CCW_PORT, LM_CCW_PIN, Bit_RESET);
	
}

void configMotorTimerChannels() 
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;
	TIM_OCInitTypeDef  TIM_OCInitStruct;

	// Enable TIM3 and TIM2 clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// Configure timebase for TIM3 and TIM2
	TIM_TimeBaseStruct.TIM_Period = PWMperiod;
	TIM_TimeBaseStruct.TIM_Prescaler = PWMprescalerValue;
	TIM_TimeBaseStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	
	// Set up timebase 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
		
	// Configure TIM2-CH2 as OC PWM1
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 0; // set inital PWM to 0
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	
	
	TIM_OC3Init(TIM2, &TIM_OCInitStruct);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM2, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	
	// Enable Timers
	TIM_Cmd(TIM2, ENABLE);
}

void setRightMotorDutyCycle(int16_t pulse) 
{
	// clamp input to avoid motor damage
	if(pulse > maxPulse)
		pulse = maxPulse;
	else if (pulse < -maxPulse)
		pulse = -maxPulse;
	
	// set direction bit 
	if(pulse > 0)
	{
		GPIO_WriteBit(RM_CW_PORT, RM_CW_PIN, Bit_SET);
		GPIO_WriteBit(RM_CCW_PORT, RM_CCW_PIN, Bit_RESET);
		// set CCR3 value 
		TIM2->CCR3 =pulse;
	}
	else 
	{	
		GPIO_WriteBit(RM_CW_PORT, RM_CW_PIN, Bit_RESET);
		GPIO_WriteBit(RM_CCW_PORT, RM_CCW_PIN, Bit_SET);
		
		// set CCR3 value 
		TIM2->CCR3 = -pulse;
	}
}

void setLeftMotorDutyCycle(int16_t pulse) 
{
	// clamp input to avoid motor damage
	if(pulse > maxPulse)
		pulse = maxPulse;
	else if (pulse < -maxPulse)
		pulse = -maxPulse;
	
	// set direction bit 
	if(pulse > 0)
	{
		GPIO_WriteBit(LM_CW_PORT, LM_CW_PIN, Bit_SET);
		GPIO_WriteBit(LM_CCW_PORT, LM_CCW_PIN, Bit_RESET);
		
		// set CCR3 value 
		TIM2->CCR2 = pulse;
	}
	else 
	{
		GPIO_WriteBit(LM_CW_PORT, LM_CW_PIN, Bit_RESET);
		GPIO_WriteBit(LM_CCW_PORT, LM_CCW_PIN, Bit_SET);
		
		// set CCR3 value 
		TIM2->CCR2 = -pulse;
	}
}



void lockLeftMotor(void)
{
	// set PWM to 0
	TIM2->CCR2 = 0;
	GPIO_WriteBit(LM_CW_PORT, LM_CW_PIN, Bit_SET);
	GPIO_WriteBit(LM_CCW_PORT, LM_CCW_PIN, Bit_SET);

}
void lockRightMotor(void)
{
	// set PWM to 0
	TIM2->CCR3 = 0;
	
	GPIO_WriteBit(RM_CW_PORT, RM_CW_PIN, Bit_SET);
	GPIO_WriteBit(RM_CCW_PORT, RM_CCW_PIN, Bit_SET);
}
