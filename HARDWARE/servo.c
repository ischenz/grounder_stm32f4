#include "servo.h"
#include "delay.h"

ServTyp Xserv = {0,1700,1300};
ServTyp Yserv = {0,1700,1300};

extern int PidOut;

void servo_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Configure PA5 as alternate function (TIM2_CH1) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect TIM2 pins to AF1 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* Enable TIM2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* Configure TIM2 */
	TIM_TimeBaseStructure.TIM_Period = 4000 - 1;  // PWM period
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;       // 84MHz clock, 1kHz PWM frequency
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Configure TIM2 channel 1 in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1500;  
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	
	/* Configure TIM2 channel 2 in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1500;  
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	/* Start TIM2 */
	TIM_Cmd(TIM2, ENABLE);
}

void servo_ctr(ServTyp *servo, int PidInput)
{
	uint16_t CompSet = PidInput + servo->MINPWM;
	if( CompSet > servo->MAXPWM ){
		if(servo == &Xserv){
			TIM_SetCompare1(TIM2, servo->MAXPWM);
		}
		else{
			TIM_SetCompare2(TIM2, servo->MAXPWM);
		}
	}
	else {
		if(servo == &Xserv){
			TIM_SetCompare1(TIM2, CompSet);
		}
		else{
			TIM_SetCompare2(TIM2, CompSet);
		}
		
	}
	
//	if( CompSet > servo->pwm ){
//		servo->pwm++;
//	}
//	else if( CompSet < servo->pwm ){
//		servo->pwm--;
//	}
//	
//	if( servo->pwm > servo->MAXPWM ){
//		servo->pwm = servo->MAXPWM;
//	}
//	else if( servo->pwm < servo->MINPWM  ){
//		servo->pwm = servo->MINPWM;
//	}
//	
//	if( servo == &Xserv ){
//		TIM_SetCompare1(TIM2, servo->pwm);
//	}
//	else if( servo == &Yserv ){
//		TIM_SetCompare2(TIM2, servo->pwm);
//	}
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //溢出中断
	{
		servo_ctr(&Yserv, PidOut);
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //清除中断标志位
}
