#include "control.h"
#include "oled.h"
#include "pid.h"
#include "pidtool.h"
#include "key.h"
#include "usart.h"
#include "servo.h"
#include "delay.h"

PID_TypeDef x_pid,y_pid;
uint8_t RecCoorFlag = 0;
void (*mode_task)(void);
extern uint8_t x, y;
extern PID_TypeDef x_pid,y_pid;
extern float after_kalman_x, after_kalman_y;
int PidOut = 0;


void mode_1(void)
{
	//PidOut = PID_Calculate(&x_pid, after_kalman_x - 120);
	//servo_ctr(&Xserv, PidOut);
	PidOut = PID_Calculate(&y_pid, after_kalman_y - 120);
	servo_ctr(&Yserv, PidOut);
}

void mode_select(void)
{
	uint8_t key_num,mode_num = 0;
	OLED_ShowString(0, 0, "Mode:", 8, 1);
	OLED_ShowNum(40, 0, mode_num, 1, 8, 1);
	OLED_Refresh();
	while(1){
		key_num = KEY_Scan();
		if(key_num){
			switch (key_num){
				case KEY0_PRES:  mode_num++;break;
				case KEY1_PRES:	if(mode_num == 1){
									//mode_task = mode_1;break;
								}
				default: break;
			}
			if(mode_num > 4){
				mode_num = 0;
			}
			if(key_num == KEY1_PRES){
				break;
			}
			OLED_ShowNum(40, 0, mode_num, 1, 8, 1);
			OLED_Refresh();
		}
	}
	
}


void TIM1_UP_TIM10_IRQHandler(void)//5ms一次pid运算
{
	if(TIM_GetITStatus(TIM10,TIM_IT_Update)==SET) //溢出中断
	{
		
		if(RecCoorFlag){
			RecCoorFlag = 0;
			mode_task();
		}
	}
	TIM_ClearITPendingBit(TIM10,TIM_IT_Update); //清除中断标志位	
}

