/*
 ******************************************************
 STM32F407ZG KEIL例程 大越创新开发板

 202207081915	创建例程

 作者：			xc						V1.0
 ******************************************************
 */
#include "sys.h"
#include "led.h"
#include "key.h"
#include "delay.h"
#include "oled.h"
#include "timer.h"
#include "control.h"
#include "pid.h"
#include "pidtool.h"
#include "usart.h"
#include "servo.h"
#include "Kalman.h"

uint8_t x, y;
float after_kalman_x, after_kalman_y;
extern void (*mode_task)(void);
extern PID_TypeDef x_pid,y_pid;
extern uint8_t RecCoorFlag;

int main(void)
{
	//500—2500  取 1100-1900
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	delay_init(168);
	protocol_init();
	uart_init(115200);
	uart2_v831_init(115200);
	LED_Init();
	OLED_Init();
	KEY_Init();
	OLED_ShowString(0,0,"Hello !!!",16,1);
	OLED_Refresh();
	
	servo_init();
	servo_ctr(&Xserv, 400);
	servo_ctr(&Yserv, 400);
	delay_ms(1000);
	OLED_Clear();
	
	//pid初始化
	PID_param_init(&x_pid);
	PID_param_init(&y_pid);
	set_pid_polarity(&x_pid, 1, 1, 1);
	set_pid_polarity(&y_pid, -1, -1, -1);
	set_p_i_d(&x_pid, 20, 4, 0);
	set_p_i_d(&y_pid, 4, 0, 0);
	set_pid_target(&x_pid, 0);
	set_pid_target(&y_pid, 0);
//	pid_tool_send_param(&Pitch_PID ,CURVES_CH2);
	PID_TimerInit();
	
	mode_task = mode_1;
	
	TIM_Cmd(TIM10, ENABLE);//开始PID运算
	
	while(1)
	{
		//i++;
//		if(i == 1900){
//			while(1){
//				i--;
//				delay_ms(2);
//				TIM_SetCompare1(TIM2, i);
//				TIM_SetCompare2(TIM2, i);
//				if(i == 1100) break;
//			}
//		}
//		TIM_SetCompare1(TIM2, i);
//		TIM_SetCompare2(TIM2, i);
		
		receiving_process();
		if(decode_uartData(&x, &y)){
			after_kalman_x = kalmanFilter_A(x);
			after_kalman_y = kalmanFilter_A(y);
			RecCoorFlag = 1;
			printf("kalman:%f,%d \n",after_kalman_y,y);
		}
		
//		temp = kalmanFilter_Roll;
//		set_computer_value(SEND_FACT_CMD, CURVES_CH1, &temp, 1);
//		temp = kalmanFilter_Pitch;
//		set_computer_value(SEND_FACT_CMD, CURVES_CH2, &temp, 1);
	}
}
