#include "key.h"
#include "usart.h"
#include "led.h"
#include "delay.h"  

//������ʼ������
void KEY_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOA,GPIOEʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_3; //KEY0 ��Ӧ����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;//����
	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE
	
	PWR_WakeUpPinCmd(DISABLE); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//WK_UP��Ӧ����PA0
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA0
} 
//������������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//0��û���κΰ�������
//1��KEY0����
//2��KEY1����
//3��KEY2���� 
//4��WKUP���� WK_UP
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2>WK_UP!!
//u8 KEY_Scan(u8 mode)
//{
//	static u8 key_up=1;//�������ɿ���־
//	if(mode)key_up=1;  //֧������		  
//	if(key_up&&(KEY0==1||WK_UP==1))
//	{
//		delay_ms(10);//ȥ���� 
//		key_up=0;
//		if(KEY0==1)return KEY0_PRES;
//		else if(WK_UP==1)return WKUP_PRES;
//	}else if(KEY0==0&&WK_UP==0)key_up=1; 	    
// 	return 0;// �ް�������
//}

uint8_t KEY_Scan(void)
{	
	volatile uint8_t ret = 0;
	static uint8_t release=1;//�������ɿ���־	  
	if( release && (KEY0==0||KEY1==0))
	{
		delay_ms(10);//ȥ���� 
		release = 0;
		if(KEY0==0)			ret = KEY0_PRES;
		else if(KEY1 ==0)	ret = KEY1_PRES;
	}
	else if( (KEY0==1) && (KEY1==1) ){
		release = 1;
		ret = 0;
	}
 	return ret;// �ް�������
}


void KeyAction(void)
{
	uint8_t KEY_Val = 0;
	KEY_Val = KEY_Scan();
	if(KEY_Val)
	{
		if(KEY_Val == WKUP_PRES){
			//LED2 = 0;
		}
		if(KEY_Val == KEY1_PRES){
			LED2 = !LED2;
		}		
	}
}
















