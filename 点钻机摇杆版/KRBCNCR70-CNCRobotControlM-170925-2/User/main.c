/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : main.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the main file.
***************************************************************************************/


#include "stm32f10x_lib.h"
#include "CANopen.h"
/*************Header file***************/
#include "platform_init.h"
#include "PulseInit.h"
#include "Delay.h"
#include "w25qxx.h" 
#include "Usart.h"
#include "SpeedControl.h"
#include "Parameter.h"
#include "StatusControl.h"
#include "SignalWatch.h"
#include "Error.h" 
#include "BackToOrigin.h"
#include "Homing.h"
/*************Header file***************/

int main(void)
{
	RCC_Configuration();
	NVIC_Configuration();         	
	GPIO_Configuration();
	USART1_Configuration();	   
	CAN_Initialize();               //--2017.7.31 
	SysTick_Init();
	TimeInit();
	#ifdef HARDLIMITJUDGE_EXTI
	Limit_EXTI_Init();
	#endif
	PulseInit();
	W25QXX_Init();
	Delay(100);
	ReadIICSysParameter();
	Delay(100);
	Robot_Reset();	             //��λ��ز���	

	Controlword = 0x000f;
	Delay(1000);
	CANopen_Init();	               //CANopen������ʼ��		
	Initialize_Finished = TRUE;
	/***********************************/
	/**         ��������״̬           **/
	/***********************************/

	while(1)
	{
		BackToOrigin();	 //��ԭ�㴦��
		ErrorOperate();	 //�������
		WatchFunction(); //���ֿ�������ͨ�ż���ز�����״̬�ļ���
		StatusControl(); //�����źż���״̬����
		ActionControl(); //�������ƣ��ֶ�/�Զ� ģʽ�µĶ�������
	}
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
