/*************** (C) COPYRIGHT 2012 Kingrobot manipulator Team ****************
* File Name          : Delay.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 20/06/2012
* Description        : This file delimits all the delay functions.
******************************************************************************/
#include "stm32f10x_lib.h"
#include "Delay.h"


TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;
static vu32 TimingDelay;     //�δ�ʱ������


void SysTick_Init()
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); 
    SysTick_SetReload(9000);  // SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) 
    SysTick_ITConfig(ENABLE); // Enable SysTick interrupt 
}

void Delay(u32 nTime)
{
    /* Enable the SysTick Counter */
    SysTick_CounterCmd(SysTick_Counter_Enable);     
    TimingDelay = nTime;    
    while(TimingDelay != 0);    
    /* Disable SysTick Counter */
    SysTick_CounterCmd(SysTick_Counter_Disable);
    /* Clear SysTick Counter */
    SysTick_CounterCmd(SysTick_Counter_Clear);

}


/*******************************************************************************
* Function Name  : TimingDelay_Decrement
* Description    : Decrements the TimingDelay variable.
* Input          : None
* Output         : TimingDelay
* Return         : None
*******************************************************************************/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

void SysTickHandler(void)
{
  TimingDelay_Decrement();

}

void DelayNus(vu32 dly)			 //��ʱdly us��dly>=6����С��ʱ��λ6us
{
	vu32 n;
	while(dly--)			   	//�ⲿ����8M��PPL_9��Sysclk=8MHz*9=72MHz
	{
		n=8;					//΢����������֤��ʱ����
		while(n--);
	}
}

void DelayNms(vu32 dly)		    //��ʱdly ms
{
	while(dly--)				//�ⲿ����8M��PPL_9��Sysclk=8MHz*9=72MHz
		DelayNus(1000);			//1ms��ʱ����
}

/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/
















