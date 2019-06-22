/********************(C)COPYRIGHT	2012 Kingrobot manipulator Team**********************
** �ļ�����:	main.c
** ����:   		����
** �汾��		V1.0.0
** ����		2012/7/10
** ������		���ļ��ṩ������
*****************************************************************************************/	
/*-------USB---------*/
#include "usbh_core.h"
#include "usbh_usr.h"
#include "usbh_msc_core.h"


//#include "stm32f10x_lib.h"
#include "PlatformInit.h"					

#include "Delay.h"
#include "i2cint.h"
#include "24Cxx.h" 

#include "DGUS.h"
//#include "Usart.h"
#include "Usart1_Robot.h"
#include "Key.h"


u8 send_buff2[]={"�ޱ���"};
u8 Beyond_LastDa[]={":����ʹ������,����ϵ���ҽ���!  "};

int main(void)
{  
//	u8 data[500]={0};
	RCC_Configuration();	  		       //�򿪸�ģ�鿪ʱ��
	NVIC_Configuration();	  		       //���ȼ�����  
	GPIO_Configuration();
	Key_EXTI_Init();
	TimeInit();				  		           //ϵͳʱ������
	#ifdef USART1_RS232
		USART1_DMA_Configuration();		   //DMA-Usart��ʼ��
	#endif 
	#ifdef USART2_RS485
		USART2_485_Configuration();		   //Usart2��ʼ��
	#endif
	  USART3_LCD_Configuration();		   //Usart3��ʼ��   
	
	I2CInit();				  		           //I2C��ʼ��	

//	 AT24CXX_Write(0x0000, data, 12);
//	 AT24CXX_Write(0x1020,data,10);
//	 AT24CXX_Write(0x1015,data,1);//��ǰ���г�����

 	DelayNms(1000);				             //ϵͳ��ʱ 
	RTC_Init();	//��ʼ��ʵʱʱ��-������Ƿ�װ�����ܾ�����Ҫһ��������ʱ���һ�ο��ܿ����˻���
	ReadIICInitPar();

/*----------------ȷ���Ƿ���-------------------*/ 	
	InitLockTime();                    //���ü�������ʼʱ��
	UnLock_Forever_flag=1;
	if(UnLock_Forever_flag==1)       // ������Ч
	{
		Reset_Flag=1;
		Unlock_LastDay=9999;
	} 
    else				 
	{
		CheckTime();
	}
	//����ذε����߳����ǣ�������ܽ���~���ܳɹ��󣬼������¡�
	while(!Reset_Flag)	               //Reset_Flag=1ʱ�������� 
	{
		UnlockCodeInput();	   		
	}
/*****************ȷ���Ƿ���*******************/

	TIM_Cmd(TIM4,ENABLE);

/*----------------ȷ��������ʼ�����-------------------*/ 
	MainBoard_InitDone();//
/*****************ȷ��������ʼ�����*******************/
	
//	DelayNms(2000);
/*----------------����ͬ������-------------------*/ 
	while(!DataSyc());//ͬ������
/*****************����ͬ������*******************/
    //Program_Ready = TRUE;  //??
   
  TIM_Cmd(TIM4,DISABLE);

/*----------------USB��ʼ��-------------------*/ 
 	/* Init Host Library 1*/
  USBH_Init(&USB_OTG_FS_dev, &MSC_cb , &USR_Callbacks);//USB�ĳ�ʼ��������
/*****************USB��ʼ��*******************/	
	//������ѹ���ݷ���
	LinkMoveDataInit();
	//������ʼ��-ҳ���ʼ��
	MYDMA_Config();													//DMA��ʼ������·ģ�������ݴ��䣬����CPU���������͹��ģ�
	Adc_Init();															//ADC��ʼ����ģ�����ɼ���
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); // ����ADC1
	DMA_Cmd(DMA1_Channel1, ENABLE);  				//����DMAͨ��
	ReadIICAlarm();
	Robot_Init();
	while(1)																			  
	{
	   RodProcess();			//ң�˴���
	   KeyProcess();
	   LCDProcess();			//��������
	   LCDDisplay();			//LCD��ʾ����
	   LCDRealTimeWatch();//ʵʱ���Ӳ���
	   LCDError();
	   //�������U�̿�������
	   if(USBH_Process_Flag)
	   {
	   		USBH_Process();
	   }
	}
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/
