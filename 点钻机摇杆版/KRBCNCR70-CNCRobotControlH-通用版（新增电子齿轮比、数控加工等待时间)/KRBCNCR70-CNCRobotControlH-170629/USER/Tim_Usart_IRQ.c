/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : TIM6_USART1_IRQ.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "DGUS.h"
#include "Usart.h"
#include "Usart1_Robot.h"
#include "Key.h"
#include "freeprogramme.h"
#include "delay.h"
u8  Test_USART1ReceiveDataBuffer[60]={0};         //�������ݻ���
u8  Test_SUART1Count=0;
u8  LCD_Control_Flag=0;
u8 LC_Empty_Flag=0;//�ϲֿձ�־λ
u8 Step_Period_Count=0;//�綯����
extern u8 Program_Transport_Flag;
extern u8 Program_Transport_Count;
extern u16 Delay_Time;
extern u8 Link_Step_Flag;
/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*-----------Usart1_Robot-----------------*/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 				
		USART1ReceiveDataBuffer[USART1ReceiveDataCounter] = USART_ReceiveData(USART1);
//		Test_USART1ReceiveDataBuffer[Test_SUART1Count] =  USART_ReceiveData(USART1);
//		Test_SUART1Count++;
//		if(Test_SUART1Count >=60)
//		{
//			Test_SUART1Count=0;
//		}
		if(USART1StartReceiveDataFlag==FALSE)
		{
			if(USART1ReceiveDataBuffer[USART1ReceiveDataCounter] == USART1_SEND_START) //F5 FA
			{			
				USART1ReceiveDataBuffer[0]=USART1_SEND_START;
				USART1StartReceiveDataFlag=TRUE;
				USART1ReceiveDataCounter = 1;
			}
		}
		else
		{
			USART1ReceiveDataCounter++;
			if((USART1ReceiveDataCounter>=2) && (USART1ReceiveDataBuffer[0]==USART1_SEND_START) && (USART1ReceiveDataCounter-1==USART1ReceiveDataBuffer[1]))//---2017.10.22(USART1ReceiveDataCounter-1)=���ݳ���
			{
				if(USART1ReceiveDataBuffer[USART1ReceiveDataCounter-1]!=USART1_SEND_END)  //���յ������һ�����ݲ��ǽ����źţ�������������ʧ��
				{	
//				 	 USART1ErrorFlag=TRUE;	               //�����������ݳ���								 
			         USART1ReceiveDataCounter = 0;
					 USART1StartReceiveDataFlag=FALSE;	
				}
				else  
				{
					 USART1RecieveFinishedFlag = TRUE;
//					 USART1ErrorFlag=FALSE;                //������������֡����
					 USART1ReceiveDataLen=USART1ReceiveDataCounter-1;    //��¼�������ݳ���
					 USART1ReceiveDataCounter = 0;
					 USART1StartReceiveDataFlag=FALSE;	 
					
				}
			}
		}
		if(USART1ReceiveDataCounter>=60)
		{
//			     USART1ErrorFlag=TRUE;
				 USART1StartReceiveDataFlag=FALSE;			//�������
				 USART1StartReceiveDataFlag=FALSE;								 
		     USART1ReceiveDataCounter = 0;	
		}
	}
}


/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*-----------Usart1_Robot-----------------*/
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{ 				
		USART1ReceiveDataBuffer[USART1ReceiveDataCounter] = USART_ReceiveData(USART2);
		if(USART1StartReceiveDataFlag==FALSE)
		{
			if(USART1ReceiveDataBuffer[USART1ReceiveDataCounter] == USART1_SEND_START) //F5 FA
			{			
				USART1ReceiveDataBuffer[0]=USART1_SEND_START;
				USART1StartReceiveDataFlag=TRUE;
				USART1ReceiveDataCounter = 1;
			}
		}
		else
		{
			USART1ReceiveDataCounter++;
			if((USART1ReceiveDataCounter>=2) && (USART1ReceiveDataBuffer[0]==USART1_SEND_START) && (USART1ReceiveDataCounter-1==USART1ReceiveDataBuffer[1]))
			{
				if(USART1ReceiveDataBuffer[3]==0xCC&USART1ReceiveDataBuffer[4]==0xCC)//�ϲֿ�
				{
					Robot_Status=ACTION_PAUSE;
					LCD_Current_Page = 	116;
					LC_Empty_Flag=1;//�ϲֿ�
					Auto_Run_Flag = FALSE;					   

				}				
				if(USART1ReceiveDataBuffer[USART1ReceiveDataCounter-1]!=USART1_SEND_END)  //���յ������һ�����ݲ��ǽ����źţ�������������ʧ��
				{	
//				 	 USART1ErrorFlag=TRUE;	               //�����������ݳ���								 
			         USART1ReceiveDataCounter = 0;
					 USART1StartReceiveDataFlag=FALSE;	
				}
				else  
				{
					 USART1RecieveFinishedFlag = TRUE;
//					 USART1ErrorFlag=FALSE;                //������������֡����
					 USART1ReceiveDataLen=USART1ReceiveDataCounter-1;    //��¼�������ݳ���
					 USART1ReceiveDataCounter = 0;
					 USART1StartReceiveDataFlag=FALSE;	
					/*************���ư��Ϸ�LCD����ָ��******************/
					if(USART1ReceiveDataBuffer[2]==0x1F)
					{
						if(USART1ReceiveDataBuffer[3]==0x01)//����
						{
							LCD_Var_Address=0x1009;
							LCD_New_Order=TRUE;
						}
						if(USART1ReceiveDataBuffer[3]==0x02)//��ͣ
						{
							LCD_Var_Address=0x100A;
							LCD_New_Order=TRUE;							
						}						
						if(USART1ReceiveDataBuffer[3]==0x03&&(g_Auto_Status == AUTO_RUNNING||g_Auto_Status == AUTO_PAUSE))//���ڲ���ֹͣ��ť
						{
							LCD_Var_Address=0x100B;
							LCD_New_Order=TRUE;	
							LCD_Control_Flag=1;//��ֹͣ
						}
						if(USART1ReceiveDataBuffer[3]==0x04&&(g_Auto_Status == AUTO_WAITE))//���ھ���ֹͣ��ť��λ
						{
							LCD_Var_Address=0x10A0;
							LCD_New_Order=TRUE;							
						}
					}					
				}
			}
		}
		if(USART1ReceiveDataCounter>=60)
		{
//		     USART1ErrorFlag=TRUE;
			 USART1StartReceiveDataFlag=FALSE;			//�������
			 USART1StartReceiveDataFlag=FALSE;								 
	         USART1ReceiveDataCounter = 0;	
		}
	}
}


/*-----------Usart3_LCD -----------------*/
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{ 						
		if( (LCD_New_Order == FALSE) && (LCDErrorFlag == FALSE) )
		{
			LCDReceiveDataBuffer[LCDReceiveDataCounter] = USART_ReceiveData(USART3);
			if(LCDReceiveDataBuffer[LCDReceiveDataCounter] == LCD_PROTOCOL1) //�յ�ͷָ��A5��LCDReceiveDataCounter����
			{
				LCDReceiveDataCounter = 0;
				LCDReceiveDataBuffer[0] = LCD_PROTOCOL1;
			}
			LCDReceiveDataCounter++;
			if((LCDReceiveDataCounter>=2) && (LCDReceiveDataBuffer[0]==LCD_PROTOCOL1) && (LCDReceiveDataBuffer[1]==LCD_PROTOCOL2))
			{
				/*-----ASCII���������ϴ�----�����������ȷ�����洢��4040������2600��������2802�ֿ� �ļ���ѯ2830-*/
				if(((LCDReceiveDataBuffer[4]==0x40)&&(LCDReceiveDataBuffer[5]==0x40)) || ((LCDReceiveDataBuffer[4]==0x46)&&(LCDReceiveDataBuffer[5]==0x00))
					||((LCDReceiveDataBuffer[4]==0x28)&&(LCDReceiveDataBuffer[5]==0x02))||((LCDReceiveDataBuffer[4]==0x28)&&(LCDReceiveDataBuffer[5]==0x30)))
				{ 	 
				     if(LCDReceiveDataBuffer[8] ==0xFF)		  //1����ĸ
					 {
						 LCD_Var_Address  = (LCDReceiveDataBuffer[4]<<8)&0xFF00;          //��ַ-��λ��ǰ
					 	 LCD_Var_Address +=  LCDReceiveDataBuffer[5]&0x00FF;              //��ַ-��λ�ں�		     
						 LCD_Var_Data     = (LCDReceiveDataBuffer[7]<<24)&0xFF000000;      //����-��λ��ǰ
					 	 LCD_Var_Data    += (0x20                   <<16)&0x00FF0000;      //����-	
						 LCD_Var_Data    += (0x20                   <<8 )&0x0000FF00;      //����-
						 LCD_Var_Data    += (0x20                       )&0x000000FF;      //����-��λ�ں�
						 LCD_New_Order = TRUE;
						 LCDErrorFlag = FALSE;
						 LCDReceiveDataCounter = 0;
					 }			
				     else if(LCDReceiveDataBuffer[9]==0xFF)		  //2����ĸ
					 {
						 LCD_Var_Address  = (LCDReceiveDataBuffer[4]<<8)&0xFF00;          //��ַ-��λ��ǰ
					 	 LCD_Var_Address +=  LCDReceiveDataBuffer[5]&0x00FF;              //��ַ-��λ�ں�
					   LCD_Var_Data     = (LCDReceiveDataBuffer[7]<<24)&0xFF000000;      //����-��λ��ǰ
					 	 LCD_Var_Data    += (LCDReceiveDataBuffer[8]<<16)&0x00FF0000;      //����-	
						 LCD_Var_Data    += (0x20                   <<8 )&0x0000FF00;      //����-
						 LCD_Var_Data    += (0x20                       )&0x000000FF;      //����-��λ�ں�
						 LCD_New_Order = TRUE;
						 LCDErrorFlag = FALSE;
						 LCDReceiveDataCounter = 0;
					 }
				     else if(LCDReceiveDataBuffer[10]==0xFF)		  //3����ĸ
					 {
						 LCD_Var_Address  = (LCDReceiveDataBuffer[4]<<8)&0xFF00;          //��ַ-��λ��ǰ
					 	 LCD_Var_Address +=  LCDReceiveDataBuffer[5]&0x00FF;              //��ַ-��λ�ں�
					   LCD_Var_Data     = (LCDReceiveDataBuffer[7]<<24)&0xFF000000;                         //����-��λ��ǰ
					 	 LCD_Var_Data    += (LCDReceiveDataBuffer[8]<<16)&0x00FF0000;      //����-	
						 LCD_Var_Data    += (LCDReceiveDataBuffer[9]<<8 )&0x0000FF00;      //����-
						 LCD_Var_Data    += (0x20                       )&0x000000FF;      //����-��λ�ں�
						 LCD_New_Order = TRUE;
						 LCDErrorFlag = FALSE;
						 LCDReceiveDataCounter = 0;
					 }	
				     else if(LCDReceiveDataBuffer[11]==0xFF)		  //4����ĸ
					 {
						 LCD_Var_Address  = (LCDReceiveDataBuffer[4]<<8)&0xFF00;          //��ַ-��λ��ǰ
					 	 LCD_Var_Address +=  LCDReceiveDataBuffer[5]&0x00FF;              //��ַ-��λ�ں�
					   LCD_Var_Data     = (LCDReceiveDataBuffer[7]<<24)&0xFF000000;      //����-��λ��ǰ
					 	 LCD_Var_Data    += (LCDReceiveDataBuffer[8]<<16)&0x00FF0000;      //����-	
						 LCD_Var_Data    += (LCDReceiveDataBuffer[9]<<8) &0x0000FF00;      //����-
						 LCD_Var_Data    +=  LCDReceiveDataBuffer[10]    &0x000000FF;      //����-��λ�ں�
						 LCD_New_Order = TRUE;
						 LCDErrorFlag = FALSE;
						 LCDReceiveDataCounter = 0;	
					 }	
				}
				/*-----���������ϴ�----+��-�ż�ֵ��ͬ-*/
				else if(LCDReceiveDataBuffer[2] == 0x06)	  //�������ݳ���Ϊ8����Ч���ݳ���Ϊ6
				{
					if(LCDReceiveDataCounter>=9)
					{
						if(LCDReceiveDataBuffer[3] == 0x83)
						{
							 LCD_Var_Address  = (LCDReceiveDataBuffer[4]<<8)&0xFF00; //��ַ-��λ��ǰ
						 	 LCD_Var_Address += LCDReceiveDataBuffer[5]&0x00FF;      //��ַ-��λ�ں�
							 LCD_Key_Value  = (LCDReceiveDataBuffer[7]<<8)&0xFF00;   //����-��λ��ǰ
						 	 LCD_Key_Value += LCDReceiveDataBuffer[8]&0x00FF;        //����-��λ�ں�
							 LCD_Var_Data = LCD_Key_Value;
							 if(LCD_Var_Data==0)
								 LCD_Var_Data=0;
						 	 LCD_New_Order = TRUE;
							 LCDErrorFlag = FALSE;
							 LCDReceiveDataCounter = 0;
						}
						else
						{
						 	 LCDErrorFlag = TRUE;
							 LCDReceiveDataCounter = 0;
							 LCD_New_Order = FALSE;
						}
					}	 					
				}
				/*-----�ٶ�ֵ�ش�-----*/
				else if(LCDReceiveDataBuffer[2] == 0x08)	  //�������ݳ���Ϊ11����Ч���ݳ���Ϊ8
				{
					if(LCDReceiveDataCounter>=11)
					{
						if(LCDReceiveDataBuffer[3] == 0x83)
						{
							 LCD_Var_Address  = (LCDReceiveDataBuffer[4]<<8)&0xFF00;  //��ַ-��λ��ǰ
							 LCD_Var_Address +=  LCDReceiveDataBuffer[5]&0x00FF;      //��ַ-��λ�ں�
							 LCD_Var_Data    = (LCDReceiveDataBuffer[7]<<24)&0xFF000000;      //����-��λ��ǰ
							 LCD_Var_Data   += (LCDReceiveDataBuffer[8]<<16)&0x00FF0000;      //����-	
							 LCD_Var_Data   += (LCDReceiveDataBuffer[9]<<8) &0x0000FF00;      //����-
							 LCD_Var_Data   +=  LCDReceiveDataBuffer[10]    &0x000000FF;      //����-��λ�ں�			 
							 Delay_Time    = LCD_Var_Data*10;
							 LCD_New_Order = TRUE;
							 LCDErrorFlag  = FALSE;
							 LCDReceiveDataCounter = 0;
						}
						else
						{
						 	 LCDErrorFlag = TRUE;
							 LCDReceiveDataCounter = 0;
							 LCD_New_Order = FALSE;
						}
					}	 					
				}
				/*-----ʵʱʱ�ӷ���-----*/
				else if(LCDReceiveDataBuffer[2] == 0x0A)	  //�������ݳ���Ϊ13����Ч���ݳ���Ϊ10
				{
					if(LCDReceiveDataCounter>=13)
					{
						if(LCDReceiveDataBuffer[3] == 0x81)
						{
						   LCD_RTC_Reg_len  = (LCDReceiveDataBuffer[4]<<8)&0xFF00;         //��ַ-��λ��ǰ
						 	 LCD_RTC_Reg_len +=  LCDReceiveDataBuffer[5]&0x00FF;             //��ַ-��λ�ں� 2007
						   LCD_RTC_Day    = (LCDReceiveDataBuffer[6]<<24)&0xFF000000;      //����-��λ��ǰ
						 	 LCD_RTC_Day   += (LCDReceiveDataBuffer[7]<<16)&0x00FF0000;      //����-	
							 LCD_RTC_Day   += (LCDReceiveDataBuffer[8]<<8) &0x0000FF00;      //����-
							 LCD_RTC_Day   +=  LCDReceiveDataBuffer[9]     &0x000000FF;      //����-��λ�ں�			 
						   LCD_RTC_Time   = (LCDReceiveDataBuffer[10]<<24)&0xFF000000;     //ʱ��-��λ��ǰ
						 	 LCD_RTC_Time  += (LCDReceiveDataBuffer[11]<<16)&0x00FF0000;     //ʱ��-	
							 LCD_RTC_Time  += (LCDReceiveDataBuffer[12]<<8) &0x0000FF00;     //ʱ��-
							 LCD_New_Order = TRUE;
							 LCDErrorFlag = FALSE;
							 LCDReceiveDataCounter = 0;
							 Program_Save_Time();
						}
						else
						{
						 	 LCDErrorFlag = TRUE;
							 LCDReceiveDataCounter = 0;
							 LCD_New_Order = FALSE;
						}
					}	 					
				}
				else if(LCDReceiveDataBuffer[4]==0x41&&LCDReceiveDataBuffer[5]==0x00)
				{
					LCDErrorFlag=FALSE;
				}
			}
			if(LCDReceiveDataCounter>=20)
			{
				 	 LCDErrorFlag = TRUE;
					 LCDReceiveDataCounter = 0;
					 LCD_New_Order = FALSE;
			}	
		}//if( (LCD_New_Order == FALSE) && (LCDErrorFlag == FALSE) )
	    else
	    {
			LCDReceiveDataBuffer[LCDReceiveDataCounter] = USART_ReceiveData(USART3);
//			LCDReceiveDataCounter++;
			LCDReceiveDataBuffer[LCDReceiveDataCounter] = 0;
			LCDReceiveDataCounter = 0;			
		}
		if(LCDErrorFlag)
		{
			LCDReceiveDataCounter = 0;
			LCDErrorFlag = FALSE;
		}				
	}	
}


void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
  	{
    	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}



/**************************************************************************
//     ��ʱ��4���ã� 720��Ƶ�����ϼ���������ֵΪ1000->10ms	  
***************************************************************************/
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) 
  	{
    	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

		if(LCD_RealTime_Watch==FALSE)
		{
			LCD_RealTime_Count++;
			if(LCD_RealTime_Count>=10)	   //100ms��ѯһ��
			{
				LCD_RealTime_Watch=TRUE;	
			}
		}
		if((LCD_Current_Page==109)||(LCD_Current_Page==2))  //�����λ����ѯ	20ms
		{
			Manual_Pulse_Count++;
			if(Manual_Pulse_Count>=2)
			{
				Manual_Pulse_Watch=TRUE;	
			}
		}
		//������ѯ
		if(g_Key_Scan==FALSE)
		{
			g_Key_Scan_Count++;
			if(g_Key_Scan_Count>=10)    //100ms
			{
				g_Key_Scan=TRUE;
				g_Key_Scan_Count=0;
			}
		}
		if(Key_Scan_Delay_Count)
		{
			Key_Scan_Delay_Count++;
		}
		//������ȡIIC��ʱ
		if(Read_IIC_Flag==TRUE)
		{
			Read_IIC_Count++;
			if(Read_IIC_Count>5) //50ms
			{
				Read_IIC_Done = TRUE;
				Read_IIC_Count = 0;
				Read_IIC_Flag = FALSE;
			}
		}
		if(USART1RecieveCount)
		{
			USART1RecieveCount++;
		}
		/*-----�������-----*/
		if(g_BackLight_Dark == FALSE)
		{
			g_BackLight_Count++;
			if(g_BackLight_Count>=(g_BackLight_Time*100))
			{
				BackLight(0x04);
				g_BackLight_Dark = TRUE;
				g_BackLight_Count = 0;
			}
		}

//		if(g_BackLight_Count>(g_BackLight_Time*100+100))
//		{
//			g_BackLight_Count = (g_BackLight_Time*100+1);
//		}

	   /*-----���ɱ��-�༭����˸----*/
	   if(g_Background_Blink_Delay)
	   {
	   		g_Background_Blink_Count++;
			if(g_Background_Blink_Count>=100)  //1000ms
			{
				g_Background_Color_Blink=~g_Background_Color_Blink;
				g_Background_Blink_Count=0;
			}	   
	   }
	   //����ʱ������
	   if(g_Alarm_Buzzer)
	   {
	       g_Alarm_Buzzer_Count++;
		   if(g_Alarm_Buzzer_Count>=100)
		   {
		   		g_Alarm_Buzzer_Timer = TRUE;
				g_Alarm_Buzzer_Count=0;
		   }
	   }
		if(Program_Transport_Flag)
		{
			Program_Transport_Count++;
		}
		
		
		if(Link_Step_Flag==1)//�綯���ڼ���50ms
		{
			Step_Period_Count++;
		}
	}

}






