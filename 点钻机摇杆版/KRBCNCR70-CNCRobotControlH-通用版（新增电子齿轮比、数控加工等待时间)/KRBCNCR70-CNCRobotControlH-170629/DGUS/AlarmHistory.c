/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ****************
* File Name          : 
* Author             : 
* Version            : 
* Date               : 
* Description        : 
******************************************************************************/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "DGUS.h"


u8 Alarm_String_QC1[4]={"    "};
u8 Alarm_String_QC2[4]={"���"};
//u8 Alarm_String_QX[2][6]={{"      "},{"ȡ��  "}};

//���
static PageCut AlarmHistory_Clear={0x6810,34,550,50,610,90,630,50};	 

u8  Alarm_String[40][30]={	//Robot_Error_Data[0]
              {"                           "},   //0	  							
							{"�ޱ���                     "},   //1
						    //��е�ֱ���
							{"                           "},   //2
							{"                           "},   
							{"                           "},   //4
							{"�����ʧ��               "},   
							{"ʧ����Ч,�����׽������    "},   
							{"��е��ͨ�ų���,����      "},   //7	
							//Robot_Error_Data[1]
						   	//�ֳ�����������
							{"�ϲ���λ����,����        "},   //8
							{"��е�ֳ�����ȫ��           "},
							{"�󻬴���,����            "},   //10   ��е�ֲ�������Ԥ��
							{"��е�ֳ���Ӳ��λ           "},		  //��ѹ����������
							{"��е�ֳ�������λ           "},   //12
							{"��������,����            "},  
							{"�ŷ�������,����          "},   
							{"��е�ּ�ͣ,������          "},   //15	
							//Robot_Error_Data[2]												   
							{"����A����ⳬʱ,����     "},   //16
							{"����A�ؼ�ⳬʱ,����     "},
							{"�Զ����ж�����ⳬʱ,����"},   //18
							{"���ڰ�ȫ����߽�,�뷴�����"},  
							{"�Զ����г������,�봦��    "},   //20
							{"��ǰ�ӹ����������,�봦��  "},
							{"�ϲ�ȱ�ϱ���,�봦��        "},  //22 
							{"�ϲ����ϱ���,�봦��        "}   //23
							//Robot_Error_Data[3]

					      };


AlarmHistory Alarm_History_Time[20]={0};
AlarmHistoryDisplay Alarm_History_Display[21]={0};
static u8 gs_Alarm_History_Refresh = 0;
static u8 gs_Robot_Error_Flag = 1;
u8 g_Alarm_Buzzer = FALSE;
u8 g_Alarm_Buzzer_Timer = TRUE;
u8 g_Alarm_Buzzer_Count = 0;
u8  g_Current_Alarm = 0;
static u8 gs_Current_Alarm = 0xFF;
static u8 gs_Alarm_Blink = 0;
static u8 j=0;
static u8 k1=0;
static u8 k2=0;
u8 Write_IIC_Temp[21]={0};

extern u8 Program_Or_Point;
/*************************************************************************
**  ��������  LCDStartUp()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/5/28
**************************************************************************/
void LCDAlarmHistoryProcess(void)
{
	if( Robot_Status != ACTION_RUN)
	{
		 //������ǵ�ǰҳ�棬���л�����ǰ����
		if( (LCD_Var_Address == 0x6000) && (LCD_Main_Page != ALARM_HISTORY) )
		{
			LCD_Main_Page = ALARM_HISTORY;
			LCD_Current_Page = 	LCD_Main_Page; 	
			g_Watch_IO_Control = 0;					  
		    Order_Temp[0]=WAIT_MODE;
		    USART1_SendData(1,WORK_MODE,&Order_Temp[0]);
		}
		switch(LCD_Var_Address)
		{
			case 0x6000: 						//
				 gs_Alarm_History_Refresh = 1;
				 LCD_Current_Page = 16;
				 break;							//
			case 0x6800:
				 LCD_Current_Page = 34; 				         
				 break;	
			case 0x6801: 				         		        
				 LCD_Current_Page = 16;
				 break;
			case 0x6802: 				       //���  		        
				 if(Admin_Authority)
				 {			 
					 for(j=0;j<22;j++)
					 {
					 	IIC_Temp[j]=0;
					 }
					 for(j=0;j<20;j++)
					 {
						Alarm_History_Time[j].Flag  = 0;	
	                 	Alarm_History_Display[j].Flag = 0;
						AT24CXX_Write(0x0200+j*21,IIC_Temp,21);
					 }
					 gs_Alarm_History_Refresh = 1;
				 }
				 break;
		       default:
			     break;		   	
		}
	}
	else
	{		
		if(LCD_Var_Address != 0x6A20)
		{
			LCD_Popup_Windows = PW_1_STOPROBOT;	//������ʾ��-����ֹͣ��е��
		}
	}

}



/*************************************************************************
**  ��������  LCDSystemSettingDisply()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/26
**************************************************************************/
void LCDAlarmHistoryDisply()
{
	u8 i=0;
	//--�����л�--//
	if(LCD_Pre_Page != LCD_Current_Page )	   
	{
		LCD_Pre_Page = LCD_Current_Page;
		PageChange(LCD_Pre_Page);
	}
	//--��ʷ����--//
	if(gs_Alarm_History_Refresh)
	{
		gs_Alarm_History_Refresh = 0;
		for(i=0;i<21;i++)
		{
			if(Alarm_History_Display[i].Flag==0)
			{
				if(i==0)
				{
					for(j=0;j<10;j++)		//��ʾ10��֮ǰ����
					{
						DisplayChineseString(0x6400+0x0060*j,"          ",10);
						DisplayChineseString(0x6410+0x0060*j,"        ",8);
						DisplayChineseString(0x6420+0x0060*j," ",1);
						DisplayChineseString(0x6430+0x0060*j,Alarm_String[0],30);
	
						DisplayChineseString(0x6000+0x0060*j,"          ",10);
						DisplayChineseString(0x6010+0x0060*j,"        ",8);
						DisplayChineseString(0x6020+0x0060*j," ",1);
						DisplayChineseString(0x6030+0x0060*j,Alarm_String[0],30);
					}
					break;
				}
				k1=i-1;		 //��k1���������¼
				k2=i-1;		 //��k2���������¼
				//--��ʷ����2--//
				if(i>10)	   
				{
					for(j=0;j<i-10;j++)		//��ʾ10��֮ǰ����
					{
						DisplayChineseString(0x6400+0x0060*j,Alarm_History_Display[k2-10].Day,10);
						DisplayChineseString(0x6410+0x0060*j,Alarm_History_Display[k2-10].Time,8);
						DisplayChineseString(0x6420+0x0060*j,&(Alarm_History_Display[k2-10].Value),1);
						DisplayChineseString(0x6430+0x0060*j,Alarm_String[Alarm_History_Display[k2-10].Type],30);
						k2--;
					}
					i=10;				
				}//--��ʷ����2--//
				//--��ʷ����1--//
				if(i<=10)	   
				{
					for(j=0;j<i;j++)		//��ʾ���µ�10��
					{
						DisplayChineseString(0x6000+0x0060*j,Alarm_History_Display[k1].Day,10);
						DisplayChineseString(0x6010+0x0060*j,Alarm_History_Display[k1].Time,8);
						DisplayChineseString(0x6020+0x0060*j,&(Alarm_History_Display[k1].Value),1);
						DisplayChineseString(0x6030+0x0060*j,Alarm_String[Alarm_History_Display[k1].Type],30);
						if(k1>0)
						{
							k1--;
						}
					}
				}//--��ʷ����1--//
			    break;
			}
		}//for(i=0;i<20;i++)
		k1=0;
		k2=0;
		//--������ʷ���--//
		if(Robot_Admin_Authority==ORDINARY_AUTHORITY)	 
		{
		    Display_PageCut(AlarmHistory_Clear);
			UnHiddenDisplay(0x6E01,0x6810);
		}
		else
		{
			HiddenDisplay(0x6E01);
		}
	}
	//--��ʷ����--//
}

/*************************************************************************
**  ��������  AlarmHistoryToString()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/7/18
**************************************************************************/
void AlarmHistoryToString(u8 i)
{
	Alarm_History_Display[i].Flag = Alarm_History_Time[i].Flag;
	//����
	Alarm_History_Display[i].Day[0] = 0x32;
	Alarm_History_Display[i].Day[1] = 0x30;
	Alarm_History_Display[i].Day[2] = 0x30+(u8)((Alarm_History_Time[i].year>>4)&0x0F);
	Alarm_History_Display[i].Day[3] = 0x30+(u8)( Alarm_History_Time[i].year&0x0F);
	Alarm_History_Display[i].Day[4] = 0x2d;	                                         //"-"
	Alarm_History_Display[i].Day[5] = 0x30+(u8)((Alarm_History_Time[i].month>>4)&0x0F);
	Alarm_History_Display[i].Day[6] = 0x30+(u8)( Alarm_History_Time[i].month&0x0F);
	Alarm_History_Display[i].Day[7] = 0x2d;	                                         //"-"
	Alarm_History_Display[i].Day[8] = 0x30+(u8)((Alarm_History_Time[i].day>>4)&0x0F);
	Alarm_History_Display[i].Day[9] = 0x30+(u8)( Alarm_History_Time[i].day&0x0F);
	//ʱ��
	Alarm_History_Display[i].Time[0] = 0x30+(u8)((Alarm_History_Time[i].hour>>4)&0x0F);
	Alarm_History_Display[i].Time[1] = 0x30+(u8)( Alarm_History_Time[i].hour&0x0F);
	Alarm_History_Display[i].Time[2] = 0x3A;	                                     //":"
	Alarm_History_Display[i].Time[3] = 0x30+(u8)((Alarm_History_Time[i].minute>>4)&0x0F);
	Alarm_History_Display[i].Time[4] = 0x30+(u8)( Alarm_History_Time[i].minute&0x0F);
	Alarm_History_Display[i].Time[5] = 0x3A;	                                     //":"
	Alarm_History_Display[i].Time[6] = 0x30+(u8)((Alarm_History_Time[i].second>>4)&0x0F);
	Alarm_History_Display[i].Time[7] = 0x30+(u8)( Alarm_History_Time[i].second&0x0F);

	
	Alarm_History_Display[i].Type = Alarm_History_Time[i].Type;
	for(j=0;j<=i;j++)
	{
		Alarm_History_Display[j].Value = 0x30;
		if(Alarm_History_Display[j].Type == Alarm_History_Display[i].Type)
		{
			Alarm_History_Display[j].Value = 0x31;
		}
	}
	//дIIC	   2016-03-17
	    IIC_Temp[0] = Alarm_History_Display[i].Flag ;

	    IIC_Temp[1] = Alarm_History_Display[i].Day[0] ;
		IIC_Temp[2] = Alarm_History_Display[i].Day[1] ;
		IIC_Temp[3] = Alarm_History_Display[i].Day[2] ;
		IIC_Temp[4] = Alarm_History_Display[i].Day[3] ;
		IIC_Temp[5] = Alarm_History_Display[i].Day[4] ;
		IIC_Temp[6] = Alarm_History_Display[i].Day[5] ;
		IIC_Temp[7] = Alarm_History_Display[i].Day[6] ;
		IIC_Temp[8] = Alarm_History_Display[i].Day[7] ;
		IIC_Temp[9] = Alarm_History_Display[i].Day[8] ;
		IIC_Temp[10]= Alarm_History_Display[i].Day[9] ;

	    IIC_Temp[11] = Alarm_History_Display[i].Time[0] ;
		IIC_Temp[12] = Alarm_History_Display[i].Time[1] ;
		IIC_Temp[13] = Alarm_History_Display[i].Time[2] ;
		IIC_Temp[14] = Alarm_History_Display[i].Time[3] ;
		IIC_Temp[15] = Alarm_History_Display[i].Time[4] ;
		IIC_Temp[16] = Alarm_History_Display[i].Time[5] ;
		IIC_Temp[17] = Alarm_History_Display[i].Time[6] ;
		IIC_Temp[18] = Alarm_History_Display[i].Time[7] ;
		IIC_Temp[19] = Alarm_History_Display[i].Value ;
		IIC_Temp[20] = Alarm_History_Display[i].Type ;		

	 	AT24CXX_Write(0x0200+i*21,IIC_Temp,21);
	//дIIC	   2016-03-17


}


/*************************************************************************
**  ��������  LCDError()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/26
**************************************************************************/
void LCDError()
{
	u8 i=0;
	u8 m=0;

	//ͨ�Ŵ�����
//	if(USART1ErrorFlag==TRUE)
//	{
//		Robot_Error_Data[0] = Robot_Error_Data[0]|0x80;
//	}

	for(i=0;i<5;i++)
	{
		if(Robot_Error_Data[i]==0)	 //�ޱ���
		{
			g_Robot_Error_Flag=FALSE;
		}
		else						 //�б���
		{
			for(m=0;m<8;m++)
			{
				if(Robot_Error_Data[i]&(0x01<<m))	//���ո�λ-��λ˳���ѯ��ǰ����
				{
					g_Current_Alarm = i*8+m;		//��ͣ���������ڸ�λ
					break;
				}
			}			
			g_Robot_Error_Flag=TRUE;
			break;
		}
	}
	//�ޱ���
	if((g_Robot_Error_Flag==FALSE) && (gs_Robot_Error_Flag != g_Robot_Error_Flag))
	{
		if(g_Emergency_Stop_Flag)
		{
			g_Emergency_Stop_Flag=FALSE;
		}
		DisplayChineseString(0x6950,Alarm_String[0],30);   //�����ı�����ʾ������
		DisplayChineseString(0x6A00,Alarm_String_QC1,4);
		DisplayChineseString(0x6900,Alarm_String[1],30);   //�ޱ����ı�����ʾ���ޱ���
		gs_Robot_Error_Flag = g_Robot_Error_Flag;
		g_Alarm_Buzzer = FALSE;
		g_Alarm_Buzzer_Timer = FALSE;
		g_Alarm_Buzzer_Count = 0;	
		g_Current_Alarm=0;	
		gs_Current_Alarm = g_Current_Alarm;		
	}
	else if( (g_Robot_Error_Flag==TRUE) && (gs_Current_Alarm != g_Current_Alarm) )  //�����������-��¼��ǰ�������ͣ���ȡ��ǰ����ʱ��
	{

		if(Robot_Status == ACTION_RUN)
		{
			Robot_Status = ACTION_PAUSE;	 //��е�ֽ�����ͣ״̬
		}
		//������ͣ����
		if((g_Current_Alarm>=8) && (g_Current_Alarm<=15)) //if(Robot_Error_Data[i]!=0)
		{
			if(g_Current_Alarm==15)
			{
				g_Emergency_Stop_Flag=TRUE;
			}
			else
			{
				g_Emergency_Stop_Flag=FALSE;
			}
			Robot_Origined=FALSE;
			g_Robot_Auto_Reset = FALSE;
			//������ڻ�ԭ�����,��ص�������
			if(LCD_Current_Page==38)
			{				
				LCD_Current_Page=START_UP;
				Robot_Origining=FALSE;
			}
			if(LCD_Current_Page!=START_UP)  //��ͣ����-������ҳ����ص���ҳ��
			{
				Program_Or_Point=0;
				LCD_Current_Page=START_UP;
				LCD_Main_Page=START_UP;		
			}

		    Robot_Status = ACTION_STOP;	 //��е�ֽ���ֹͣ״̬
		}
//		if(g_Current_Alarm==21)
//		{
//			Robot_Status = ACTION_STOP;	 //��ɵ�ǰ�ӹ����񣻻�е�ֽ���ֹͣ״̬
//	    }
		g_Alarm_Buzzer = TRUE;	

		ReadRTC();
		gs_Current_Alarm = g_Current_Alarm;
		//������ʷ-ʵʱʱ�Ӷ�ȡ
			//--��ǰ����--//
		DisplayChineseString(0x6900,Alarm_String[0],30); //�ޱ����ı�����ʾ������
		DisplayChineseString(0x6950,Alarm_String[g_Current_Alarm],30);
		DisplayChineseString(0x6A00,Alarm_String_QC2,4);
		//gs_Alarm_Refresh = FALSE;	
		gs_Robot_Error_Flag = g_Robot_Error_Flag;
	}
	else
	{
				
	}
	if(LCD_RTC_Reg_len == 0x2007)
	{
		LCD_RTC_Reg_len = 0;
		if(Alarm_History_Time[19].Flag==1)
		{
			for(i=0;i<19;i++)
			{
				Alarm_History_Time[i].Flag  = Alarm_History_Time[i+1].Flag;
				Alarm_History_Time[i].year  = Alarm_History_Time[i+1].year;
				Alarm_History_Time[i].month = Alarm_History_Time[i+1].month;
				Alarm_History_Time[i].day   = Alarm_History_Time[i+1].day;
				Alarm_History_Time[i].week  = Alarm_History_Time[i+1].week;
				Alarm_History_Time[i].hour  = Alarm_History_Time[i+1].hour;
				Alarm_History_Time[i].minute= Alarm_History_Time[i+1].minute;
				Alarm_History_Time[i].second= Alarm_History_Time[i+1].second;
				Alarm_History_Time[i].Type  = Alarm_History_Time[i+1].Type;				
			    AlarmHistoryToString(i);   //�����ݽ����ı������Ա���ʾ
			}
			Alarm_History_Time[19].Flag=1;
			Alarm_History_Time[19].year  = 0x2000|((0xFF000000&LCD_RTC_Day)>>24);
			Alarm_History_Time[19].month = (0x00FF0000&LCD_RTC_Day)>>16;
			Alarm_History_Time[19].day   = (0x0000FF00&LCD_RTC_Day)>>8;
			Alarm_History_Time[19].week  = (0x000000FF&LCD_RTC_Day);
			Alarm_History_Time[19].hour  = (0xFF000000&LCD_RTC_Time)>>24;
			Alarm_History_Time[19].minute= (0x00FF0000&LCD_RTC_Time)>>16;
			Alarm_History_Time[19].second= (0x0000FF00&LCD_RTC_Time)>>8;
			Alarm_History_Time[19].Type  = g_Current_Alarm;
			//g_Current_Alarm = 0;
			AlarmHistoryToString(19);   //�����ݽ����ı������Ա���ʾ
			gs_Alarm_History_Refresh=1;
		}
		else
		{
			for(i=0;i<20;i++)
			{
				if(Alarm_History_Time[i].Flag==0)
				{
					Alarm_History_Time[i].Flag=1;
					Alarm_History_Time[i].year  = 0x2000|((0xFF000000&LCD_RTC_Day)>>24);
					Alarm_History_Time[i].month = (0x00FF0000&LCD_RTC_Day)>>16;
					Alarm_History_Time[i].day   = (0x0000FF00&LCD_RTC_Day)>>8;
					Alarm_History_Time[i].week  = (0x000000FF&LCD_RTC_Day);
					Alarm_History_Time[i].hour  = (0xFF000000&LCD_RTC_Time)>>24;
					Alarm_History_Time[i].minute= (0x00FF0000&LCD_RTC_Time)>>16;
					Alarm_History_Time[i].second= (0x0000FF00&LCD_RTC_Time)>>8;
					Alarm_History_Time[i].Type  = g_Current_Alarm;
					//g_Current_Alarm = 0;
					AlarmHistoryToString(i);   //�����ݽ����ı������Ա���ʾ
					gs_Alarm_History_Refresh=1;
					break;
				}
			}
		}
	}


	//���±���ȡ����
	if( (g_Robot_Error_Flag==TRUE) && (LCD_Var_Address == 0x6A20) )
	{
		g_Alarm_Buzzer = FALSE;
		g_Alarm_Buzzer_Timer = FALSE;
		g_Alarm_Buzzer_Count = 0;
		DisplayChineseString(0x6950,Alarm_String[g_Current_Alarm],30);
		//ֹͣ����
		if(g_Program_Is_Debuging)
		{
			Robot_Status = ACTION_STOP;	 //��е�ֽ���ֹͣ״̬
			g_Program_Is_Debuging=FALSE;
			g_Current_Program_Debug=TRUE;
		}
		//��е�����ͱ���
		if(g_Current_Alarm>=8)
		{
			Order_Temp[0]=ALARM_CLEAR;
			Order_Temp[1]=0;  //g_Current_Alarm-8;	//����ı������
		    USART1_SendData(2,WATCH_COMMAND,&Order_Temp[0]);
			if(g_Current_Alarm==21)
			{
				SC_Parameter.SC_Num=0;
			}
		}
		//ʾ�������ͱ���
		else
		{
			Robot_Error_Data[0]=0;
		}
		if(LCD_Main_Page != START_UP)
		{
			Robot_Status = ACTION_STOP;	 //��е�ֽ�����ͣ״̬
		}	
		if(g_Current_Alarm==22)	
		{
			//LCD_Main_Page=START_UP;
			//LCD_Current_Page = 116;		//������ѷ
		}			
	}
	//������
	if(g_Alarm_Buzzer)
	{
		if(g_Alarm_Buzzer_Timer)
		{
			BuzzerRing(20);
			//����������˸
			if(gs_Alarm_Blink)
			{
				DisplayChineseString(0x6950,Alarm_String[0],30);  //��ʾ�հ�
				gs_Alarm_Blink=0;
			}
			else
			{
				DisplayChineseString(0x6950,Alarm_String[g_Current_Alarm],30);  //��ʾ����
				gs_Alarm_Blink=1;
			}
			g_Alarm_Buzzer_Timer = FALSE;
			g_Alarm_Buzzer_Count = 0;
		}
	}


}





