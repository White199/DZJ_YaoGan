/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : 
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 
* Description        : This is the ....
***************************************************************************************/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "DGUS.h"
#include "Usart.h"
#include "SystemSetting.h"
#include "manuloperate.h"
u8 Usart1ReceiveData[60]={0}; //����1��������
u8 LCD_RealTime_Count = 0;	  //ʵʱ���Ӽ���
u8 LCD_RealTime_Watch=FALSE;  //ʵʱ���ӱ�־λ
u8 LCD_Position_Watch=FALSE;  //ʵʱ�����ȡ
u8 Manual_Pulse_Count = 0;
u8 Manual_Pulse_Watch=FALSE;;

static u8 Data_Temp[4]={0};
u8 Code=0; //������
u8 Time=0; //ÿ�������ȡ�Ĵ���
u16 Count =0 ;
u8 Read_IIC_Done = TRUE;  //��ȡIIC�����־����һ��ֱ�ӿ��Բ�ѯ��û�гɹ�����������ʱʱ�������ѯ
u8 Read_IIC_Count = 0;
u8 Read_IIC_Flag = FALSE;
u8 Read_Program_Over=0;
extern u8 LC_ID,gs_Run_Program_Color, LCD_Control_Flag;
extern u16 Point_Num,Order_Num;
extern u8 Program_Transport_Count;
extern u8 Program_Transport_Flag;
static u8 f_readI2C_step=1;		//��ȡIIC�������裺һ��6��
extern LCParameter Temp_LC_Parameter_Load[LC_MAXNUM];        //���ϲ�
extern u8 Program_Flag[SAVEPROGRAMNUM],Display_Flag;
extern JXSParameter Temp_JXS_Parameter;
extern SoftLimit  Temp_Soft_Limit1[6];				//XZYOAB
extern DZJSpeed DZJSpeed1;
extern u8 ChuJiao1,ChuJiao2,ChuJiao3,JiaoTouXiaYi1,JiaoTouXiaYi2,JiaoTouXiaYi3,
XiZuan1,XiZuan2,XiZuan3,XiZuan4,XiZuan5,XiZuan6,XiZuan7,XiZuan8,
ZuanTouXiaYi1,ZuanTouXiaYi2,ZuanTouXiaYi3,ZuanTouXiaYi4,ZuanTouXiaYi5,ZuanTouXiaYi6,ZuanTouXiaYi7,ZuanTouXiaYi8;
extern u32 IO_Choose;
extern u8 IO_Parameter_Refresh;
/*************************************************************************
**  ��������  LCDRealTimeWatch()
**	���������LCD�������Ӻ���
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/5/28
**************************************************************************/
void LCDRealTimeWatch()
{
	//ʵʱ��ѯ��Ҫ�Ĳ���  50ms
	if((LCD_RealTime_Watch) &&(Landing_Success))		
	{
		/*
		//��ȡʹ���ź� PC13
		if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13))  //�͵�ƽ ʧ��
		{
			Robot_Enable_Switch_On = FALSE;
		}
		else										  //�ߵ�ƽ ʹ����
		{
			if(Robot_Error_Data[0] & 0x40)//��������֮��,ʹ��
			{
				Robot_Error_Data[0] = Robot_Error_Data[0]&0xbf;	//��������
			}
			Robot_Enable_Switch_On = TRUE;
		}
		*/				
		//������ѯ
		Data_Temp[0]=ROBOT_STATUS; 
		USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
//	
		if(Robot_Origining)	    //Z�����ڻ�ԭ�㣬��ѯ�Ƿ��Ѿ���ԭ��
		{
 			Data_Temp[0]=ROBOT_ORIGINED; 
			USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
		} 
	
/*		if(g_Watch_IO_Control)	 //IO����
		{
//				//��ѯ���IO״̬
//				Order_Temp[0]=IO_DEBUG_OUTPUT1_LCD;
//				USART1_SendData(1,WATCH_COMMAND,&Order_Temp[0]);
			switch(g_Watch_IO_Control)
			{
				case IO_DEBUG_LOCAL:
					 Data_Temp[0]=IO_DEBUG_LOCAL;
				     USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
				     break;
				case IO_DEBUG_INPUT1:
					 Data_Temp[0]=IO_DEBUG_INPUT1;
				     USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
				     break;		
				case IO_DEBUG_INPUT2:
					 Data_Temp[0]=IO_DEBUG_INPUT1;//Data_Temp[0]=IO_DEBUG_INPUT2;
				     USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
				     break;		
				case IO_DEBUG_INPUT3:
					 Data_Temp[0]=IO_DEBUG_INPUT1;//Data_Temp[0]=IO_DEBUG_INPUT3;
				     USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
				     break;
				case IO_DEBUG_OUTPUT1:
					 Order_Temp[0]=IO_DEBUG_OUTPUT1_LCD;
					 USART1_SendData(1,WATCH_COMMAND,&Order_Temp[0]);
				     break;
				case IO_DEBUG_OUTPUT2:
					 Data_Temp[0]=IO_DEBUG_OUTPUT1_LCD;
				     USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
				     break;					 
				case IO_DEBUG_OUTPUT3:
					 Data_Temp[0]=IO_DEBUG_OUTPUT1_LCD;
				     USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
				     break;					 				 
				default:
				     break;	 		
			}
		}
*/
		//ʵʱλ�ò�ѯ--�Զ����е�ʱ��	�Զ�������ѯ ���� ��ǰ������
		if((Robot_Origined==TRUE) && ((Robot_Status == ACTION_RUN)||Robot_Status == ACTION_RESET))
		{
			Axis_Position_Scan=TRUE;
		}
		if(Axis_Position_Scan)
		{
			 Data_Temp[0]=X_AXSIS_POSITION; 
			 USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);		
			 Data_Temp[0]=Z_AXSIS_POSITION; 
			 USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);		
			 Data_Temp[0]=L_AXSIS_POSITION; 
			 USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
			 Data_Temp[0]=O_AXSIS_POSITION; 
			 USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
			 Data_Temp[0]=A_AXSIS_POSITION; 
			 USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
			 Data_Temp[0]=B_AXSIS_POSITION; 
			 USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
		}
		if((Axis_Position_Scan==TRUE)&&(g_XAxis_Position == g_Pre_XAxis_Position)&&(g_ZAxis_Position == g_Pre_ZAxis_Position) 
		                             &&(g_LAxis_Position == g_Pre_LAxis_Position)&&(g_OAxis_Position == g_Pre_OAxis_Position) 
																 &&(g_AAxis_Position == g_Pre_VAxis_Position)&&(g_BAxis_Position == g_Pre_WAxis_Position))
		{
			Axis_Position_Scan=FALSE;		
		}
		else
		{
			 g_Pre_XAxis_Position = g_XAxis_Position;
			 g_Pre_ZAxis_Position = g_ZAxis_Position;
			 g_Pre_LAxis_Position = g_LAxis_Position;
			 g_Pre_OAxis_Position = g_OAxis_Position;
			 g_Pre_VAxis_Position = g_AAxis_Position;
			 g_Pre_WAxis_Position = g_BAxis_Position;
		}

		//------------------//
		if(g_Program_Is_Debuging==FALSE)
		{
			if(Robot_Status == ACTION_RUN)
			{
				 Data_Temp[0]=AUTO_PARAMETER; 
				 USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
			}
			else if((Robot_Status == ACTION_PAUSE)&&(Robot_Auto_Mode == SINGLE_MODE))
			{
				 Data_Temp[0]=AUTO_PARAMETER; 
				 USART1_SendData(1,WATCH_COMMAND,&Data_Temp[0]);
			}
		}
		//��е�����ɱ�̵���״̬��ѯ
		else  //if(g_Program_Is_Debuging)
		{
			 Order_Temp[0]=ROBOT_DEBUG_STATUS;
		     USART1_SendData(1,WATCH_COMMAND,&Order_Temp[0]);		
		}
	   //��е�ָ�λ״̬��ѯ
		if(g_Robot_Auto_Reset_Scan)
		{
			 Order_Temp[0]=ACTION_RESET_SCAN;
		     USART1_SendData(1,WATCH_COMMAND,&Order_Temp[0]);
		}

		LCD_RealTime_Watch=FALSE;
		LCD_RealTime_Count = 0;
	}//ʵʱ��ѯ��Ҫ�Ĳ���

	//��е��΢����ѯ 20ms
	if((LCD_Current_Page==109)&&(Manual_Pulse_Watch==TRUE))
	{
			 g_Manule_Pulse=TIM_GetCounter(TIM3);
			 if((g_Manule_Pulse>0xFF00)||(g_Manule_Pulse<0x00FF))
			 {
				 TIM3->CNT =0x7FFF;
				 g_Manule_Pulse=0x7FFF;
				 gs_Manule_Pulse=0x7FFF;			 
			 }
			 if((g_Manule_Pulse>gs_Manule_Pulse)&&((g_Manule_Pulse-gs_Manule_Pulse)<=2))//��ת	���ٿ��ܳ���1/2���������  1��ʾ����
			 {
				 if((g_Manule_Pulse%2)==1)	  //����  ��������2 ����1  
				 {					 
					 if((g_Manule_Pulse-gs_Manule_Pulse)==2)
					 {
						 gs_Manule_Pulse=g_Manule_Pulse;
						 if(g_Current_Manul_Axis==X_AXIS)	   //x��
						 {
							  Order_Temp[0] =XAXIS_MOVE_RIGHT;
						 }
						 else if(g_Current_Manul_Axis==Z_AXIS)	   //Z��
						 {
							  Order_Temp[0] =ZAXIS_MOVE_RIGHT;			 
						 } 
						 else if(g_Current_Manul_Axis==L_AXIS)	   //L��
						 {
							 Order_Temp[0] = LAXIS_MOVE_RIGHT;					 
						 } 
						 else if(g_Current_Manul_Axis==O_AXIS)	   //O��
						 {
							 Order_Temp[0] = OAXIS_MOVE_RIGHT;					 
						 } 
					 	 Order_Temp[1] = JOG_MOVE_MODE;	//�綯
						 Order_Temp[2] = 1;	
						 USART1_SendData(3,MANUL_OPERATE,&Order_Temp[0]);
						 Axis_Position_Scan=TRUE;
					 }
				 }					 
			 }
			 else if((g_Manule_Pulse<gs_Manule_Pulse)&&((gs_Manule_Pulse-g_Manule_Pulse)<=2))//��ת
			 {
				 if((g_Manule_Pulse%2)==1)	  //����  ��������2 ����1  
				 {	
				 	 if((gs_Manule_Pulse-g_Manule_Pulse)==2)
					 {	 
						 gs_Manule_Pulse=g_Manule_Pulse;
						 if(g_Current_Manul_Axis==X_AXIS)	   //x��
						 {
							  Order_Temp[0] =XAXIS_MOVE_LEFT;
						 }
						 else if(g_Current_Manul_Axis==Z_AXIS)	   //Z��
						 {
							  Order_Temp[0] =ZAXIS_MOVE_LEFT;				 
						 } 
						 else if(g_Current_Manul_Axis==L_AXIS)	   //L��
						 {
							 Order_Temp[0] = LAXIS_MOVE_LEFT;					 
						 } 
						 else if(g_Current_Manul_Axis==O_AXIS)	   //O��
						 {
							 Order_Temp[0] = OAXIS_MOVE_LEFT;					 
						 } 
					 	 Order_Temp[1] = JOG_MOVE_MODE;	//�綯
						 Order_Temp[2] = 1;	
						 USART1_SendData(3,MANUL_OPERATE,&Order_Temp[0]);
						 Axis_Position_Scan=TRUE;	
					 }
				 }			 
			 }
			 else if(gs_Manule_Pulse!=g_Manule_Pulse)		  //ת�ܿ죬������
			 {
			     gs_Manule_Pulse=g_Manule_Pulse;
			 }
			 Manual_Pulse_Watch=FALSE;
			 Manual_Pulse_Count=0;
	}//��е��΢����ѯ	
	
//---2017.12.04��е�����ɱ����ת��ť
	if((LCD_Current_Page==FREE_PROGRAMME)&&(Manual_Pulse_Watch==TRUE))
	{
			g_Manule_Pulse=TIM_GetCounter(TIM3);
			if((g_Manule_Pulse>0xFF00)||(g_Manule_Pulse<0x00FF))
			{
					TIM3->CNT =0x7FFF;
					g_Manule_Pulse=0x7FFF;
					gs_Manule_Pulse=0x7FFF;			 
			}
			if((g_Manule_Pulse>gs_Manule_Pulse)&&((g_Manule_Pulse-gs_Manule_Pulse)<=2))//��ת	���ٿ��ܳ���1/2���������  1��ʾ����
			{
			 if((g_Manule_Pulse%2)==1)	  //����  ��������2 ����1  
			 {					 
				 if((g_Manule_Pulse-gs_Manule_Pulse)==2)
				 {
						gs_Manule_Pulse=g_Manule_Pulse;
						LCD_Var_Address=0x2006;
					  LCD_New_Order=TRUE;
				 }
			 }					 
			}
			else if((g_Manule_Pulse<gs_Manule_Pulse)&&((gs_Manule_Pulse-g_Manule_Pulse)<=2))//��ת
			{
			 if((g_Manule_Pulse%2)==1)	  //����  ��������2 ����1  
			 {	
				 if((gs_Manule_Pulse-g_Manule_Pulse)==2)
				 {	 
						gs_Manule_Pulse=g_Manule_Pulse;
						LCD_Var_Address=0x2005;
						LCD_New_Order=TRUE;					 
				 }
			 }			 
			}
			else if(gs_Manule_Pulse!=g_Manule_Pulse)		  //ת�ܿ죬������
			{
				 gs_Manule_Pulse=g_Manule_Pulse;
			}
			Manual_Pulse_Watch=FALSE;
			Manual_Pulse_Count=0;
	}//---2017.12.04��е�����ɱ����ת��ť
}

/*************************************************************************
**  ��������  MainBoard_InitDone()
**	�����������е�ֿ�����ʼ�����
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/11/03
**************************************************************************/
void MainBoard_InitDone()
{
	u8 data[1]; 
	data[0]=ROBOT_PRE_STATUS;	             //��е������״̬
	while(!Data_Syc_Ready)
	{
		if(LCD_RealTime_Watch)	             //100ms��ѯһ�� 
		{
			USART2_SendDataArray(1,WATCH_COMMAND,data);
			LCD_RealTime_Watch=FALSE;
			LCD_RealTime_Count = 0;
		}
		if(USART1RecieveFinishedFlag==TRUE)  //---��û���յ����ذ巢����ͨ����ɱ�־λʱ��һֱ�����whileѭ�����档
		{		    
			if(USART1ReceiveDataBuffer[2] == M_ROBOT_PRE_STATUS)
			{
				Data_Syc_Ready=1;
			}
			USART1RecieveFinishedFlag=FALSE;
		}	
	}
	USART1RecieveFinishedFlag=FALSE;
}

/*************************************************************************
**  ��������  DataSyc()
**	�����������е�ֿ���ͬ������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/11/03
**************************************************************************/
u8 DataSyc()
{
	u8 data[2]={0}; 
	switch(f_readI2C_step)
	{
		//����IIC������1-���ò���
		case 1:
		  data[0] = READ_IIC_1;			                  //---��Ӧ��λ��parameter.c1045(ReadIICData())
			while(USART1RecieveFinishedFlag!=TRUE)
			{
				if(Read_IIC_Done)
				{
					USART2_SendDataArray(1,READ_IIC,data);  //---���Ϳ�����IIC����READ_IIC��������data[0]=READ_IIC_1��0x01��
					USART1RecieveFinishedFlag = FALSE;
					Read_IIC_Done=FALSE;
					Read_IIC_Flag = TRUE;
				}
			}
			USART1RecieveDataDecode();                  //����1����Ӧ������ 
			USART1RecieveFinishedFlag = FALSE;
			if(Setting_Syc_Ready)
			{
				f_readI2C_step++;
				Read_IIC_Done = TRUE;
				Read_IIC_Flag = FALSE;
				Read_IIC_Count = 0;
			}
			break;
	   
	  //����IIC������2-�洢�����
		case 2:		    
		  data[0] = READ_IIC_2;
			while(USART1RecieveFinishedFlag!=TRUE)
			{
				if(Read_IIC_Done)
				{
					USART2_SendDataArray(1,READ_IIC,data);
					USART1RecieveFinishedFlag = FALSE;
					Read_IIC_Done=FALSE;
					Read_IIC_Flag = TRUE;
				}
			}
			USART1RecieveDataDecode();                  //����1����Ӧ������ 
			USART1RecieveFinishedFlag = FALSE;
			if(Point_Syc_Ready)
			{
				f_readI2C_step++;
				Read_IIC_Done = TRUE;
				Read_IIC_Flag = FALSE;
				Read_IIC_Count = 0;
			}
			break;

    //����IIC������3-�洢������Ϣ����
		case 3:		    
		  data[0] = READ_IIC_3;			    
			while(USART1RecieveFinishedFlag!=TRUE)
			{
				if(Read_IIC_Done)
				{
					//Count++;	  			 //��ǳ����ȡ����
					USART2_SendDataArray(1,READ_IIC,data);
					USART1RecieveFinishedFlag = FALSE;
					Read_IIC_Done=FALSE;
					Read_IIC_Flag = TRUE;
				}
			}
			USART1RecieveDataDecode();                  //����1����Ӧ������ 
			USART1RecieveFinishedFlag = FALSE;
			Count=0;
			if(ProgramInf_Syc_Ready)
			{
				f_readI2C_step++;
				//Count=0;
				Read_IIC_Done = TRUE;
				Read_IIC_Flag = FALSE;
				Read_IIC_Count = 0;
			}
			break;

		//����IIC������5-����λ����
		case 4:		    
		  data[0] = READ_IIC_5;			    
			while(USART1RecieveFinishedFlag!=TRUE)
			{
				if(Read_IIC_Done)
				{
					USART2_SendDataArray(1,READ_IIC,data);
					USART1RecieveFinishedFlag = FALSE;
					Read_IIC_Done=FALSE;
					Read_IIC_Flag = TRUE;
				}
			}
			USART1RecieveDataDecode();                  //����1����Ӧ������ 
			USART1RecieveFinishedFlag = FALSE;
			if(SoftLimit_Syc_Ready)
			{
				f_readI2C_step++;
				Read_IIC_Done = TRUE;
				Read_IIC_Flag = FALSE;
				Read_IIC_Count = 0;
			}
			break;
		//����IIC������6-��ȫ������					
		case 5:
		  data[0] = READ_IIC_6;			    
			while(USART1RecieveFinishedFlag!=TRUE)
			{
				if(Read_IIC_Done)
				{
					USART2_SendDataArray(1,READ_IIC,data);
					USART1RecieveFinishedFlag = FALSE;
					Read_IIC_Done = FALSE;
					Read_IIC_Flag = TRUE;
				}
			}
			USART1RecieveDataDecode();   //����1����Ӧ������ 
			USART1RecieveFinishedFlag = FALSE;
			if(SafeArea_Syc_Ready)
			{
				f_readI2C_step++;
				Read_IIC_Done = TRUE;
				Read_IIC_Flag = FALSE;
				Read_IIC_Count = 0;
			}
			break;
		//����IIC������7-IO������
		case 6:
		  data[0] = READ_IIC_7;
			while(USART1RecieveFinishedFlag!=TRUE)
			{
				if(Read_IIC_Done)
				{
					USART2_SendDataArray(1,READ_IIC,data);
					USART1RecieveFinishedFlag = FALSE;
					Read_IIC_Done = FALSE;
					Read_IIC_Flag = TRUE;
				}
			}
			USART1RecieveDataDecode();   //����1����Ӧ������ 
			USART1RecieveFinishedFlag = FALSE;
			if(IODetect_Syc_Ready)
			{
				f_readI2C_step++;
				Read_IIC_Done = TRUE;
				Read_IIC_Flag = FALSE;
				Read_IIC_Count = 0;
			}
			break;
			
//		case 7:

//			//USART2_SendDataArray(1,READ_IIC,&data);//����IIC������6-����״̬
//			break;

		default:
		    break;
		
	}
	if(f_readI2C_step==6)
	{
		f_readI2C_step=1;
		return (u8)1;
	}
	else 
	{
		return (u8)0;	
	}
}

/*************************************************************************
**  ��������  Robot_Setting()
**	�����������е��ʹ�ܼ�ԭ���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/25
**************************************************************************/
void Robot_Setting()
{
	switch(USART1_Current_Key)//���͵�key(������)
	{
		case ROBOT_ORIGIN:				
			 Robot_Origining = TRUE;
			 Robot_Origined = FALSE;
			 LCD_Current_Page = 38;			 
			 break;
		
		case X_AXSIS_ORIGIN:
			 //X_Axsis_Origining = TRUE;		 //X�����ڻ�ԭ��
		
			 break;
		case Z_AXSIS_ORIGIN:
			 //Z_Axsis_Origining = TRUE;		 //X�����ڻ�ԭ��		
			 break;
		case ORIGIN_SPEED:
			 //Z_Axsis_Origining = TRUE;		 //		
		     break;	
		case ROBOT_ENABLE:
			 if(Robot_Enable_Switch_On)
			 {
				Robot_Enable = TRUE;
				g_Robot_Enable_Flag = 1;
		     }
			 else
			 {
				Robot_Enable = FALSE;
				g_Robot_Enable_Flag = 0;
				Robot_Origined = FALSE;
			 }
			 break;			 		
		default:
		     break;	 		 
	}
}

/*************************************************************************
**  ��������  Auto_Run()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/25
**************************************************************************/
void Auto_Run()
{
	switch(USART1_Current_Key)//���͵�key(������)
	{
		case CYCLE_MODE:				
			 g_Cycle_Run_Flag = 1;
			 Robot_Auto_Mode = CYCLE_MODE;
			 break;		
		case SINGLE_MODE:
			 g_Cycle_Run_Flag = 0;
			 Robot_Auto_Mode = SINGLE_MODE;
			 break;
		case ACTION_RUN:
			 g_Robot_Run_Flag = 1;
			 g_Robot_Pause_Flag = 0;
			 g_Robot_Stop_Flag = 0;
			 Robot_Status = ACTION_RUN;
			 break;
		case ACTION_PAUSE:
			 g_Robot_Run_Flag = 0;
			 g_Robot_Pause_Flag = 1;
			 Robot_Status = ACTION_PAUSE;		 
		     break;	
		case ACTION_STOP:
			 g_Robot_Run_Flag = 0;
			 g_Robot_Pause_Flag = 0;
			 g_Robot_Stop_Flag = 1;
			 Robot_Status = ACTION_STOP;	
			 if(LCD_Control_Flag)//ֹͣ����и�λ
			 {
				 LCD_Control_Flag=0;
				 Robot_Status = ACTION_RESET;	
				 Order_Temp[0]=ACTION_RESET;                 //���û�е�ָ�λ
				 USART1_SendData(1,AUTO_RUN,&Order_Temp[0]); //���ڷ��ͻ�е���Զ������еĸ�λ�����ذ�
				 LCD_Current_Page = 89;
			 }		
			 break;	
		case ACTION_PROGRAM:
			 break;	
		case ACTION_RESET:
			 g_Robot_Auto_Reset_Scan = TRUE;
			 break;				 			 		 		
		case ACTION_LORIGIN:
			 g_Robot_Auto_Reset_Scan = TRUE;
			 break;	
		default:
		     break;	 		 
	}

}

/*************************************************************************
**  ��������  Manul_Operate()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/25
**************************************************************************/
void Manul_Operate()
{
	switch(USART1_Current_Key)//���͵�key(������)
	{
		case XAXIS_MANUL_SPEED:				

			 break;		
		case XAXIS_MOVE_LEFT:
			 if(USART1_Current_Num)
			 {
				 //Robot_Status = ACTION_RUN;
				 g_Axis_Choose = X_AXIS;
			 }
			 else
			 {
				 //Robot_Status = ACTION_STOP;
				 g_Axis_Choose = NO_AXIS;
			 }
			 break;
		case XAXIS_MOVE_RIGHT:
 			 if(USART1_Current_Num)
			 {
				 //Robot_Status = ACTION_RUN;
				 g_Axis_Choose = X_AXIS;
			 }
			 else
			 {
				 //Robot_Status = ACTION_STOP;
				 g_Axis_Choose = NO_AXIS;
			 }
			 break;
		case XAXIS_STEP_STOP:
		 	 Robot_Status = ACTION_STOP;
		     break;	
		case XAXIS_SAVE_POINT:
			 //g_X_SavePoint_Flag = TRUE;
			 break;			 		
		case ZAXIS_MANUL_SPEED:				
			 break;		
		case ZAXIS_MOVE_LEFT:
			 if(USART1_Current_Num)
			 {
				 //Robot_Status = ACTION_RUN;
				 g_Axis_Choose = Z_AXIS;
			 }
			 else
			 {
				 //Robot_Status = ACTION_STOP;
				 g_Axis_Choose = NO_AXIS;
			 }
			 break;
		case ZAXIS_MOVE_RIGHT:
			 if(USART1_Current_Num)
			 {
				 //Robot_Status = ACTION_RUN;
				 g_Axis_Choose = Z_AXIS;
			 }
			 else
			 {
				 //Robot_Status = ACTION_STOP;
				 g_Axis_Choose = NO_AXIS;
			 }
			 break;
		case ZAXIS_STEP_STOP:
		 	 Robot_Status = ACTION_STOP;
		     break;	
		case ZAXIS_SAVE_POINT:
			 g_Z_SavePoint_Flag = TRUE;
			 break;
		case XZAXIS_SAVE_POINT:
			 break;
		case XZAXIS_DELETE_POINT:
			 break;

		case LAXIS_MANUL_SPEED:				
			 break;		
		case LAXIS_MOVE_LEFT:
			 if(USART1_Current_Num)
			 {
				 //Robot_Status = ACTION_RUN;
				 g_Axis_Choose = L_AXIS;
			 }
			 else
			 {
				 //Robot_Status = ACTION_STOP;
				 g_Axis_Choose = NO_AXIS;
			 }
			 break;
		case LAXIS_MOVE_RIGHT:
			 if(USART1_Current_Num)
			 {
				 //Robot_Status = ACTION_RUN;
				 g_Axis_Choose = L_AXIS;
			 }
			 else
			 {
				 //Robot_Status = ACTION_STOP;
				 g_Axis_Choose = NO_AXIS;
			 }
			 break;
		case LAXIS_STEP_STOP:
		 	 Robot_Status = ACTION_STOP;
		     break;	
		case LAXIS_SAVE_POINT:
			 break;
		case OAXIS_MANUL_SPEED:				
			 break;		
		case OAXIS_MOVE_LEFT:
			 if(USART1_Current_Num)
			 {
				 //Robot_Status = ACTION_RUN;
				 g_Axis_Choose = O_AXIS;
			 }
			 else
			 {
				 //Robot_Status = ACTION_STOP;
				 g_Axis_Choose = NO_AXIS;
			 }
			 break;
		case OAXIS_MOVE_RIGHT:
			 if(USART1_Current_Num)
			 {
				 //Robot_Status = ACTION_RUN;
				 g_Axis_Choose = O_AXIS;
			 }
			 else
			 {
				 //Robot_Status = ACTION_STOP;
				 g_Axis_Choose = NO_AXIS;
			 }
			 break;
		case OAXIS_STEP_STOP:
		 	 Robot_Status = ACTION_STOP;
		     break;	
		case OAXIS_SAVE_POINT:
			 break;
		case LOAXIS_DELETE_POINT:
			 break;
		case AAXIS_MOVE_LEFT:
			 if(USART1_Current_Num)
			 {
				 //Robot_Status = ACTION_RUN;
				 g_Axis_Choose = V_AXIS;
			 }
			 else
			 {
				 //Robot_Status = ACTION_STOP;
				 g_Axis_Choose = NO_AXIS;
			 }
			 break;
		case AAXIS_MOVE_RIGHT:
			 if(USART1_Current_Num)
			 {
				 //Robot_Status = ACTION_RUN;
				 g_Axis_Choose = V_AXIS;
			 }
			 else
			 {
				 //Robot_Status = ACTION_STOP;
				 g_Axis_Choose = NO_AXIS;
			 }
			 break;
		case VAXIS_STEP_STOP:
		 	 Robot_Status = ACTION_STOP;
		     break;	
		case VAXIS_SAVE_POINT:
			 break;
		case BAXIS_MOVE_LEFT:
			 if(USART1_Current_Num)
			 {
				 //Robot_Status = ACTION_RUN;
				 g_Axis_Choose = V_AXIS;
			 }
			 else
			 {
				 //Robot_Status = ACTION_STOP;
				 g_Axis_Choose = NO_AXIS;
			 }
			 break;
		case BAXIS_MOVE_RIGHT:
			 if(USART1_Current_Num)
			 {
				 //Robot_Status = ACTION_RUN;
				 g_Axis_Choose = W_AXIS;
			 }
			 else
			 {
				 //Robot_Status = ACTION_STOP;
				 g_Axis_Choose = NO_AXIS;
			 }
			 break;
		case WAXIS_STEP_STOP:
		 	 Robot_Status = ACTION_STOP;
		     break;	
		case WAXIS_SAVE_POINT:
			 break;
		default:
		     break;	 		 
	}
}
/*************************************************************************
**  ��������  Watch_Command()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/25
**************************************************************************/
void Watch_Command()
{
	switch(Usart1ReceiveData[0]) 
	{
		case M_IO_DEBUG_LOCAL:
		     g_LocalIO_Status =	(((u16)Usart1ReceiveData[2]&0x0004)<<1)	| (((u16)Usart1ReceiveData[1]&0x0040)>>2) |
			 					(((u16)Usart1ReceiveData[2]&0x0008)<<2)	| (((u16)Usart1ReceiveData[1]&0x0080)>>1) |
			 					(((u16)Usart1ReceiveData[2]&0x0002)<<6)	| (((u16)Usart1ReceiveData[1]&0x0020)<<3) |
								(((u16)Usart1ReceiveData[2]&0x0001)<<9)	| (((u16)Usart1ReceiveData[1]&0x0010)<<6);
		     break;

		case M_IO_DEBUG_INPUT1:
		     g_Input1_Status =  ( (u16)Usart1ReceiveData[1])|(((u16)Usart1ReceiveData[2]&0x007f)<<8);

		     g_Input2_Status =  (((u16)Usart1ReceiveData[2]&0x0080)>>7)|(((u16)Usart1ReceiveData[3]&0x00ff)<<5)|(((u16)Usart1ReceiveData[4]&0x0003)<<13);
		     g_Input3_Status =  (((u16)Usart1ReceiveData[4]&0x00fc)>>2)|(((u16)Usart1ReceiveData[5]&0x000f)<<10);
			 break;

		case M_IO_DEBUG_INPUT2:
		     //g_Input2_Status = (u16)( ((u16)Usart1ReceiveData[2]<<8) | Usart1ReceiveData[1] );
		     break;

		case M_IO_DEBUG_INPUT3:
		     //g_Input3_Status = (u16)( ((u16)Usart1ReceiveData[2]<<8) | Usart1ReceiveData[1] );
		     break;

		case M_IO_DEBUG_OUTPUT1_LCD:
		     //��״̬�������Ϸ���״ֵ̬��Ӧ���� 15  
			 g_Output1_Status =  ( (u16)Usart1ReceiveData[1]) | (((u16)Usart1ReceiveData[2]&0x0007)<<8);
			 g_Output2_Status =  (((u16)Usart1ReceiveData[2]&0x00f8)>>3)|(((u16)Usart1ReceiveData[3]&0x000f)<<5)|(((u16)Usart1ReceiveData[3]&0x00f0)<<6) ;
			 g_Output3_Status =  ( (u16)Usart1ReceiveData[4])|(((u16)Usart1ReceiveData[5]&0x000f)<<8);			 		 

			 g_Output1_Status = (~g_Output1_Status) & 0x07ff;
			 g_Output2_Status = (~g_Output2_Status) & 0x3dff;
			 g_Output3_Status = (~g_Output3_Status) & 0x0fff;
			 
			 //���ս������Ԥ��λ
//			 g_Output1_Status = ((g_Output1_Status&0xfff0)<<1) | (g_Output1_Status&0x000f); //12:H-8+1+4-L
//			 g_Output2_Status = ((g_Output2_Status&0xff00)<<2) | (g_Output2_Status&0x00ff);	//12:H-4+2+8-L
//			 g_Output3_Status = ((g_Output3_Status&0xfe00)<<1) | (g_Output3_Status&0x01ff); //13:H-4+1+9-L
			 g_Output1Ack_Flag = TRUE;
//			 g_Output2Ack_Flag = TRUE;
//			 g_Output3Ack_Flag = TRUE;
		     break;

		case M_IO_DEBUG_OUTPUT2_LCD:
			gs_Current_Choose_IO1 =  ( (u16)Usart1ReceiveData[1]) | (((u16)Usart1ReceiveData[2]&0x0007)<<8);
			gs_Current_Choose_IO2 =  (((u16)Usart1ReceiveData[2]&0x00f8)>>3)|(((u16)Usart1ReceiveData[3]&0x000f)<<5)|(((u16)Usart1ReceiveData[3]&0x00f0)<<6) ;
			gs_Current_Choose_IO3 =  ( (u16)Usart1ReceiveData[4])|(((u16)Usart1ReceiveData[5]&0x000f)<<8);			 		 

			gs_Current_Choose_IO1 = (~gs_Current_Choose_IO1) & 0x07ff;
			gs_Current_Choose_IO2 = (~gs_Current_Choose_IO2) & 0x07ff;
			gs_Current_Choose_IO3 = (~gs_Current_Choose_IO3) & 0x07ff;
		
			g_IO_DetectAck_Flag = TRUE;		
			break;

		case M_IO_DEBUG_OUTPUT3_LCD:
		     break;

		case M_ROBOT_ORIGINED:			 //0xE8
	 		 Robot_Origined = Usart1ReceiveData[1];
			 if(Robot_Origined)
			 {
			 	Robot_Origining=FALSE;
				g_Robot_Origin_Flag=0;
				g_Robot_Stop_Flag = 1;	//ֹͣЧ��
				LCD_Current_Page = 1;
				g_XAxis_Position=0;
				g_ZAxis_Position=0;
				g_LAxis_Position=0;
				g_OAxis_Position=0;
				g_AAxis_Position=0;
				g_BAxis_Position=0; 
				 
				IO_Parameter_Refresh=TRUE;
				IO_Choose=0;
				ChuJiao1=0;
				ChuJiao2=0;
				ChuJiao3=0;
				JiaoTouXiaYi1=0;
				JiaoTouXiaYi2=0;
				JiaoTouXiaYi3=0;
				XiZuan1=0;
				XiZuan2=0;
				XiZuan3=0;
				XiZuan4=0;
				XiZuan5=0;
				XiZuan6=0;
				XiZuan7=0;
				XiZuan8=0;
				ZuanTouXiaYi1=0;
				ZuanTouXiaYi2=0;
				ZuanTouXiaYi3=0;
				ZuanTouXiaYi4=0;
				ZuanTouXiaYi5=0;
				ZuanTouXiaYi6=0;
				ZuanTouXiaYi7=0;
				ZuanTouXiaYi8=0;
				 
			  Robot_Status = ACTION_RESET;	
			  Order_Temp[0]=ACTION_RESET;                 //���û�е�ָ�λ
			  USART1_SendData(1,AUTO_RUN,&Order_Temp[0]); //���ڷ��ͻ�е���Զ������еĸ�λ�����ذ�
			  LCD_Current_Page = 89;
			 }
//			 else
//			 {
//			 	g_Robot_Origin_Flag=1;
//			 }    
			 break;
										 
		case M_ACTION_RESET_SCAN:	   //��λ״̬��ѯ����
			 g_Robot_Auto_Reset = Usart1ReceiveData[1];
			 if(g_Robot_Auto_Reset)
			 {
				LCD_Current_Page = START_UP;
				g_Robot_Auto_Reset_Scan = FALSE;
				Robot_Status = ACTION_STOP;
				gs_Run_Program_Color=1;
				g_Run_Program_Operater_List=0;
				 
				IO_Parameter_Refresh=TRUE;
				IO_Choose=0;
				ChuJiao1=0;
				ChuJiao2=0;
				ChuJiao3=0;
				JiaoTouXiaYi1=0;
				JiaoTouXiaYi2=0;
				JiaoTouXiaYi3=0;
				XiZuan1=0;
				XiZuan2=0;
				XiZuan3=0;
				XiZuan4=0;
				XiZuan5=0;
				XiZuan6=0;
				XiZuan7=0;
				XiZuan8=0;
				ZuanTouXiaYi1=0;
				ZuanTouXiaYi2=0;
				ZuanTouXiaYi3=0;
				ZuanTouXiaYi4=0;
				ZuanTouXiaYi5=0;
				ZuanTouXiaYi6=0;
				ZuanTouXiaYi7=0;
				ZuanTouXiaYi8=0;
			 }
		     break;

		case M_ALARM_CLEAR:
		     break;

		case M_ROBOT_STATUS:			 //������־λ			 		 
			 //Work_Status = Usart1ReceiveData[1];
			 g_Auto_Status = Usart1ReceiveData[2];
			 if((Robot_Auto_Mode == SINGLE_MODE)&&(g_Auto_Status==AUTO_PAUSE))
			 {
			 	Robot_Status = ACTION_PAUSE;
				if(g_Program_Is_Debuging)
				{
				    Robot_Status = ACTION_STOP;
				}
			 }
			 g_Servo_Is_Run = Usart1ReceiveData[3];
			 //g_Robot_Error  = (g_Robot_Error|0xff00)&((((u16)Usart1ReceiveData[4]<<8))|0x00ff);			 		 	 
			 Robot_Error_Data[1]=Usart1ReceiveData[4];
			 Robot_Error_Data[2]=Usart1ReceiveData[5];
			 Robot_Error_Data[3]=Usart1ReceiveData[6];
			 Robot_Error_Data[4]=Usart1ReceiveData[7];
			 Robot_Axis_State[X_Axsis]=Usart1ReceiveData[8];
			 Robot_Axis_State[Z_Axsis]=Usart1ReceiveData[9];
			 Robot_Axis_State[L_Axsis]=Usart1ReceiveData[10];
			 Robot_Axis_State[O_Axsis]=Usart1ReceiveData[11];
			 Robot_Axis_State[A_Axsis]=Usart1ReceiveData[12];
			 Robot_Axis_State[B_Axsis]=Usart1ReceiveData[13];
		     break;
		case M_AUTO_PARAMETER:
			 g_Run_Program_Operater_List = (Usart1ReceiveData[1]|Usart1ReceiveData[2]<<8)+1;	   //��ǰ����������
			 SC_Parameter.SC_Num = (u32)(((u32)Usart1ReceiveData[3]) |((u32)Usart1ReceiveData[4]<<8)|((u32)Usart1ReceiveData[5]<<16)|((u32)Usart1ReceiveData[6]<<24) );
			 SC_Parameter.LJ_Num = (u32)(((u32)Usart1ReceiveData[7]) |((u32)Usart1ReceiveData[8]<<8)|((u32)Usart1ReceiveData[9]<<16)|((u32)Usart1ReceiveData[10]<<24) );
//			 g_Reflash_Progress_Num = TRUE;
			 break;

		case M_X_AXSIS_POSITION:
			 g_XAxis_Position = 0;
			 g_XAxis_Position  = Usart1ReceiveData[1];
			 g_XAxis_Position |= (u32)(Usart1ReceiveData[2]<<8);
			 g_XAxis_Position |= (u32)(Usart1ReceiveData[3]<<16);
			 g_XAxis_Position |= (u32)(Usart1ReceiveData[4]<<24);	     
			 g_XAxis_Position = g_XAxis_Position-1000000;
			 Axsis_Move_Direction[X_Axsis]=Usart1ReceiveData[5];
			 g_Axis_Is_Run  = Usart1ReceiveData[6];
			 break;

		case M_Z_AXSIS_POSITION:
			 g_ZAxis_Position = 0;
			 g_ZAxis_Position  = Usart1ReceiveData[1];
			 g_ZAxis_Position |= (u32)(Usart1ReceiveData[2]<<8);
			 g_ZAxis_Position |= (u32)(Usart1ReceiveData[3]<<16);
			 g_ZAxis_Position |= (u32)(Usart1ReceiveData[4]<<24);
		   g_ZAxis_Position = g_ZAxis_Position-1000000;
			 Axsis_Move_Direction[Z_Axsis]=Usart1ReceiveData[5];
			 g_Axis_Is_Run  = Usart1ReceiveData[6];
			 break;

		case M_L_AXSIS_POSITION:
			 g_LAxis_Position = 0;
			 g_LAxis_Position  = Usart1ReceiveData[1];
			 g_LAxis_Position |= (u32)(Usart1ReceiveData[2]<<8);
			 g_LAxis_Position |= (u32)(Usart1ReceiveData[3]<<16);
			 g_LAxis_Position |= (u32)(Usart1ReceiveData[4]<<24);
		   g_LAxis_Position = g_LAxis_Position-1000000;
			 Axsis_Move_Direction[L_Axsis]=Usart1ReceiveData[5];
			 g_Axis_Is_Run  = Usart1ReceiveData[6];
			 break;

		case M_O_AXSIS_POSITION:
			 g_OAxis_Position = 0;
			 g_OAxis_Position  = Usart1ReceiveData[1];
			 g_OAxis_Position |= (u32)(Usart1ReceiveData[2]<<8);
			 g_OAxis_Position |= (u32)(Usart1ReceiveData[3]<<16);
			 g_OAxis_Position |= (u32)(Usart1ReceiveData[4]<<24);
		   g_OAxis_Position = g_OAxis_Position-1000000;
			 Axsis_Move_Direction[O_Axsis]=Usart1ReceiveData[5];
			 g_Axis_Is_Run  = Usart1ReceiveData[6];
			 break;
		
		case M_A_AXSIS_POSITION:
			 g_AAxis_Position = 0;
			 g_AAxis_Position  = Usart1ReceiveData[1];
			 g_AAxis_Position |= (u32)(Usart1ReceiveData[2]<<8);
			 g_AAxis_Position |= (u32)(Usart1ReceiveData[3]<<16);
			 g_AAxis_Position |= (u32)(Usart1ReceiveData[4]<<24);	     
			 g_AAxis_Position = g_AAxis_Position-1000000;
			 Axsis_Move_Direction[A_Axsis]=Usart1ReceiveData[5];
			 g_Axis_Is_Run  = Usart1ReceiveData[6];
			 break;

		case M_B_AXSIS_POSITION:
			 g_BAxis_Position = 0;
			 g_BAxis_Position  = Usart1ReceiveData[1];
			 g_BAxis_Position |= (u32)(Usart1ReceiveData[2]<<8);
			 g_BAxis_Position |= (u32)(Usart1ReceiveData[3]<<16);
			 g_BAxis_Position |= (u32)(Usart1ReceiveData[4]<<24);	     
			 g_BAxis_Position = g_BAxis_Position-1000000;
			 Axsis_Move_Direction[B_Axsis]=Usart1ReceiveData[5];
			 g_Axis_Is_Run  = Usart1ReceiveData[6];
			 break;
		
		case M_ROBOT_PRE_STATUS:
		 	 //g_Servo_Is_Run  = Usart1ReceiveData[2];	   //g_Robot_Running_Flag = Servo_Is_Run
			 break;

		case M_ROBOT_DEBUG_STATUS:	//����״̬
		 	 if((Usart1ReceiveData[1]+1)!=gs_Current_Operate_List) //���ز����з����仯�����ʾ���������
			 {
			 	gs_Current_Operate_List=Usart1ReceiveData[1]+1;
				if(gs_Current_Operate_List==1)	  //��������ǵ�һ�У���ʾ��ǰ���������һ������
				{
					g_Background_Color=1;
				}
				else
				{
					if(g_Background_Color<0x08)
					{
						g_Background_Color++;
					}				
				}
				Robot_Status = ACTION_STOP;
				g_Program_Is_Debuging=FALSE;
				g_Current_Program_Debug=TRUE;
				g_Refresh_Dispaly_List_Flag = TRUE;
			 }
			 break;
		case M_KEY_RUN:
			 g_Robot_Run_Flag = 1;
			 g_Robot_Pause_Flag = 0;
			 g_Robot_Stop_Flag = 0;
			 Robot_Status = ACTION_RUN;            //����״̬			
			break;
		case M_KEY_PAUSE:
			 g_Robot_Run_Flag = 0;
			 g_Robot_Pause_Flag = 1;
			 Robot_Status = ACTION_PAUSE;            //��ͣ״̬						
			break;
		case M_KEY_RESET:
			 g_Robot_Auto_Reset_Scan=TRUE;
			 LCD_Current_Page = START_UP;
			 Robot_Status = ACTION_STOP;
			 gs_Run_Program_Color=1;
		   g_Run_Program_Operater_List=0;
			 g_Robot_Auto_Reset_Scan = TRUE;
			 Robot_Status = ACTION_RESET;            //��λ״̬						
			break;
		default: 
		     break;	
	}
}


/*************************************************************************
**  ��������  ReadIICSetting()
**	�����������ȡIIC�������ò���
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/25
**************************************************************************/
void ReadIICSetting()
{
	u8 i=0;
	if((Usart1ReceiveData[1]==0xAA) && (Usart1ReceiveData[2]==25))  //�������ý������  ----��Ӧ����parameter.c �е�906 ReadIIC_Setting������
	{
		LC_ID=0;
		Setting_Syc_Ready = TRUE;	
	}
	else if(Usart1ReceiveData[1]==0x11)		//���յ���ѯ���ز���
	{
//		if(Usart1ReceiveData[2] == 1)		//���г�����
//		{
//			g_Run_Program_Num = Usart1ReceiveData[3];
//		}

	 if(Usart1ReceiveData[2] == 1)		//��е�ֲ���
		{
			JXS_Parameter.Axis      =Usart1ReceiveData[3];
//			JXS_Parameter.Origin    =Usart1ReceiveData[4];
			JXS_Parameter.OriginDir[0]=Usart1ReceiveData[5];
			JXS_Parameter.OriginDir[1]=Usart1ReceiveData[6];
			JXS_Parameter.OriginDir[2]=Usart1ReceiveData[7];
			JXS_Parameter.OriginDir[3]=Usart1ReceiveData[8];
			JXS_Parameter.OriginDir[4]=Usart1ReceiveData[9];
			JXS_Parameter.OriginDir[5]=Usart1ReceiveData[10];
			JXS_Parameter.OriginSpeed[0]=Usart1ReceiveData[11];
			JXS_Parameter.OriginSpeed[1]=Usart1ReceiveData[12];
			JXS_Parameter.OriginSpeed[2]=Usart1ReceiveData[13];
			JXS_Parameter.OriginSpeed[3]=Usart1ReceiveData[14];
			JXS_Parameter.OriginSpeed[4]=Usart1ReceiveData[15];
			JXS_Parameter.OriginSpeed[5]=Usart1ReceiveData[16];
			JXS_Parameter.OriginPosition[0]=Usart1ReceiveData[17];
			JXS_Parameter.OriginPosition[1]=Usart1ReceiveData[18];
			JXS_Parameter.OriginPosition[2]=Usart1ReceiveData[19];
			JXS_Parameter.OriginPosition[3]=Usart1ReceiveData[20];
			JXS_Parameter.OriginPosition[4]=Usart1ReceiveData[21];
			JXS_Parameter.OriginPosition[5]=Usart1ReceiveData[22];
			JXS_Parameter.Origin[0] = Usart1ReceiveData[23];
			JXS_Parameter.Origin[1] = Usart1ReceiveData[24];
			JXS_Parameter.Origin[2] = Usart1ReceiveData[25];
			JXS_Parameter.Origin[3] = Usart1ReceiveData[26];
			JXS_Parameter.Origin[4] = Usart1ReceiveData[27];
			JXS_Parameter.Origin[5] = Usart1ReceiveData[28];
		}
		else if(Usart1ReceiveData[2] == 2||Usart1ReceiveData[2] == 3||Usart1ReceiveData[2] == 4||Usart1ReceiveData[2] == 5||Usart1ReceiveData[2] == 6||Usart1ReceiveData[2] == 7
			||Usart1ReceiveData[2] == 8||Usart1ReceiveData[2] == 9||Usart1ReceiveData[2] == 10||Usart1ReceiveData[2] == 11)	 //���ϲֲ���1
		{
			LC_ID=Usart1ReceiveData[2]-2;
			LC_Parameter_Load[LC_ID].ID								= Usart1ReceiveData[3];
			LC_Parameter_Load[LC_ID].Switch           = Usart1ReceiveData[4];
			LC_Parameter_Load[LC_ID].H_Num            = Usart1ReceiveData[5];
			LC_Parameter_Load[LC_ID].V_Num            = Usart1ReceiveData[6];	
			LC_Parameter_Load[LC_ID].Glue_Time        = (u16)(Usart1ReceiveData[7]|Usart1ReceiveData[8]<<8);
			LC_Parameter_Load[LC_ID].Diamonds_Time    = (u16)(Usart1ReceiveData[9]|Usart1ReceiveData[10]<<8);
			LC_Parameter_Load[LC_ID].HT_Hight		  = (u32)(Usart1ReceiveData[11]|Usart1ReceiveData[12]<<8|Usart1ReceiveData[13]<<16|Usart1ReceiveData[14]<<24);
			 Temp_LC_Parameter_Load[LC_ID].Switch     = LC_Parameter_Load[LC_ID].Switch;
			 Temp_LC_Parameter_Load[LC_ID].H_Num      = LC_Parameter_Load[LC_ID].H_Num;
			 Temp_LC_Parameter_Load[LC_ID].V_Num      = LC_Parameter_Load[LC_ID].V_Num;
			 Temp_LC_Parameter_Load[LC_ID].Glue_Time     = LC_Parameter_Load[LC_ID].Glue_Time;
			 Temp_LC_Parameter_Load[LC_ID].Diamonds_Time = LC_Parameter_Load[LC_ID].Diamonds_Time;			 
			 Temp_LC_Parameter_Load[LC_ID].HT_Hight      = LC_Parameter_Load[LC_ID].HT_Hight;
		}
		else if(Usart1ReceiveData[2] == 12||Usart1ReceiveData[2] == 13||Usart1ReceiveData[2] == 14||Usart1ReceiveData[2] == 15||Usart1ReceiveData[2] == 16||Usart1ReceiveData[2] == 17
			||Usart1ReceiveData[2] == 18||Usart1ReceiveData[2] == 19||Usart1ReceiveData[2] == 20||Usart1ReceiveData[2] == 21)	 //���ϲֲ���2
		{
			LC_ID=Usart1ReceiveData[2]-12;
			LC_Parameter_Load[LC_ID].LC_Hight         = (u32)( ((u32)Usart1ReceiveData[3]) |((u32)Usart1ReceiveData[4]<<8) |((u32)Usart1ReceiveData[5]<<16) |((u32)Usart1ReceiveData[6]<<24) );
			LC_Parameter_Load[LC_ID].Position1.XPoint = (u32)( ((u32)Usart1ReceiveData[7]) |((u32)Usart1ReceiveData[8]<<8) |((u32)Usart1ReceiveData[9]<<16) |((u32)Usart1ReceiveData[10]<<24) );
			LC_Parameter_Load[LC_ID].Position1.YPoint = (u32)( ((u32)Usart1ReceiveData[11]) |((u32)Usart1ReceiveData[12]<<8) |((u32)Usart1ReceiveData[13]<<16) |((u32)Usart1ReceiveData[14]<<24) );
			LC_Parameter_Load[LC_ID].Position2.XPoint = (u32)( ((u32)Usart1ReceiveData[15]) |((u32)Usart1ReceiveData[16]<<8) |((u32)Usart1ReceiveData[17]<<16) |((u32)Usart1ReceiveData[18]<<24) );
			LC_Parameter_Load[LC_ID].Position2.YPoint = (u32)( ((u32)Usart1ReceiveData[19]) |((u32)Usart1ReceiveData[20]<<8) |((u32)Usart1ReceiveData[21]<<16) |((u32)Usart1ReceiveData[22]<<24) );
			LC_Parameter_Load[LC_ID].Position3.XPoint = (u32)( ((u32)Usart1ReceiveData[23]) |((u32)Usart1ReceiveData[24]<<8) |((u32)Usart1ReceiveData[25]<<16) |((u32)Usart1ReceiveData[26]<<24) );
			LC_Parameter_Load[LC_ID].Position3.YPoint = (u32)( ((u32)Usart1ReceiveData[27]) |((u32)Usart1ReceiveData[28]<<8) |((u32)Usart1ReceiveData[29]<<16) |((u32)Usart1ReceiveData[30]<<24) );		
			 Temp_LC_Parameter_Load[LC_ID].LC_Hight      = LC_Parameter_Load[LC_ID].LC_Hight;			 	 
			 Temp_LC_Parameter_Load[LC_ID].Position1.XPoint = LC_Parameter_Load[LC_ID].Position1.XPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position1.YPoint = LC_Parameter_Load[LC_ID].Position1.YPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position2.XPoint = LC_Parameter_Load[LC_ID].Position2.XPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position2.YPoint = LC_Parameter_Load[LC_ID].Position2.YPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position3.XPoint = LC_Parameter_Load[LC_ID].Position3.XPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position3.YPoint = LC_Parameter_Load[LC_ID].Position3.YPoint;			
			//�������ϲֲ���
			Calculate_LC_Load_Parameter();
		}
		else if(Usart1ReceiveData[2] == 22)	 //��������
		{
			SC_Parameter.RW_Num = (u32)( ((u32)Usart1ReceiveData[3]) |((u32)Usart1ReceiveData[4]<<8) |((u32)Usart1ReceiveData[5]<<16) |((u32)Usart1ReceiveData[6]<<24) );
			SC_Parameter.CJ_Num = (u32)( ((u32)Usart1ReceiveData[7]) |((u32)Usart1ReceiveData[8]<<8) |((u32)Usart1ReceiveData[9]<<16) |((u32)Usart1ReceiveData[10]<<24) );
			SC_Parameter.JG_Num = (u32)( ((u32)Usart1ReceiveData[11])|((u32)Usart1ReceiveData[12]<<8)|((u32)Usart1ReceiveData[13]<<16)|((u32)Usart1ReceiveData[14]<<24) );						
			SC_Parameter.SC_Num = (u32)( ((u32)Usart1ReceiveData[15])|((u32)Usart1ReceiveData[16]<<8)|((u32)Usart1ReceiveData[17]<<16)|((u32)Usart1ReceiveData[18]<<24) );
			SC_Parameter.LJ_Num = (u32)( ((u32)Usart1ReceiveData[19])|((u32)Usart1ReceiveData[20]<<8)|((u32)Usart1ReceiveData[21]<<16)|((u32)Usart1ReceiveData[22]<<24) );
		}
		else if(Usart1ReceiveData[2] == 23)
		{	
			JXS_Parameter.GearRatio.Ratio[0]=Usart1ReceiveData[3]|Usart1ReceiveData[4]<<8;//X���ӳ��ֱ�
			JXS_Parameter.GearRatio.Ratio[1]=Usart1ReceiveData[5]|Usart1ReceiveData[6]<<8;//Z���ӳ��ֱ�  
			JXS_Parameter.GearRatio.Ratio[2]=Usart1ReceiveData[7]|Usart1ReceiveData[8]<<8;//Y���ӳ��ֱ�
			JXS_Parameter.GearRatio.Ratio[3]=Usart1ReceiveData[9]|Usart1ReceiveData[10]<<8;//O���ӳ��ֱ�
			JXS_Parameter.GearRatio.Ratio[4]=Usart1ReceiveData[11]|Usart1ReceiveData[12]<<8;//A���ӳ��ֱ�
			JXS_Parameter.GearRatio.Ratio[5]=Usart1ReceiveData[13]|Usart1ReceiveData[14]<<8;//B���ӳ��ֱ�	
		}
		else if(Usart1ReceiveData[2] == 24)
		{	
			DZJSpeed1.ManualSpeed_L[0] = Usart1ReceiveData[3];
			DZJSpeed1.ManualSpeed_H[0] = Usart1ReceiveData[4];
			DZJSpeed1.AutoSpeed_L[0]   = Usart1ReceiveData[5];
			DZJSpeed1.AutoSpeed_H[0]   = Usart1ReceiveData[6];
			DZJSpeed1.Acceleration_UP[0]   = Usart1ReceiveData[7];
			DZJSpeed1.Acceleration_DOWN[0] = Usart1ReceiveData[8];
			DZJSpeed1.ManualSpeed_L[1]     = Usart1ReceiveData[9];
			DZJSpeed1.ManualSpeed_H[1]     = Usart1ReceiveData[10];
			DZJSpeed1.AutoSpeed_L[1]       = Usart1ReceiveData[11];
			DZJSpeed1.AutoSpeed_H[1]       = Usart1ReceiveData[12];
			DZJSpeed1.Acceleration_UP[1]   = Usart1ReceiveData[13];
			DZJSpeed1.Acceleration_DOWN[1] = Usart1ReceiveData[14];
			DZJSpeed1.ManualSpeed_L[2]     = Usart1ReceiveData[15];
			DZJSpeed1.ManualSpeed_H[2]     = Usart1ReceiveData[16];
			DZJSpeed1.AutoSpeed_L[2]       = Usart1ReceiveData[17];
			DZJSpeed1.AutoSpeed_H[2]       = Usart1ReceiveData[18];
			DZJSpeed1.Acceleration_UP[2]   = Usart1ReceiveData[19];
			DZJSpeed1.Acceleration_DOWN[2] = Usart1ReceiveData[20];	
		}
	}
	else
	{}
	for(i=0;i<35;i++)
	{
		Usart1ReceiveData[i] = 0;
	}
}


/*************************************************************************
**  ��������  ReadIICPoint()
**	�����������ȡIIC�洢��
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/25
**************************************************************************/
void ReadIICPoint()
{
	u8 i=0;
	u8 Code=0;
	if((Usart1ReceiveData[1]==0xAA) && (Usart1ReceiveData[2]==101))  //�����������
	{
		Point_Syc_Ready = TRUE;	
	}
	else if((Usart1ReceiveData[1]==0x11) &&(Usart1ReceiveData[2]>0) &&(Usart1ReceiveData[3]==1))		//���ճ���,�����Ų���0����ֹIIC����ʱ����������
	{
		if(Usart1ReceiveData[12]==0)   //X��
		{
			Code = Usart1ReceiveData[2]-1;				
			Manul_Save_PointX[Code].Flag  = Usart1ReceiveData[3];
			Manul_Save_PointX[Code].Name  = (u32)( ((u32)Usart1ReceiveData[4])|((u32)Usart1ReceiveData[5]<<8)|((u32)Usart1ReceiveData[6]<<16)|((u32)Usart1ReceiveData[7]<<24) );
			Manul_Save_PointX[Code].Point = (u32)( ((u32)Usart1ReceiveData[8])|((u32)Usart1ReceiveData[9]<<8)|((u32)Usart1ReceiveData[10]<<16)|((u32)Usart1ReceiveData[11]<<24) );
		}
		else if(Usart1ReceiveData[12]==1) //Z��
		{
			Code = Usart1ReceiveData[2]-1;				
			Manul_Save_PointZ[Code].Flag  = Usart1ReceiveData[3];
			Manul_Save_PointZ[Code].Name  = (u32)( ((u32)Usart1ReceiveData[4])|((u32)Usart1ReceiveData[5]<<8)|((u32)Usart1ReceiveData[6]<<16)|((u32)Usart1ReceiveData[7]<<24) );
			Manul_Save_PointZ[Code].Point = (u32)( ((u32)Usart1ReceiveData[8])|((u32)Usart1ReceiveData[9]<<8)|((u32)Usart1ReceiveData[10]<<16)|((u32)Usart1ReceiveData[11]<<24) );
		}
		else if(Usart1ReceiveData[12]==2) //L��
		{
			Code = Usart1ReceiveData[2]-1;				
			Manul_Save_PointL[Code].Flag  = Usart1ReceiveData[3];
			Manul_Save_PointL[Code].Name  = (u32)( ((u32)Usart1ReceiveData[4])|((u32)Usart1ReceiveData[5]<<8)|((u32)Usart1ReceiveData[6]<<16)|((u32)Usart1ReceiveData[7]<<24) );
			Manul_Save_PointL[Code].Point = (u32)( ((u32)Usart1ReceiveData[8])|((u32)Usart1ReceiveData[9]<<8)|((u32)Usart1ReceiveData[10]<<16)|((u32)Usart1ReceiveData[11]<<24) );
		}
		else if(Usart1ReceiveData[12]==3) //O��
		{
			Code = Usart1ReceiveData[2]-1;				
			Manul_Save_PointO[Code].Flag  = Usart1ReceiveData[3];
			Manul_Save_PointO[Code].Name  = (u32)( ((u32)Usart1ReceiveData[4])|((u32)Usart1ReceiveData[5]<<8)|((u32)Usart1ReceiveData[6]<<16)|((u32)Usart1ReceiveData[7]<<24) );
			Manul_Save_PointO[Code].Point = (u32)( ((u32)Usart1ReceiveData[8])|((u32)Usart1ReceiveData[9]<<8)|((u32)Usart1ReceiveData[10]<<16)|((u32)Usart1ReceiveData[11]<<24) );
		}
	}
	else
	{}
	for(i=0;i<13;i++)
	{
		Usart1ReceiveData[i] = 0;
	}
}


/*************************************************************************
**  ��������  ReadIICProgramInf()
**	�����������ȡIIC����
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/25
**************************************************************************/
void ReadIICProgramInf()
{
	u8 i=0;
	u8 Code=0;
	if((Usart1ReceiveData[1]==0xAA) && (Usart1ReceiveData[2]==(SAVEPROGRAMNUM+1)))  //������Ϣ�������
	{
		ProgramInf_Syc_Ready = TRUE;	
	}
	else if((Usart1ReceiveData[1]==0x11) && (Usart1ReceiveData[2]>0))		//���ճ�����Ϣ
	{
		Code=Usart1ReceiveData[4];		//����Codeѡ��λ�ô洢,Free_Program_Save.Codeֵ����
		if(Usart1ReceiveData[3]==1)
		{
			Program_Flag[Code-1]=1;
		}
		Saved_Program[Code-1].Flag    = Usart1ReceiveData[3];	   //�����־λ
		Saved_Program[Code-1].Code    = Usart1ReceiveData[4];	   //������
		Saved_Program[Code-1].Name    = (u32)(((u32)Usart1ReceiveData[5])  | ((u32)Usart1ReceiveData[6]<<8)  | ((u32)Usart1ReceiveData[7]<<16)  | ((u32)Usart1ReceiveData[8])<<24);	   //��������
		Saved_Program[Code-1].Order_Num     = Usart1ReceiveData[9]|Usart1ReceiveData[10]<<8;	   //�����ģ
		Saved_Program[Code-1].Time = Usart1ReceiveData[11]|Usart1ReceiveData[12]<<8|Usart1ReceiveData[13]<<16|Usart1ReceiveData[14]<<24;//���򱣴��ַ
	}
	else
	{}
	for(i=0;i<14;i++)
	{
		Usart1ReceiveData[i] = 0;
	}
}
/*************************************************************************
**  ��������  ReadIICProgram()
**	�����������ȡIIC����
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/25
**************************************************************************/
void ReadIICProgram()
{
	u8 i=0;
	if((Usart1ReceiveData[1]==0xAA) && (Usart1ReceiveData[2]==2))  //����������
	{
		Program_Syc_Ready = TRUE;	
		//������ʾ���ɱ�̳���	
		for(i=Free_Program_Operate.Num;i<LARGESTPROGRAMNUM;i++)
		{
			Free_Program_Operate.Program[i].Flag   =0;
			Free_Program_Operate.Program[i].List   =0;
			Free_Program_Operate.Program[i].Order  =0;
			Free_Program_Operate.Program[i].Key    =0;
			Free_Program_Operate.Program[i].Value1 =0;
			Free_Program_Operate.Program[i].Value2 =0;
			Free_Program_Operate.Program[i].Value3 =0;	
			Free_Program_Operate.Program[i].Value4 =0;
		}
		if((Free_Program_Operate.Flag>0) && (USBH_Process_Flag==FALSE))
		{
			Current_Operate_Program.Flag = Free_Program_Operate.Flag;			
			Current_Operate_Program.Code = Free_Program_Operate.Code;
			Current_Operate_Program.Name = Free_Program_Operate.Name;
			Current_Operate_Program.Num  = Free_Program_Operate.Num;
			for(i=0;i<Current_Operate_Program.Num;i++)
			{
				Current_Operate_Program.Program[i].Flag    = Free_Program_Operate.Program[i].Flag;
				Current_Operate_Program.Program[i].List    = Free_Program_Operate.Program[i].List;
				Current_Operate_Program.Program[i].Order   = Free_Program_Operate.Program[i].Order;
				Current_Operate_Program.Program[i].Key     = Free_Program_Operate.Program[i].Key;
				Current_Operate_Program.Program[i].Value1  = Free_Program_Operate.Program[i].Value1;
				Current_Operate_Program.Program[i].Value2  = Free_Program_Operate.Program[i].Value2;
				Current_Operate_Program.Program[i].Value3  = Free_Program_Operate.Program[i].Value3;
				Current_Operate_Program.Program[i].Value4  = Free_Program_Operate.Program[i].Value4;
			}
			//���ϲ�ָ����
			Read_LC_List(g_Run_Program_Num);
			g_Current_Program_Debug=TRUE;
		}
	}
	else if((Usart1ReceiveData[1]==0x11) && (Usart1ReceiveData[2]>0))		//���ճ���,������벻��0����ֹIIC����ʱ����������
	{
		Time = Usart1ReceiveData[2];		   //��������1����ǰ��ŷ��ʹ���
		if(Time == 1)
		{ 	//������Ϣ
	        Free_Program_Operate.Flag = Usart1ReceiveData[3];	   //�����־λ
			Free_Program_Operate.Code = Usart1ReceiveData[4]; //������
			Free_Program_Operate.Name = (u32)( (u32)Usart1ReceiveData[5]|( (u32)Usart1ReceiveData[6]<<8 )|
			                                 ((u32)Usart1ReceiveData[7]<<16 )|( (u32)Usart1ReceiveData[8]<<24));	   //��������
		    Free_Program_Operate.Num = Usart1ReceiveData[9];	   //�����ģ
		}
		else 
		{   //ѡ�еĳ�������
			Free_Program_Operate.Program[Time-2].Flag  = Usart1ReceiveData[3];	  //����ָ���־λ
			Free_Program_Operate.Program[Time-2].List  = Usart1ReceiveData[4];	  //����ָ���־λ
			Free_Program_Operate.Program[Time-2].Order = Usart1ReceiveData[5];	  //����ָ������
			Free_Program_Operate.Program[Time-2].Key   = Usart1ReceiveData[6];	  //����ָ��������
			Free_Program_Operate.Program[Time-2].Value1= (u32)(((u32)Usart1ReceiveData[7])  | ((u32)Usart1ReceiveData[8]<<8)  | ((u32)Usart1ReceiveData[9]<<16)  | ((u32)Usart1ReceiveData[10])<<24);
			Free_Program_Operate.Program[Time-2].Value2= (u32)(((u32)Usart1ReceiveData[11])  | ((u32)Usart1ReceiveData[12]<<8)  | ((u32)Usart1ReceiveData[13]<<16)  | ((u32)Usart1ReceiveData[14])<<24);
			Free_Program_Operate.Program[Time-2].Value3= (u32)(((u32)Usart1ReceiveData[15])  | ((u32)Usart1ReceiveData[16]<<8)  | ((u32)Usart1ReceiveData[17]<<16)  | ((u32)Usart1ReceiveData[18])<<24);
			Free_Program_Operate.Program[Time-2].Value4= (u32)(((u32)Usart1ReceiveData[19])  | ((u32)Usart1ReceiveData[20]<<8)  | ((u32)Usart1ReceiveData[21]<<16)  | ((u32)Usart1ReceiveData[22])<<24);
		}
	}
	else
	{}
	for(i=0;i<30;i++)
	{
		Usart1ReceiveData[i] = 0;
	}
}

/*************************************************************************
**  ��������  ReadIICSoftLimit()
**	�����������ȡIIC��ȫ��
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/25
**************************************************************************/
void ReadIICSoftLimit()
{
	u8 i=0;
	if((Usart1ReceiveData[1]==0xAA) && (Usart1ReceiveData[2]==7)) //
	{
		SoftLimit_Syc_Ready = TRUE;	
	}
	else if((Usart1ReceiveData[1]==0x11) && (Usart1ReceiveData[2]>0))		//���ճ���,������벻��0����ֹIIC����ʱ����������
	{
		Code = Usart1ReceiveData[2]-1;		//��������1��������,�����Ӧ������ �±�ΪCode-1
		Robot_SoftLimit[Code].Left_Limit  = (u32)(((u32)Usart1ReceiveData[3])   | ((u32)Usart1ReceiveData[4]<<8)   | ((u32)Usart1ReceiveData[5]<<16)   | ((u32)Usart1ReceiveData[6])<<24);
		Robot_SoftLimit[Code].Right_Limit = (u32)(((u32)Usart1ReceiveData[7])   | ((u32)Usart1ReceiveData[8]<<8)   | ((u32)Usart1ReceiveData[9]<<16)   | ((u32)Usart1ReceiveData[10])<<24);
		Robot_SoftLimit[Code].Switch_Limit= Usart1ReceiveData[11];
		Temp_Soft_Limit1[Code].Left_Limit = Robot_SoftLimit[Code].Left_Limit/JXS_Parameter.GearRatio.Ratio[Code];
		Temp_Soft_Limit1[Code].Right_Limit = Robot_SoftLimit[Code].Right_Limit/JXS_Parameter.GearRatio.Ratio[Code];		
	}
	else
	{}
	for(i=0;i<12;i++)
	{
		Usart1ReceiveData[i] = 0;
	}
}

/*************************************************************************
**  ��������  ReadIICSafeArea()
**	�����������ȡIIC��ȫ��
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/25
**************************************************************************/
void ReadIICSafeArea()
{
	u8 i=0;
	if((Usart1ReceiveData[1]==0xAA) && (Usart1ReceiveData[2]==4)) //��ȫ����������-ֻ����3��
	{
		SafeArea_Syc_Ready = TRUE;	
	}
	else if((Usart1ReceiveData[1]==0x11) && (Usart1ReceiveData[2]>0))		//���ճ���,������벻��0����ֹIIC����ʱ����������
	{
		Code = Usart1ReceiveData[2]-1;		//��������1��������,�����Ӧ������ �±�ΪCode-1
		Robot_Safe_Area[Code].X_Left  = (u32)(((u32)Usart1ReceiveData[3])   | ((u32)Usart1ReceiveData[4]<<8)   | ((u32)Usart1ReceiveData[5]<<16)   | ((u32)Usart1ReceiveData[6])<<24);
		Robot_Safe_Area[Code].Z_Left  = (u32)(((u32)Usart1ReceiveData[7])   | ((u32)Usart1ReceiveData[8]<<8)   | ((u32)Usart1ReceiveData[9]<<16)   | ((u32)Usart1ReceiveData[10])<<24);
		Robot_Safe_Area[Code].X_Right = (u32)(((u32)Usart1ReceiveData[11])  | ((u32)Usart1ReceiveData[12]<<8)  | ((u32)Usart1ReceiveData[13]<<16)  | ((u32)Usart1ReceiveData[14])<<24);
		Robot_Safe_Area[Code].Z_Right = (u32)(((u32)Usart1ReceiveData[15])  | ((u32)Usart1ReceiveData[16]<<8)  | ((u32)Usart1ReceiveData[17]<<16)  | ((u32)Usart1ReceiveData[18])<<24);
		Robot_Safe_Area[Code].SafeArea_Switch = Usart1ReceiveData[19];
	}
	else
	{}
	for(i=0;i<20;i++)
	{
		Usart1ReceiveData[i] = 0;
	}
}

/*************************************************************************
**  ��������  ReadIICIODetect()
**	�����������ȡIICIO���
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    zj   
**  �������ڣ�2018/1/18
**************************************************************************/
void ReadIICIODetect()
{
	u8 i=0;
	if((Usart1ReceiveData[1]==0xAA) && (Usart1ReceiveData[2]==3)) //
	{
		IODetect_Syc_Ready = TRUE;	
	}
	else if (Usart1ReceiveData[1]==0x11)
	{
		if(Usart1ReceiveData[2] == 1)
		{
			 IO_Detect_Parameter[0]=Usart1ReceiveData[3];
		   IO_Detect_Parameter[1]=Usart1ReceiveData[4];
		   IO_Detect_Parameter[2]=Usart1ReceiveData[5];
		   IO_Detect_Parameter[3]=Usart1ReceiveData[6];
			 IO_Detect_Parameter[4]=Usart1ReceiveData[7];
		   IO_Detect_Parameter[5]=Usart1ReceiveData[8];
		   IO_Detect_Parameter[6]=Usart1ReceiveData[9];
		   IO_Detect_Parameter[7]=Usart1ReceiveData[10];
		   IO_Detect_Parameter[8]=Usart1ReceiveData[11];
		   IO_Detect_Parameter[9]=Usart1ReceiveData[12];
		   IO_Detect_Parameter[10]=Usart1ReceiveData[13];
		   IO_Detect_Parameter[11]=Usart1ReceiveData[14];
		   IO_Detect_Parameter[12]=Usart1ReceiveData[15];
		   IO_Detect_Parameter[13]=Usart1ReceiveData[16];
		   IO_Detect_Parameter[14]=Usart1ReceiveData[17];
		   IO_Detect_Parameter[15]=Usart1ReceiveData[18];
			 IO_Detect_Parameter[16]=Usart1ReceiveData[19];
		   IO_Detect_Parameter[17]=Usart1ReceiveData[20];
		}
		else if(Usart1ReceiveData[2]==2)
		{
			 IO_Detect_Parameter[18]=Usart1ReceiveData[3];
		   IO_Detect_Parameter[19]=Usart1ReceiveData[4];
			 IO_Detect_Parameter[20]=Usart1ReceiveData[5];
		   IO_Detect_Parameter[21]=Usart1ReceiveData[6];
		   IO_Detect_Parameter[22]=Usart1ReceiveData[7];
		   IO_Detect_Parameter[23]=Usart1ReceiveData[8];
		   IO_Detect_Parameter[24]=Usart1ReceiveData[9];
		   IO_Detect_Parameter[25]=Usart1ReceiveData[10];
		   IO_Detect_Parameter[26]=Usart1ReceiveData[11];
		   IO_Detect_Parameter[27]=Usart1ReceiveData[12];
		   IO_Detect_Parameter[28]=Usart1ReceiveData[13];
		   IO_Detect_Parameter[29]=Usart1ReceiveData[14];
		   IO_Detect_Parameter[30]=Usart1ReceiveData[15];
		   IO_Detect_Parameter[31]=Usart1ReceiveData[16];
		   IO_Detect_Parameter[32]=Usart1ReceiveData[17];
		   IO_Detect_Parameter[33]=Usart1ReceiveData[18];
		   IO_Detect_Parameter[34]=Usart1ReceiveData[19];
		   IO_Detect_Parameter[35]=Usart1ReceiveData[20];
		}
	}
  else
	{}

	for(i=0;i<21;i++)
	{
		Usart1ReceiveData[i] = 0;
	}
}

/*************************************************************************
**  ��������  Read_IIC()
**	�����������е��ʹ�ܼ�ԭ���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/25
**************************************************************************/
void Read_IIC()
{
	switch(Usart1ReceiveData[0]) 
	{
		case M_READ_IIC_1:			    //0xF1--�����趨����
	 		 ReadIICSetting();     
			 break;
		
		case M_READ_IIC_2:			    //0xF2--�����
	 		 ReadIICPoint();     
			 break;
		
		case M_READ_IIC_3:			    //0xF3--���������Ϣ
	 		 ReadIICProgramInf();    
			 break;

		case M_READ_IIC_4:			    //0xF4--��������
	 		 ReadIICProgram();    
			 break;
			 		
		case M_READ_IIC_5:			    //0xF5
	 		 ReadIICSoftLimit();    
			 break;
		
		case M_READ_IIC_6:			    //0xF6
	 		 ReadIICSafeArea();     
			 break;
		
//		case M_READ_IIC_7:			    //0xF7--IO������
//       ReadIICIODetect();			
//			 break;

//		case M_READ_IIC_8:			    //0xF8--�������
//	 		 ReadIICOneProgram();    
//			 break;

		default: 
		     break;	
	}
}
/*************************************************************************
**  ��������  Program_Receive()
**	����������������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    White_L
**  �������ڣ�2018/8/16
**************************************************************************/
extern u8 Program_Save_OrNot;
void Program_Receive(void)
{
	u8 temp[60];
	LCD_RealTime_Watch=FALSE;//�ر�ʵʱ���
		if(Program_Save_OrNot==3)//��������
		{
			AT24CXX_Write(CURRENT_PROGRAM_ADR1+Order_Num*ORDER_LENTH,&Usart1ReceiveData[1],ORDER_LENTH);
			AT24CXX_Write(CURRENT_PROGRAM_ADR2+Order_Num*ORDER_LENTH,&Usart1ReceiveData[1],ORDER_LENTH);
			if(Order_Num<Free_Program_Operate.Num)
			{
					temp[0] = 0xED;
					temp[1]	= 0xBB;
					temp[2] = g_Run_Program_Num;
					temp[3] = Order_Num+2;
					temp[4] = (Order_Num+2)<<8;	
					if(Order_Num<LARGESTPROGRAMNUM)
					{
						Free_Program_Operate.Program[Order_Num].Flag  = 1;
						Free_Program_Operate.Program[Order_Num].List  = Usart1ReceiveData[1]|Usart1ReceiveData[2]<<8;
						Free_Program_Operate.Program[Order_Num].Value1= (u32)( (((u32)Usart1ReceiveData[3]))|(((u32)Usart1ReceiveData[4])<<8)|(((u32)Usart1ReceiveData[5])<<16)|(((u32)Usart1ReceiveData[6])<<24) );
						Free_Program_Operate.Program[Order_Num].Value2= (u32)( (((u32)Usart1ReceiveData[7]))|(((u32)Usart1ReceiveData[8])<<8)|(((u32)Usart1ReceiveData[9])<<16)|(((u32)Usart1ReceiveData[10])<<24) );
						Free_Program_Operate.Program[Order_Num].Value3= (u32)( (((u32)Usart1ReceiveData[11]))|(((u32)Usart1ReceiveData[12])<<8)|(((u32)Usart1ReceiveData[13])<<16)|(((u32)Usart1ReceiveData[14])<<24) );
						Free_Program_Operate.Program[Order_Num].Value4= (u32)( (((u32)Usart1ReceiveData[15]))|(((u32)Usart1ReceiveData[16])<<8)|(((u32)Usart1ReceiveData[17])<<16)|(((u32)Usart1ReceiveData[18])<<24) );
						Free_Program_Operate.Program[Order_Num].Value5= (u32)( (((u32)Usart1ReceiveData[19]))|(((u32)Usart1ReceiveData[20])<<8)|(((u32)Usart1ReceiveData[21])<<16)|(((u32)Usart1ReceiveData[22])<<24) );
						Free_Program_Operate.Program[Order_Num].Value1 = (Free_Program_Operate.Program[Order_Num].Value1 & 0x0fffffff);
						Free_Program_Operate.Program[Order_Num].Value2 = (Free_Program_Operate.Program[Order_Num].Value2 & 0x0fffffff);	
						Free_Program_Operate.Program[Order_Num].Value3 = (Free_Program_Operate.Program[Order_Num].Value3 & 0x0fffffff);
						Free_Program_Operate.Program[Order_Num].Value4 = (Free_Program_Operate.Program[Order_Num].Value4 & 0x0fffffff);
						Free_Program_Operate.Program[Order_Num].Value5 = (Free_Program_Operate.Program[Order_Num].Value5 & 0x0fffffff);
						Free_Program_Operate.Program[Order_Num].LC_ID	= Usart1ReceiveData[23];
						Free_Program_Operate.Program[Order_Num].Key	  = Usart1ReceiveData[24];
					 if(1==Usart1ReceiveData[24]||2==Usart1ReceiveData[24]||3==Usart1ReceiveData[24]||5==Usart1ReceiveData[24])//Order_Type 1 �㽺 2���� 3ȡ�� 4 ��ʱ 5����
					 {
						Free_Program_Operate.Program[Order_Num].IO_Group  = Usart1ReceiveData[25]|Usart1ReceiveData[26]<<8|Usart1ReceiveData[27]<<16|Usart1ReceiveData[28]<<24;	
					 }
					else
					{
						Free_Program_Operate.Program[Order_Num].Delay_Time= Usart1ReceiveData[27]|Usart1ReceiveData[28]<<8;					 
					}
				}
				Order_Num++;
				USART2_SendDataArray(5,FREE_PROGRAM_SEND,temp);     //��ȡ��Ӧ�������
			}
			else
			{
				LCD_RealTime_Watch=TRUE;//��ʵʱ���
			}
			USART1RecieveFinishedFlag=FALSE;
		}
		else if(Program_Save_OrNot==1)//���򱣴�
		{
			AT24CXX_Read(CURRENT_PROGRAM_ADR2+ORDER_LENTH*Order_Num,&IIC_Temp[3],ORDER_LENTH);
			AT24CXX_Write(CURRENT_PROGRAM_ADR1+ORDER_LENTH*Order_Num,&IIC_Temp[3],ORDER_LENTH);
			if(Order_Num<Free_Program_Operate.Num)
			{
				IIC_Temp[0] = 0xED;		
				IIC_Temp[1] = 0xAA;					
				IIC_Temp[2] = g_Run_Program_Num;
				if(Order_Num<LARGESTPROGRAMNUM)
				{
					Free_Program_Operate.Program[Order_Num].Flag  = 1;
					Free_Program_Operate.Program[Order_Num].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
					Free_Program_Operate.Program[Order_Num].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
					Free_Program_Operate.Program[Order_Num].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
					Free_Program_Operate.Program[Order_Num].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
					Free_Program_Operate.Program[Order_Num].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
					Free_Program_Operate.Program[Order_Num].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
					Free_Program_Operate.Program[Order_Num].LC_ID	= IIC_Temp[25];	
					Free_Program_Operate.Program[Order_Num].Key   = IIC_Temp[26];
					 if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 �㽺 2���� 3ȡ�� 4 ��ʱ 5����
					 {
						Free_Program_Operate.Program[Order_Num].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
					 }
					else if(4==IIC_Temp[26])//��ʱָ��
					{
						Free_Program_Operate.Program[Order_Num].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
					}
				}
				Order_Num++;
				USART2_SendDataArray(ORDER_LENTH+3,FREE_PROGRAM_SEND,IIC_Temp);     //��ȡ��Ӧ�������
			}
			else
			{
				LCD_RealTime_Watch=TRUE;//��ʵʱ���
			}
			USART1RecieveFinishedFlag=FALSE;			
		}
}
/*************************************************************************
**  ��������  USART1RecieveDataDecode()
**	�������������1���տ��ư巵��ָ����뺯��
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/16
**************************************************************************/
 void USART1RecieveDataDecode()
{
    u8 i=0;
	for(i=0;i<USART1ReceiveDataLen-2;i++)
	{
		Usart1ReceiveData[i]=USART1ReceiveDataBuffer[i+2];	 //������ȡ��ȥ����ͷ�����ݳ���
	}																 //���õ�����+����
	for(i=0;i<60;i++)
	{
	    USART1ReceiveDataBuffer[i]=0;
	}
	USART1ReceiveDataLen = 0;

   /*----------0x0E-----��������---------------*/
	if(USART1_Current_Order == 0x0E)  
	{
		Watch_Command();
	}
   /*----------0x0F-----������ȡIIC-----------*/
	else if(USART1_Current_Order == 0x0F)  
	{
		Read_IIC();
	}
   /*----------0x0A~0x0D--��������-----------*/
	else
	{
		switch(Usart1ReceiveData[0]) //
		{
			case M_ROBOT_ENABLE_A_ORIGIN:			 //0xA0 ��е��ʹ�ܼ���ԭ��
	 		 	 Robot_Setting();			 		     
				 break;
			case M_WORK_MODE:			             //0xB0 
			     break;			 	
			case M_AUTO_RUN:			             //0xC0 
					 Auto_Run();
			     break;	
			case M_FREE_PROGRAM_RECEIVE:			 //0xE0  ///�����ȡ
					Program_Receive();
			     break;	
			case M_IO_DEBUG_OUTPUT1:			     //0xA1 
			     g_Output1Ack_Flag = TRUE;
				 break;	
			case M_IO_DEBUG_OUTPUT2:			     //0xB1 
			     g_Output1Ack_Flag = TRUE;
				 break;	
			case M_IO_DEBUG_OUTPUT3:			     //0xC1 
			     g_Output1Ack_Flag = TRUE;
				 break;	
			case M_MANUL_OPERATE:			         //0xD1 
			     Manul_Operate();		 			 
				 break;	
			case M_PARAMETER_ORDER:			         //0xE1
				g_IO_DetectAck_Flag = TRUE;				
				 break;
			default: 
			     break;			 	
		}		
	}
}



/*************************************************************************
**  ��������  USART1Acknowledge()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/16
**************************************************************************/
void USART1Acknowledge()
{
	u8 USART1ErrorFlag_1 = FALSE;
	USART1RecieveFinishedFlag = FALSE;	
  USART1RecieveCount = 1;
  /*--�ڷ������ݺ����н���Э�����֣�������ֵ��ȷʱ���Ž�����һ��������ͨ�ų���--*/
	while( (USART1RecieveFinishedFlag!=TRUE) && (USART1ErrorFlag_1 == FALSE))
	{
		if((g_USART1_SendProgram_Flag==TRUE) || (g_USART1_DeletePoint_Flag==TRUE))	//��������ص�ͨ��
		{
			if(USART1RecieveCount > 30)	 //300ms��δ���յ��ظ�����
			{
				USART1ErrorFlag_1 = TRUE;
			}
			else
			{
			    USART1ErrorFlag_1 = FALSE;
			}					
		}
		else
		{
			if(USART1RecieveCount > 5)	 //50ms��δ���յ��ظ�����
			{
				USART1ErrorFlag_1 = TRUE;
			}
			else
			{
			    USART1ErrorFlag_1 = FALSE;
			}
		}	
	}	
	USART1RecieveCount = 0;
	if(USART1RecieveFinishedFlag==TRUE)	   //�����������
	{
	   /*------------------------*/
		USART1RecieveDataDecode();		   //����Ӧ�����ݺ���
	  /*------------------------*/
	}
	//return USART1ErrorFlag_1;
}


/*************************************************************************
**  ��������  USART1_SendProgram()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/16
**************************************************************************/
void USART1_SendProgram(u32 P_DataLen, SaveProgram *P_Data)
{
	u8 i=0;
	u8 temp[18]={0};
	temp[0] =PROGRAM_START;
	USART1_SendData(1,FREE_PROGRAM_SEND,temp);     //��ʼ���ͳ���

	temp[0] =PROGRAM_INFO;
	temp[1] =P_Data->Flag;
	temp[2] =P_Data->Code;
	temp[3] =P_Data->Name;
	temp[4] =P_Data->Name>>8;
	temp[5] =P_Data->Name>>16;
	temp[6] =P_Data->Name>>24;
	temp[7] =P_Data->Num;
	USART1_SendData(8,FREE_PROGRAM_SEND,temp);  //�ȷ������������Ϣ����

	for(i=0;i<P_DataLen;i++)					//���ͱ����������Ϣ����
	{
		temp[0] =PROGRAM_CONT;
		temp[1] =P_Data->Program[i].Flag;
		temp[2] =P_Data->Program[i].List;
		temp[3] =P_Data->Program[i].Order;
		temp[4] =P_Data->Program[i].Key;
		temp[5] =P_Data->Program[i].Value1;
		temp[6] =P_Data->Program[i].Value1>>8;
		temp[7] =P_Data->Program[i].Value1>>16;
		temp[8] =P_Data->Program[i].Value1>>24;
		temp[9] =P_Data->Program[i].Value2;
		temp[10]=P_Data->Program[i].Value2>>8;
		temp[11]=P_Data->Program[i].Value2>>16;
		temp[12]=P_Data->Program[i].Value2>>24;
		temp[13]=P_Data->Program[i].Value3;
		temp[14]=P_Data->Program[i].Value3>>8;
		temp[15]=P_Data->Program[i].Value3>>16;
		temp[16]=P_Data->Program[i].Value3>>24;
		USART1_SendData(17,FREE_PROGRAM_SEND,temp);
	}
	temp[0] =PROGRAM_END;
	USART1_SendData(1,FREE_PROGRAM_SEND,temp);     //�������ͳ���
	//���س���༭����
}




/*************************************************************************
**  ��������  USART1_ReadProgram()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/16
**************************************************************************/
extern u8 Program_Save_OrNot;
void USART1_ReceiveProgram(u16 P_DataLen, u8 Code)
{
	u16 i=0,Flag=0;
	u8 temp[60]={0};
	if(Program_Save_OrNot==3)
	{
		if(P_DataLen!=0)
		{
			temp[0] = 0xED;
			temp[1] = 0xBB;
			temp[2] = Code;
			temp[3] = Order_Num+1;	
			temp[4] = (Order_Num+1)<<8;	
			USART2_SendDataArray(5,FREE_PROGRAM_SEND,temp);     //��ȡ��Ӧ�������
			USART1RecieveFinishedFlag=FALSE;
			Flag=1;
			Program_Transport_Flag=1;
			Program_Transport_Count=0;
			while(Flag)
			{
				if(Program_Transport_Count>=200)
					Flag=0;
				if(USART1RecieveFinishedFlag==TRUE)	   //�����������
				{
					Program_Transport_Count=0;
					 /*------------------------*/
					USART1RecieveDataDecode();		   //����Ӧ�����ݺ���
					/*------------------------*/
					USART1RecieveFinishedFlag=FALSE;
				}		
				if(Order_Num==P_DataLen)
				{
					Program_Transport_Count=0;
					Program_Transport_Flag=0;
					Flag=0;
				}
			}
			Program_Transport_Count=0;
			Program_Transport_Flag=0;		
			Program_Transport_Flag=0;
			Program_Save_OrNot=0;
			if(Order_Num<LARGESTPROGRAMNUM)
			{
				for(i=Order_Num;i<LARGESTPROGRAMNUM;i++)
				{
					Free_Program_Operate.Program[i].Flag  = 1;
					Free_Program_Operate.Program[i].List  = 0;
					Free_Program_Operate.Program[i].Key   = 0;//X��
					Free_Program_Operate.Program[i].Value1= 0;
					Free_Program_Operate.Program[i].Value2= 0;
					Free_Program_Operate.Program[i].Value3= 0;	
					Free_Program_Operate.Program[i].Value4= 0;
					Free_Program_Operate.Program[i].LC_ID	= 0;				
				}
			}
		}
		else
		{
			 Order_Num = 0;
			for(i=0;i<LARGESTPROGRAMNUM;i++)
			{
				Free_Program_Operate.Program[i].Flag  = 0;
				Free_Program_Operate.Program[i].List  = 0;
				Free_Program_Operate.Program[i].Key   = 0;//X��
				Free_Program_Operate.Program[i].Value1= 0;
				Free_Program_Operate.Program[i].Value2= 0;
				Free_Program_Operate.Program[i].Value3= 0;	
				Free_Program_Operate.Program[i].Value4= 0;
				Free_Program_Operate.Program[i].LC_ID	= 0;				
			}
		}
	}
	//���س���༭����
}


/*************************************************************************
**  ��������  USART1_ReadProgram()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2017/1/10
**************************************************************************/
u8 USART1_ReadProgram(u8 Key)
{
    u8 data[2]={0};
	u8 result=1;
	Free_Program_Operate.Flag = 0;			
	Free_Program_Operate.Code = 0;
	Free_Program_Operate.Name = 0;
	Free_Program_Operate.Num  = 0;
    data[0] = READ_IIC_4;
	data[1] = Key;	
	Read_IIC_Done=TRUE;
	Read_IIC_Flag = FALSE;
	Program_Syc_Ready=FALSE;
	USART1RecieveFinishedFlag=FALSE;			    
	while(!Program_Syc_Ready)
	{
		while(USART1RecieveFinishedFlag!=TRUE)
		{
			if(Read_IIC_Done)
			{
				result++;
				USART2_SendDataArray(2,READ_IIC,data);
				USART1RecieveFinishedFlag = FALSE;
				Read_IIC_Done=FALSE;
				Read_IIC_Flag = TRUE;
			}
		}
		USART1RecieveDataDecode();   //����1����Ӧ������ 
		USART1RecieveFinishedFlag = FALSE;
		if(result>120)
		{
		   result=120;
		   break;
		}
	}
	if(Program_Syc_Ready)
	{
		Read_IIC_Done = TRUE;
		Read_IIC_Flag = FALSE;
		Read_IIC_Count = 0;
		result=0;
	}
	return result;
}

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team ******** END OF FILE ************************/
