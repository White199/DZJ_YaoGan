/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Auto.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "Usart.h" 
#include "StatusControl.h"
#include "SpeedControl.h"
#include "Auto.h"
#include "Manual.h"
#include "out.h"
#include "in.h"
#include "Error.h"
#include "Parameter.h"
#include "SignalWatch.h"
#include "Auto_2.h"
#include "PDO.h"

#include "SpeedControl_CAN.h"

//u8 Servo_Is_Run = FALSE;		   //�ŷ����Ƿ�������ת��־λ
u8 Synchronize = FALSE;			   //ͬ����־λ
u8 Initialize_Finished = FALSE;	   //��ʼ����ɱ�־λ
u8 Origin_Backed = FALSE;		   //�ﵽԭ���־λ
u8 X_Origin_Backed = FALSE;		   //X��ﵽԭ���־λ
u8 Z_Origin_Backed = FALSE;		   //Z��ﵽԭ���־λ
u8 L_Origin_Backed = FALSE;		   //L��ﵽԭ���־λ
u8 O_Origin_Backed = FALSE;		   //O��ﵽԭ���־λ
u8 Back_Origin_Flag = FALSE;	   //��ԭ���־λ
u8 Origin_Speed_Down=FALSE;        //��ԭ���⵽�źź�״̬Ϊ����״̬��־λ
u32 Back_Origin_Speed = 50;	       //��ԭ���ٶ�
u8 Robot_Enable_Flag = FALSE;	   //��е��ʹ��
//u8 Origin_Type = Z_X_TYPE;		   //��ԭ�㷽ʽ

u8 Work_Status = AUTO_WORK_MODE;        //��е�ֹ���ģʽ 
u8 Axis_Manul_BackToOrigin=FALSE;  //�ֶ�����
u8 Axsis_Chosen = X_Axsis;		   //�˶���ѡ��
u8 Axsis_Move_Direction[6] = {POSITIVE,POSITIVE,POSITIVE,POSITIVE,POSITIVE,POSITIVE};//�˶�����ѡ��			  
u32 Input_Detect_Time = 0;		   //�����ⶨʱ����
u8 Action_Detect_Flag=FALSE;	   //��������
u8 Input_Detect_Enable = ENABLE;   //������ʹ�ܱ�־����һ��ʹ��
u8 Jog_Pause_Enable=DISABLE;       //�綯������ͣ
u8 Position_Handle_Flag=FALSE;     //�Ƿ����λ�ñ�־
u8 SafeArea_Is_On=FALSE;
u8 X_In_SafeArea_Flag=FALSE;
u8 Safe_Area_Flag=FALSE;		   //��ȫ�����жϣ���ֹ��ǰ����������־λ���ֿ����������ѽ���
u8 Safe_Area_Alarm=FALSE;		   //��ȫ���򱨾���������ȫ��
u8 g_Current_SafeArea_Num=0;	   //��ȫ�����
u8 Input_Count15 = 0;			   //����15����
u8 Input_Count16 = 0;			   //����16����
u8 Input15_16_Detect=DISABLE;	   //����������ֹͣ�жϱ�־λ
u8 Servo_Stop_Done[6]={FALSE,FALSE,FALSE,FALSE,FALSE,FALSE};		   //�ŷ���ֹͣ�����Ƿ�ִ�б�־
u8 Test_ServoStop=0;
u8 g_Auto_Reset_Flag = FALSE;	   //�Զ���λ��־λ
u8 Robot_Auto_Reset = FALSE;
u8 g_Auto_LOrigin_Flag=FALSE;	   //L�����
u32 g_USART_Delay_Timer=0;		   //������ʱ����-��ʽͨ��ʧ��
static u8 gs_AutoStatue=4;
static u8 gs_Error_Status = NO_ERROR;

//------------------------------------------//

/**************************************************************************************************
**  ��������  WorkMode()
**	�����������
**	�����������
**	�������ܣ�����ģʽѡ��
**	��ע��	  ��
**  ���ߣ�         
**  �������ڣ� 
***************************************************************************************************/
void WorkMode()
{
	switch(UsartReceiveData[1])
	{
		case P_AUTO_MODE:
			 Work_Status = AUTO_WORK_MODE;
			 if(g_Robot_Has_Debuged_Flag)
			 {
				 g_AutoStatue = AUTO_WAITE;
				 g_Auto_Order_Start = FALSE;
				 g_Auto_Order_Pause = FALSE;
				 g_Auto_Order_Stop = FALSE;
				 Single_Mode_Enable = DISABLE;
				 g_Auto_PresentLine = 0;	  //��ǰ��������
				 g_Auto_ActionRun_Step=0;
				 Robot_Auto_Reset=FALSE;		
				 g_Robot_Has_Debuged_Flag=FALSE;	 
			 }
		   break;	

		case P_FREE_PROGRAM:
			 Work_Status = FREE_PROGRAM_MODE;     
			 break;	

		case P_IO_MODE:
			 Work_Status = IO_DEBUG_MODE;
		   break;	

		case P_MANUL_MODE:
			 Work_Status = MANUAL_WORK_MODE;
			 Axsis_Chosen = X_Axsis;
			 Axsis_Move_Direction[X_Axsis] = POSITIVE;
			 Linked_Mode_Enable = ENABLE;
			 Jog_Mode_Enable = DISABLE;
		   Jog_Pause_Enable=DISABLE;
		   break;

	    case WAIT_MODE:
			 Work_Status = WAIT_MODE;
	 		 break;

		default:
		   break;
	}
}

/**************************************************************************************************
**  ��������  DirectionJudge()
**	�����������
**	�����������
**	�������ܣ���е�ֶ�������
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void DirectionJudge(u8 Axis)
{
   switch(Axis)
   {
       case X_Axsis:     	 
  	      if(Axsis_Move_Direction[X_Axsis] == POSITIVE)
          {
		        SetOutput(X_AXSIS_DIRECTION);
	        }
	        else if(Axsis_Move_Direction[X_Axsis] == NEGATIVE)
	        {
		        ResetOutput(X_AXSIS_DIRECTION);
	        }
			    break;
					
   	   /*- Z�������˶�Ϊ���������˶�Ϊ���� -*/
       case Z_Axsis:  
	   	    if(Axsis_Move_Direction[Z_Axsis] == POSITIVE)
	        {
			     ResetOutput(Z_AXSIS_DIRECTION);
		      }
		      else if(Axsis_Move_Direction[Z_Axsis] == NEGATIVE)
		      {			    
				   SetOutput(Z_AXSIS_DIRECTION);
		      }
			    break;

       case L_Axsis:     	 
  	      if(Axsis_Move_Direction[L_Axsis] == POSITIVE)
          {
		        SetOutput(L_AXSIS_DIRECTION);
	        }
	        else if(Axsis_Move_Direction[L_Axsis] == NEGATIVE)
	        {
		        ResetOutput(L_AXSIS_DIRECTION);
	        }
			    break;
					
       case O_Axsis:     	 
  	      if(Axsis_Move_Direction[O_Axsis] == POSITIVE)
          {
		        SetOutput(O_AXSIS_DIRECTION);
	        }
	        else if(Axsis_Move_Direction[O_Axsis] == NEGATIVE)
	        {
		        ResetOutput(O_AXSIS_DIRECTION);
	        }
			    break;
					
	     default: 
	        break;
   }
}

/**************************************************************************************************
**  ��������  SafeAreaDetection()
**	�����������
**	�����������
**	�������ܣ���е���˶���ȫ������
**	��ע��	  
**			  
**  ���ߣ�    ����     
**  �������ڣ�2013/2/10 
***************************************************************************************************/
void SafeAreaDetection()
{
	u8 Num=0;
	if(SafeArea_Is_On)
	{
		//���Z��λ��0��,�����е���X��,����Ϊ���԰�ȫ
		if((Pulse_TotalCounter[1]==MINROBOTPOSITION) && (Axsis_Chosen==X_Axsis))
		{
			Safe_Area_Flag=FALSE;
		}
		else
		{
			if(g_Current_SafeArea_Num)
			{
			 	Num=g_Current_SafeArea_Num-1;
				//X���ڰ�ȫ����Num֮��ʱ
				if((Pulse_TotalCounter[0]>=Robot_Safe_Area[Num].X_Left)&&(Pulse_TotalCounter[0]<=Robot_Safe_Area[Num].X_Right))  	   
			 	{
					   /*- ��ȫ�����־λ�ж� -*/
					   if((Pulse_TotalCounter[0]==Robot_Safe_Area[Num].X_Left)&&(Axsis_Chosen==X_Axsis)&&(Axsis_Move_Direction[X_Axsis]==NEGATIVE))
					   {
					   		Safe_Area_Flag=TRUE;
					   }
					   if((Pulse_TotalCounter[0]==Robot_Safe_Area[Num].X_Right)&&(Axsis_Chosen==X_Axsis)&&(Axsis_Move_Direction[X_Axsis]==POSITIVE))
					   {
					   		Safe_Area_Flag=TRUE;
					   }
					   if((Pulse_TotalCounter[1]==Robot_Safe_Area[Num].Z_Down)&&(Axsis_Chosen==Z_Axsis)&&(Axsis_Move_Direction[Z_Axsis]==POSITIVE))
					   {
					   		Safe_Area_Flag=TRUE;
					   }
					   /*- ��ȫ�����־λ�ж� -*/
					   if(Pulse_TotalCounter[1]<Robot_Safe_Area[Num].Z_Up)	 //Z��С���ϲ�����Z�ϰ�ȫλ��
					   {
							 if((Linked_Mode_Enable==ENABLE) && (Axsis_Chosen==Z_Axsis) && (Axsis_Move_Direction[Z_Axsis]==POSITIVE))
							 {							  
								   Linked_Pulse=Robot_Safe_Area[Num].Z_Down-Pulse_TotalCounter[1];
							 }   
					   }
					   else if(Pulse_TotalCounter[1]>Robot_Safe_Area[Num].Z_Down) //Z������ϲ�����Z�°�ȫλ��
					   {
						   if(Axsis_Chosen==Z_Axsis)								   
						   {
							   if(Axsis_Move_Direction[Z_Axsis] == POSITIVE)	   //Z�ᣬ��������˶����򲻶���
							   {
								   Safe_Area_Flag=TRUE;
							   }
						   }
						   else if(Axsis_Chosen==X_Axsis)			   //X�ᣬ������
						   {
							   Safe_Area_Flag=TRUE;
						   }
						   else
						   {
						   }
					   }
					   else	 //Z�ڰ�ȫ������  XZ�����ڰ�ȫ������,���г̽����޶�
					   {
							if(Jog_Mode_Enable == ENABLE )	   //�綯ģʽ
							{
								 if(Axsis_Chosen == X_Axsis)
								 {
									  //��ֹ�綯������X��ȫ����
										if(((Pulse_TotalCounter[0]-Jog_Pulse_Count)<=Robot_Safe_Area[Num].X_Left)&&(Axsis_Move_Direction[X_Axsis]==NEGATIVE))
										{
											 Jog_Pulse_Count=Pulse_TotalCounter[0]-Robot_Safe_Area[Num].X_Left;
										}
									  //��ֹ�綯������X��ȫ����
										if(((Pulse_TotalCounter[0]+Jog_Pulse_Count)>=Robot_Safe_Area[Num].X_Right)&&(Axsis_Move_Direction[X_Axsis]==POSITIVE))
										{
											 Jog_Pulse_Count=Robot_Safe_Area[Num].X_Right-Pulse_TotalCounter[0];
										}
								 }
								 else if(Axsis_Chosen == Z_Axsis)
								 {								
										if(((Pulse_TotalCounter[1]+Jog_Pulse_Count)>=Robot_Safe_Area[Num].Z_Down)&&(Axsis_Move_Direction[Z_Axsis]==POSITIVE))
										{
											 Jog_Pulse_Count=Robot_Safe_Area[Num].Z_Down-Pulse_TotalCounter[1];
										}
								 }
								 else
								 {
								 }						 	
							}
							else if(Linked_Mode_Enable == ENABLE )	//����ģʽ
							{
								 if(Axsis_Chosen == X_Axsis)
								 {							  
									   //��ֹ��������X��ȫ����
									   if(Axsis_Move_Direction[X_Axsis] == NEGATIVE)
								       {
										     Linked_Pulse=Pulse_TotalCounter[0]-Robot_Safe_Area[Num].X_Left;
								       }
									   //��ֹ��������X�Ұ�ȫ����
									   else if(Axsis_Move_Direction[X_Axsis] == POSITIVE)
									   {
									   		 Linked_Pulse=Robot_Safe_Area[Num].X_Right-Pulse_TotalCounter[0];
									   }
									   else
									   {
									   }
								 }
								 else if(Axsis_Chosen == Z_Axsis)
								 {							  
									   if(Axsis_Move_Direction[Z_Axsis] == POSITIVE)
									   {
									   		Linked_Pulse=Robot_Safe_Area[Num].Z_Down-Pulse_TotalCounter[1];
									   }
								 }						 					
							}
							else
							{
							}
					   }//else	Z�ڰ�ȫ������
				}//X���ڰ�ȫ����Num֮��ʱ 	 	
			}//if(g_Current_SafeArea_Num)
			//��X�᲻���κΰ�ȫ����ʱ,Z���ֹ�����ƶ� 
			else
			{
			   if(Axsis_Chosen==Z_Axsis)
			   {
				   Safe_Area_Flag=TRUE;
			   }	
			}
		}	//if(Pulse_TotalCounter[1]==MINROBOTPOSITION)
	}//if(SafeArea_Is_On)
	/*--  ��������֮��ķ����� --*/
	if(Pulse_TotalCounter[0]<=X_Axsis_Minlength && Axsis_Chosen==X_Axsis && Axsis_Move_Direction[X_Axsis]==NEGATIVE)
	{
		Safe_Area_Flag=TRUE;
	}
	if(Pulse_TotalCounter[0]>=X_Axsis_Maxlength && Axsis_Chosen==X_Axsis && Axsis_Move_Direction[X_Axsis]==POSITIVE)
	{
		Safe_Area_Flag=TRUE;
	}
	if(Pulse_TotalCounter[1]<=Z_Axsis_Minlength && Axsis_Chosen==Z_Axsis && Axsis_Move_Direction[Z_Axsis]==NEGATIVE)
	{
		Safe_Area_Flag=TRUE;
	}
	if(Pulse_TotalCounter[1]>=Z_Axsis_Maxlength && Axsis_Chosen==Z_Axsis && Axsis_Move_Direction[Z_Axsis]==POSITIVE)
	{		
		Safe_Area_Flag=TRUE;
	}
	/*--  �ڱ߽��ֹ����,���ͱ��� --*/
	if(Safe_Area_Flag)
	{
		Axsis_Move_Direction[X_Axsis]=NONE;
		Axsis_Move_Direction[Z_Axsis]=NONE;
		Axsis_Move_Direction[L_Axsis]=NONE;
		Axsis_Move_Direction[O_Axsis]=NONE;
		Axsis_Move_Direction[A_Axsis]=NONE;
		Axsis_Move_Direction[B_Axsis]=NONE;
		Linked_Move_Enable=DISABLE;
		//Jog_Move_Enable=DISABLE;
	}
}
void Halt_To_Reset(void)
{
//�ж��ŷ���λ�õ����ֹͣ
	if(Timer_Enable_X )
	{		     
		 if(Axsis_Stop_Flag[X_Axsis])//--�ж�X���Ƿ��Ѿ�ͣ����
		 {
				if(Set_Halt_Flag[X_Axsis])//�Ƿ�����ͣ
				{
						Reset_Halt(X_Axsis);
						Set_Halt_Flag[X_Axsis]=0;
//						Timer_Enable_X = DISABLE;
				}
				else if(Target_Rreached[X_Axsis])//�Ƿ��� Ŀ�굽��
				{
//						Timer_Enable_X = DISABLE;
				}	
				Axsis_Stop_Flag[X_Axsis]=0;
		  }
	 }
if(Timer_Enable_Z )	//�ٴ�����
	{		     
		 if(Axsis_Stop_Flag[Z_Axsis])//--�ж�Z���Ƿ��Ѿ�ͣ����
		 {
				if(Set_Halt_Flag[Z_Axsis])//  �Ƿ�����ͣ
				{
						Reset_Halt(Z_Axsis);
						Set_Halt_Flag[Z_Axsis]=0;
//						Timer_Enable_Z = DISABLE;
				}
				else if(Target_Rreached[Z_Axsis])//�Ƿ��� Ŀ�굽��
				{
						Timer_Enable_Z = DISABLE;
				}	
				Axsis_Stop_Flag[Z_Axsis]=0;
		  }
	 }
		if(Timer_Enable_L )
	{		     
			 if(Axsis_Stop_Flag[L_Axsis])//--�ж�L���Ƿ��Ѿ�ͣ����
			 {
					if(Set_Halt_Flag[L_Axsis])//  �Ƿ�����ͣ
					{
							Reset_Halt(L_Axsis);
							Set_Halt_Flag[L_Axsis]=0;
//							Timer_Enable_L = DISABLE;
					}
					else if(Target_Rreached[L_Axsis])//�Ƿ��� Ŀ�굽��
					{
							Timer_Enable_L = DISABLE;
					}	
					Axsis_Stop_Flag[L_Axsis]=0;
			 }
		 }
			if(Timer_Enable_O )
			{		     
				 if(Axsis_Stop_Flag[O_Axsis])//--�ж�O���Ƿ��Ѿ�ͣ����
				 {
						if(Set_Halt_Flag[O_Axsis])//  �Ƿ�����ͣ
						{
								Reset_Halt(O_Axsis);
								Set_Halt_Flag[O_Axsis]=0;
//								Timer_Enable_O = DISABLE;
						}
						else if(Target_Rreached[O_Axsis])//�Ƿ��� Ŀ�굽��
						{
								Timer_Enable_O = DISABLE;
						}	
						Axsis_Stop_Flag[O_Axsis]=0;
				 }
		  }
		if(Timer_Enable_A )
	{		     
			 if(Axsis_Stop_Flag[A_Axsis])//--�ж�L���Ƿ��Ѿ�ͣ����
			 {
					if(Set_Halt_Flag[A_Axsis])//  �Ƿ�����ͣ
					{
							Reset_Halt(A_Axsis);
							Set_Halt_Flag[A_Axsis]=0;
//							Timer_Enable_A = DISABLE;
					}
					else if(Target_Rreached[A_Axsis])//�Ƿ��� Ŀ�굽��
					{
							Timer_Enable_A = DISABLE;
					}	
					Axsis_Stop_Flag[A_Axsis]=0;
			 }
		 }
			if(Timer_Enable_B )
			{		     
				 if(Axsis_Stop_Flag[B_Axsis])//--�ж�O���Ƿ��Ѿ�ͣ����
				 {
						if(Set_Halt_Flag[B_Axsis])//  �Ƿ�����ͣ
						{
							Reset_Halt(B_Axsis);
							Set_Halt_Flag[B_Axsis]=0;
//							Timer_Enable_B = DISABLE;
						}
						else if(Target_Rreached[B_Axsis])//�Ƿ��� Ŀ�굽��
						{
								Timer_Enable_B = DISABLE;
						}	
						Axsis_Stop_Flag[B_Axsis]=0;
				 }
		  }
}
/**************************************************************************************************
**  ��������  ActionControl()
**	�����������
**	�����������
**	�������ܣ���е�ֶ�������
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void ActionControl()
{
	if(Error_Status != ERROR_EMERG_HAPPEN)
	{
//		Halt_To_Reset();
		/**--------------------�Զ�ģʽ-------------------------**/ 
		 if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE && Origin_Backed==TRUE)
		 {
			  Servo_Control_Mode=AUTORUNNINGMODE;
			  if(g_Auto_Reset_Flag)
			  {
			  	 RobotAutoReset();				 				
			  }
			  if(Robot_Auto_Reset)
			  {
			  	 AutoRunning();	
			  }			
		 }
		/**--------------------�ֶ�ģʽ-------------------------**/
		 else if(Work_Status == MANUAL_WORK_MODE && Back_Origin_Flag == FALSE && Origin_Backed==TRUE )
		 {	    
			   /*******����ģʽ*********/
			   if(Linked_Mode_Enable == ENABLE )
			  {
				  ManualLinkedRunning();
			  }	
		 }
		 else  //---�����Զ����ֶ�ģʽ0925
		 {}			 
		}						 			 
}	
/**************************************************************************************************
**  ��������  StatusControl()
**	�����������
**	�����������
**	�������ܣ�����״̬���
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void StatusControl()
{
    u8 i=0,data_temp[5];
	if(Input_Detect_Enable == ENABLE)
	{
		for(i=1;i<22;i++)
		{		
		   ReadInput(i);
		}
		Input_Detect_Enable = DISABLE;
		
		/*- �Զ�����ģʽ�£�������������ֹͣ������� -*/    
		    if(Input_Detect_Status[1] & 0x40)	 //���� X15
		    {
					Input_Detect_Status[1]&=0xbf;
				 if(Work_Status == AUTO_WORK_MODE && Origin_Backed)
				 {				 			
					  data_temp[0]=0x01;
						USART1_SendData(1,P_AUTO_RUN_LCD,data_temp);						
				 }
		    }
		    if(Input_Detect_Status[1] & 0x20)	 //��ͣ	X16
		    {
					Input_Detect_Status[1]&=0xdf;
					if(Work_Status == AUTO_WORK_MODE && Origin_Backed)
				 {
					  data_temp[0]=0x02;
						USART1_SendData(1,P_AUTO_RUN_LCD,data_temp);	
				 }
		    }
				if(Input_Detect_Status[1] & 0x10) 
				{
					 Input_Detect_Status[1]&=0xef;
					if(Origin_Backed&&(g_AutoStatue==AUTO_PAUSE))//�Ѿ�������һ���ֹͣ����λ
					{
					  data_temp[0]=0x03;
						USART1_SendData(1,P_AUTO_RUN_LCD,data_temp);//ֹͣ
					}	
					if(g_AutoStatue==AUTO_WAITE)
					{
						data_temp[0]=0x04;
						USART1_SendData(1,P_AUTO_RUN_LCD,data_temp);//��λ						
					}
				}
		//��ɫ�ƿ���
	   if(g_AutoStatue != gs_AutoStatue)
	   {
	       gs_AutoStatue=g_AutoStatue;
		   switch(g_AutoStatue)
		   {
		       case AUTO_RUNNING:
				      SetOutput(O_IODEBUG_OUTPUT_37);	   //�Ƶ�
					    SetOutput(O_IODEBUG_OUTPUT_38);	   //���
					    SetOutput(O_IODEBUG_OUTPUT_39);	   //������
              ResetOutput(O_IODEBUG_OUTPUT_36);  //�̵�
			        break;

		       case AUTO_PAUSE:
				      SetOutput(O_IODEBUG_OUTPUT_36);
					    SetOutput(O_IODEBUG_OUTPUT_38);
					    SetOutput(O_IODEBUG_OUTPUT_39);	  //������
				      ResetOutput(O_IODEBUG_OUTPUT_37); //�Ƶ�
			        break;

		       case AUTO_WAITE:
				      SetOutput(O_IODEBUG_OUTPUT_36);
					    SetOutput(O_IODEBUG_OUTPUT_38);
					    SetOutput(O_IODEBUG_OUTPUT_39);	  //������
				      ResetOutput(O_IODEBUG_OUTPUT_37); //�Ƶ�
			        break;

		       case AUTO_ERROR:
				      SetOutput(O_IODEBUG_OUTPUT_36);
					    SetOutput(O_IODEBUG_OUTPUT_37);					
					    ResetOutput(O_IODEBUG_OUTPUT_38); //���
					    ResetOutput(O_IODEBUG_OUTPUT_39); //������
			        break;

		       default:
			       break;
		   }
	   }
	   if(Error_Status != gs_Error_Status)
	   {
	     gs_Error_Status=Error_Status;
		   if(Error_Status==ERROR_EMERG_HAPPEN)
		   {		   
			   SetOutput(O_IODEBUG_OUTPUT_36);
			   SetOutput(O_IODEBUG_OUTPUT_37);					
			   ResetOutput(O_IODEBUG_OUTPUT_38); //���
			   ResetOutput(O_IODEBUG_OUTPUT_39); //������	 
		   }
		   else 
		   {	     	   
			   SetOutput(O_IODEBUG_OUTPUT_36);
			   SetOutput(O_IODEBUG_OUTPUT_38);
			   SetOutput(O_IODEBUG_OUTPUT_39);	  //������
			   ResetOutput(O_IODEBUG_OUTPUT_37); //�Ƶ�
		   }
	   }
	}
}

/**************************************************************************************************
**  ��������  Robot_Reset()
**	�����������
**	�����������
**	�������ܣ�������ز�����λ����ʼֵ
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void Robot_Reset()
{
	Error_Status = 	NO_ERROR;
	Error_Data = 0;
	Robot_Error_Data[0]=0;
	Pulse_TotalCounter[0] = MINROBOTPOSITION;	
	Pulse_TotalCounter[1] = MINROBOTPOSITION;
	Pulse_TotalCounter[2] = MINROBOTPOSITION;
	Pulse_TotalCounter[3] = MINROBOTPOSITION;
	Pulse_TotalCounter[4] = MINROBOTPOSITION;
	Pulse_TotalCounter[5] = MINROBOTPOSITION;
	Pulse_Maxspeed_HalfPeriod[Z_Axsis] = 180;
	Pulse_Maxspeed_HalfPeriod[X_Axsis] = 180;
	Pulse_Maxspeed_HalfPeriod[L_Axsis] = 180;
	Pulse_Maxspeed_HalfPeriod[O_Axsis] = 180;		
	Pulse_Maxspeed_HalfPeriod[A_Axsis] = 180;
	Pulse_Maxspeed_HalfPeriod[B_Axsis] = 180;
	Axsis_Move_Direction[X_Axsis] = POSITIVE;
	Axsis_Move_Direction[Z_Axsis] = POSITIVE;
	Axsis_Move_Direction[L_Axsis] = POSITIVE;
	Axsis_Move_Direction[O_Axsis] = POSITIVE;
	Axsis_Move_Direction[A_Axsis] = POSITIVE;
	Axsis_Move_Direction[B_Axsis] = POSITIVE;
	Axsis_Chosen = X_Axsis;
	Work_Status = AUTO_WORK_MODE;
	Servo_Control_Mode = LINKEDRUNNINGMODE;
	Pulse_PresentAcceleration[Z_Axsis]=20;
	Pulse_PresentAcceleration[X_Axsis]=20;
	Pulse_PresentAcceleration[L_Axsis]=20;
	Pulse_PresentAcceleration[O_Axsis]=20;
	Pulse_PresentAcceleration[A_Axsis]=20;
	Pulse_PresentAcceleration[B_Axsis]=20;
	Action_Delay_Flag=FALSE;
	Action_Done_Flag=FALSE;
	Pulse_ControlStep[Z_Axsis]=SPEED_UP;
	Pulse_ControlStep[X_Axsis]=SPEED_UP;
	Pulse_ControlStep[L_Axsis]=SPEED_UP;
	Pulse_ControlStep[O_Axsis]=SPEED_UP;
	Pulse_ControlStep[A_Axsis]=SPEED_UP;
	Pulse_ControlStep[B_Axsis]=SPEED_UP;
	DIR485_L;	   //485Ĭ�϶�״̬
	SetOutput(O_IODEBUG_OUTPUT_36);
	SetOutput(O_IODEBUG_OUTPUT_38);
	SetOutput(O_IODEBUG_OUTPUT_39);	  //������
	ResetOutput(O_IODEBUG_OUTPUT_37); //�Ƶ�
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
