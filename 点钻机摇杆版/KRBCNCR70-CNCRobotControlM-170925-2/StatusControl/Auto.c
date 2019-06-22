/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Auto.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "Usart.h" 
#include "Auto.h"
#include "StatusControl.h"
#include "SpeedControl.h"
#include "in.h"
#include "out.h"
#include "Manual.h"
#include "Parameter.h"
#include "Error.h"
#include "SignalWatch.h"
#include "Delay.h"
#include "w25qxx.h" 
#include "Auto_2.h"
#include "ActionOperate.h"

#include "SpeedControl_CAN.h"
#include "PDO.h"

u32 Auto_Pulse_Count = 0;        //�Զ�ģʽÿ�����������������

/**-- �Զ�ģʽ�µĸ�ģʽ��־λ --**/
u8 Auto_Mode = 	LOOP_MODE;	     //�Զ�ģʽѡ��
u8 Once_Mode_Enable = DISABLE;	 //����ģʽʹ��
u8 Single_Mode_Enable = DISABLE; //����ģʽʹ��
u8 Loop_Mode_Enable = DISABLE;	 //ѭ��ģʽʹ��

u8 Action_Complete = FALSE;      //105��е��һ��ѭ���������
u8 Section_Action_Complete=FALSE;//���ڶ����Ƿ���ɱ�־λ
u8 Auto_Run_Enable = DISABLE;    //�Զ�ģʽʹ�ܱ�־

u32 Total_Product_Number = 0;	 //�ܼƼӹ�����
u32 Product_Number = 0 ;	 	 //��Ʒ����
u32 Check_Number = 0;			 //��Ʒ�������
u8  Unload_Material_Finish=FALSE; //ж�Ͻ���

u8 Stop_Status = RUN ;			 //ֹͣ��ť����ȥ֮��Ļ�е��״̬
u8 Stop_Time = 0;				 //����ֹͣ����
u8 JHRobot_Run_Time = 0;	     //��е���Զ����ж���ѭ������

u32 JHRobot_Section_Time=0;
u8 JHRobot_Section = 1 ;	     //��е������һ��������������һ�ڣ���ʱ��Ϊһ��ѭ��10�ڣ�
u8 Last_JHRobot_Section=0;
u8 JHRobot_Section_Delay=FALSE;
u8 JHRobot_Warning=FALSE;		 //����ȫ�̼��ӱ���
u8 Unload_Material_Step=1;       //���ϵ�Ķ�������
u8 Section_Step = 1;
u8 Step_Done=FALSE;				 //�����Ƿ������־λ������ÿһ�����X.Z����
u8 Delay_Done=FALSE;			 //��ʱ�Ƿ������־λ�������ŷ���������ʱ
u8 Manchine_Door_Flag=FALSE;     //�����Ƿ�����־λ
u8 Robot_Move_Flag=FALSE;		 //��е���Ƿ���Ҫ�ƶ����ܵ�Ŀ��λ�ñ�־λ


u32	X_Axsis_Target = MINROBOTPOSITION;	 //X��Ŀ������
u32 Z_Axsis_Target = MINROBOTPOSITION;	 //Z��Ŀ������
u32 Present_Position = MINROBOTPOSITION;	 //��ǰλ��ֵ
u32 Max_Robot_Position=MAXROBOTPOSITION;	 //��е������������
u32 Min_Robot_Position=MINROBOTPOSITION;	 //��е����С��������
u32 Action_Delay_Time=0;         //��е��ÿ�ζ������֮�����ʱ����Ϊ�ŷ��������Ǹ��涯��
u8 	Puls_Delay_Time[50]={0};		  //�����ź���ʱ
u8  Puls_Delay_Enable[50]={DISABLE};  //�����źű�ע
u8  Puls_Delay_Num=0;			  //�����źű�ע
u8  Action_Done_Flag =FALSE;		  //�ŷ���������ɱ�־
u8  Action_Delay_Flag=FALSE;		  //������ʱ���
u8  Send_UMF_Signle=FALSE;			  //����ж����������źű�־λ

//��������ź���ʱ����
u16 Output_Delay_Time=0;		      //���������ʱ


u32 Test_Pulse_Xaxsis=0;		  //����X�ᶪ����
u8  Test_Pulse_Xaxsis_Flag = FALSE;
u8  Auto_Action_Delay=DISABLE;	  //ж�Ͻ������ζ�����ʱ
/*------------------------------------------------------------
// ˵��:ȡ�ϵ�Xλ�ô�position2��ʼ�����ϵ�λ�ô�position1��ʼ //
//      ���μ��һ������Ϊ��һ����㣬ȡ��position2~6 ������  //
//	    ��Ʒ����Ϊposition1~5��ԭʼ״̬������������       	  //
//      ���ֿ���������ʱ����position1������Ϊȡ�ϵ�X����	  //
  ------------------------------------------------------------*/
/* ��Ʒȡԭ�Ϻͷų�Ʒλ�� */
u8  Load_Material_Row_Number=6;    //ȡ�ϵ�һ��ԭ�ϸ�����Ĭ��Ϊ6
u32 Product_Position[10]={0};
u32 Interval_Position=5000;			 //5000:5cm
/* ȡ�Ϻͷų�Ʒ��λ��������0~5 */
u8  Load_Material_Number=1;
u8  Place_Product_Number=0;

static u8 j =0;
u8 Read_IIC_Auto_Program[16]={0}; 
u8 Auto_Reset_Step = 0;
u8 Auto_Reset_Step_Done = FALSE;

u8 g_Robot_Has_Debuged_Flag=FALSE;	 //��е��֮ǰ״̬�Ƿ������ɱ�̵���״̬
u8 g_Program_Is_Debuging=FALSE;
u32 Target_Position_Can;

//--------------------------//
u8 LCD_Control_Flag=0;//���ư����LCD
u16 LCD_Control_Time=0;//���ư����LCD��ʱ

extern u8 Product_Num;
extern u16 Confirm_Order_Num,g_Delay_Timer;
extern u8 g_Delay_Timer_Flag;
/**************************************************************************************************
**  ��������  RobotAutoReset()
**	�����������
**	�����������
**	�������ܣ��Զ��µı�־λ����
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
u8 REST_Flag=0;
extern u8 Axis_Step;
void RobotAutoReset()
{
	u8 i=0;
		 for(i=0;i<35;i++)		//ָʾ�Ʋ���λ
		 {
			SetOutput(i);
		 }
		if(Axis_Step<6)
		{
			switch(JXS_Parameter.Origin[Axis_Step])
			{
				case 0x00:
					Axis_Step++;
					REST_Flag=0;
					break;
				case 0x20://X
					 if(Pulse_TotalCounter[X_Axsis] == MINROBOTPOSITION)
					 {
						Axis_Step++;
						REST_Flag=0;
					 }
					 else if(0==REST_Flag)
					 {
						 REST_Flag=1;
						AXisMove(X_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[X_Axsis]*100);
					 }
					break;
				case 0x10://Y
					 if(Pulse_TotalCounter[L_Axsis] == MINROBOTPOSITION)
					 {
						Axis_Step++;
						REST_Flag=0;
					 }
					 else if(0==REST_Flag)
					 {
						 REST_Flag=1;
						AXisMove(L_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[L_Axsis]*100);	
					 }						 
					break;
				case 0x08://Z
					 if(Pulse_TotalCounter[Z_Axsis] == MINROBOTPOSITION)
					 {
						Axis_Step++;
						REST_Flag=0;
					 }
					 else if(0==REST_Flag)
					 {
						 REST_Flag=1;
						AXisMove(Z_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[Z_Axsis]*100);	
					 }					
					 break;
				case 0x04://O
					 if(Pulse_TotalCounter[O_Axsis] == MINROBOTPOSITION)
					 {
						Axis_Step++;
						REST_Flag=0;
					 }
					 else if(0==REST_Flag)
					 {
						 REST_Flag=1;
						AXisMove(O_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[O_Axsis]*100);	
					 }					
					 break;
				case 0x02://A
					 if(Pulse_TotalCounter[A_Axsis] == MINROBOTPOSITION)
					 {
						Axis_Step++;
						REST_Flag=0;
					 }
					 else if(0==REST_Flag)
					 {
						 REST_Flag=1;
						AXisMove(A_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[A_Axsis]*100);	
					 }					
					 break;
				case 0x01://B
					 if(Pulse_TotalCounter[B_Axsis] == MINROBOTPOSITION)
					 {
						Axis_Step++;
						REST_Flag=0;
					 }
					 else if(0==REST_Flag)
					 {
						 REST_Flag=1;
						AXisMove(B_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[B_Axsis]*100);	
					 }
					 break;
			}
		}
		else
		{
////////------------------------����Ŀ��λ�ú󲻵���ServoStop_CAN������޷��������У�-------------------------------//////////
			 ServoStop_CAN(X_Axsis);
			 CAN_Servo_Stop_Action[X_Axsis]=FALSE;	
			 ServoStop_CAN(Z_Axsis);
			 CAN_Servo_Stop_Action[Z_Axsis]=FALSE;
			 ServoStop_CAN(L_Axsis);
			 CAN_Servo_Stop_Action[L_Axsis]=FALSE;
			 ServoStop_CAN(O_Axsis);
			 CAN_Servo_Stop_Action[O_Axsis]=FALSE;
			 ServoStop_CAN(A_Axsis);
			 CAN_Servo_Stop_Action[A_Axsis]=FALSE;
			 ServoStop_CAN(B_Axsis);
			 CAN_Servo_Stop_Action[B_Axsis]=FALSE;
////////------------------------����Ŀ��λ�ú󲻵���ServoStop_CAN������޷��������У�-------------------------------//////////
			
			Axis_Step=0;
			REST_Flag=0;			
			Auto_Reset_Step_Done = TRUE;			
			 if(g_Auto_LOrigin_Flag)
			 {
			 	g_Auto_LOrigin_Flag=FALSE;
			 }
			 else
			 {
				 //ResetOutput(O_IODEBUG_OUTPUT_6); 
				 g_AutoStatue = AUTO_WAITE;
			 }
			 Auto_Reset_Step=0;
			 Robot_Auto_Reset = TRUE;
			 g_Auto_Reset_Flag = FALSE;
		}	 	

}

/**************************************************************************************************
**  ��������  AutoReset()
**	�����������
**	�����������
**	�������ܣ��Զ��µı�־λ����
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void AutoReset()
{
	  Stop_Status = STOP;
	  Back_Origin_Flag = FALSE;
	  Origin_Backed = FALSE;
	  Robot_Auto_Reset = FALSE;
	  Auto_Run_Enable = DISABLE;
	  Once_Mode_Enable = DISABLE;
	  Single_Mode_Enable = DISABLE;			  
	  Loop_Mode_Enable = DISABLE;
	  Jog_Move_Enable = DISABLE;
	  Linked_Move_Enable = DISABLE;
	  g_AutoStatue = AUTO_WAITE;
	  Work_Status = WAIT_MODE;
	  g_Auto_ActionError_Flag=FALSE;
	  g_Auto_ActionTimeOut_Flag=FALSE;
	  g_Auto_ActionRun_Timer=0;
	  g_Auto_ActionRun_Step=0;
	  g_Auto_PresentLine = 0;			//�Զ������к�
	  Action_Step_List_Num=0;
	  Action_Step_Run_Num=0;
	  Action_Step_Confirm_Num=0;
	  g_USART_Delay_Timer=0;
	  g_LC_ActionRun_Timer=0;
	  g_LC_ActionRun_Step=0;
	  g_LC_PresentLine = 0;			//�Զ������ϲ��к�
	  g_Read_LC_Load_Program=FALSE;
	  g_Read_LC_UnLoad_Program=FALSE;
	  g_LC_Step_Control=FALSE;
	  g_Start_Step_Control=FALSE;
    g_Program_Start_Action=FALSE;
	  g_Program_Reset_Action=FALSE;
}

/**************************************************************************************************
**  ��������  Auto_Start()
**	�����������
**	�����������
**	�������ܣ�����������
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2014/2/13 
***************************************************************************************************/
void AutoStart()
{
	 Auto_Run_Enable = ENABLE;
	 Once_Mode_Enable = ENABLE;
	 Single_Mode_Enable = ENABLE;
	 Loop_Mode_Enable = ENABLE;
	 Section_Action_Complete = FALSE;	 //���ڶ�����ͣ			 
	 Stop_Time = 0;
	 Stop_Status = RUN;
	 Error_Data = Error_Data & 0xfdff;	 //����ӹ���ɱ����ź�	
	 Watch_Data = (Watch_Data&0xbf) | 0x80 ;		//1000
}

/**************************************************************************************************
**  ��������  AutoPause()
**	�����������
**	�����������
**	�������ܣ���ͣ������
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2015/8/10 
***************************************************************************************************/
void AutoPause()
{
	Step_Done = FALSE;
	Stop_Status = PAUSE;
	Watch_Data = (Watch_Data&0x7f) | 0x40 ;
}

/**************************************************************************************************
**  ��������  AutoStop()
**	�����������
**	�����������
**	�������ܣ�ֹͣ������
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2014/2/13 
***************************************************************************************************/
void AutoStop()
{
//	 Stop_Time++;			 
//	 //��ͣ
//	 if(Stop_Time == 1)
//	 {
//		 Step_Done = FALSE;
//		 Stop_Status = PAUSE;
//         Watch_Data = (Watch_Data&0x7f) | 0x40 ;					 							 
//	 }
//	 //ֹͣ
//	 if(Stop_Time == 2)
//	 {
	 	 AutoReset();
		 Watch_Data = Watch_Data & 0x3f ;	 				 
//	 } 
}

/**************************************************************************************************
**  ��������  SafeAreaJudge()
**	�����������
**	�����������
**	�������ܣ��Զ�����ʱ��ȫ�����ж�
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/2/17
***************************************************************************************************/
void SafeAreaJudge()
{
 	u8 i=0;	
	//�жϰ�ȫ���Ƿ�����
	for(i=0;i<3;i++)
	{
		if(Robot_Safe_Area[i].SafeArea_Switch)
		{
			SafeArea_Is_On=TRUE;
			break;
		}
		else
		{
			SafeArea_Is_On=FALSE;
		}
	}

//	//���Z��λ��0��,����Ϊ���԰�ȫ
//	if(Pulse_TotalCounter[1]==MINROBOTPOSITION)
//	{
//		Safe_Area_Alarm=FALSE;
//	}
//	else
//	{
		//�簲ȫ����������,��е����ɻ�ԭ��,��X��λ�ڰ�ȫ���Ž����ж�
		if(SafeArea_Is_On==TRUE && Origin_Backed==TRUE)
		{
			for(i=0;i<3;i++)
			{
				if(Robot_Safe_Area[i].SafeArea_Switch)
				{
					if((Pulse_TotalCounter[0]>=Robot_Safe_Area[i].X_Left) && (Pulse_TotalCounter[0]<=Robot_Safe_Area[i].X_Right))
					{
						g_Current_SafeArea_Num=i+1;  //1-2-3��ǰX�ᴦ�ڰ�ȫ������
						break;
					}
				}
				else
				{
					g_Current_SafeArea_Num=0;		 //�����ڰ�ȫ�����ڲ�
				}
			}
			//X��λ��ĳ����ȫ����ʱ
			if(g_Current_SafeArea_Num)
			{	
			   //Z�� ���ڰ�ȫ��Z����λ��	�� С�ڰ�ȫ��Z����λ��
			   if((Pulse_TotalCounter[1]>Robot_Safe_Area[g_Current_SafeArea_Num-1].Z_Down)||(Pulse_TotalCounter[1]<Robot_Safe_Area[g_Current_SafeArea_Num-1].Z_Up)) 
			   {
					  Safe_Area_Alarm=TRUE;
			   }
			   else
			   {
			   		Safe_Area_Alarm=FALSE;
			   }					
			}
			//X�᲻λ���κΰ�ȫ����ʱ
			else  
			{
			   //Z�� �������·�
			   if(Pulse_TotalCounter[1]>MINROBOTPOSITION) 
			   {
					  Safe_Area_Alarm=TRUE;
			   }
			   else
			   {
			   		Safe_Area_Alarm=FALSE;
			   }
			}
		} //if(SafeArea_Is_On==TRUE && Origin_Backed==TRUE)
		else 
		{
			g_Current_SafeArea_Num=0;
		}
//	}
	//������ȫ��
	if(Safe_Area_Alarm)
	{
		Robot_Error_Data[0] = Robot_Error_Data[0] | 0x02;
		Timer_Enable_X = DISABLE;
		Timer_Enable_Z = DISABLE;
		Timer_Enable_L = DISABLE;
		Timer_Enable_O = DISABLE;
		Timer_Enable_A = DISABLE;
		Timer_Enable_B = DISABLE;
		//---2017.9.22
		ServoEmergencyStop_CAN(X_Axsis);	
		ServoEmergencyStop_CAN(Z_Axsis);
		ServoEmergencyStop_CAN(L_Axsis);
		ServoEmergencyStop_CAN(O_Axsis);
		ServoEmergencyStop_CAN(A_Axsis);
		ServoEmergencyStop_CAN(B_Axsis);
	}
	else
	{
		Robot_Error_Data[0] = Robot_Error_Data[0] & 0xfd;
	}
}

/**************************************************************************************************
**  ��������  MoveToTarget()
**	�����������
**	�����������
**	�������ܣ��˶���Ŀ��λ��
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void MoveToTarget(u32 Target_Position,u8 Axis,u32 Axsis_Speed)
{
//	 if((Axis == X_Axsis)&&(Timer_Enable_X == ENABLE))
//	 {
//	 }
//	 else if((Axis == Z_Axsis)&&(Timer_Enable_Z == ENABLE))
//	 {
//	 }
//	 else if((Axis == L_Axsis)&&(Timer_Enable_L == ENABLE))
//	 {
//	 }
//	 else if((Axis == O_Axsis)&&(Timer_Enable_O == ENABLE))
//	 {
//	 }
//	 else if((Axis == A_Axsis)&&(Timer_Enable_A == ENABLE))
//	 {
//	 }
//	 else if((Axis == B_Axsis)&&(Timer_Enable_B == ENABLE))
//	 {
//	 }
//	 else
//	 {		 
     //---2017.8.21
		 Target_Position_Can=Target_Position-1000000;		 				 
		 MoveToTarget_CAN(Axis,Target_Position_Can,Axsis_Speed);
//   }
}

/**************************************************************************************************
**  ��������  SetSingle()
**	�����������
**	�����������
**	�������ܣ��ź����������λ��־λ
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/3/10 
***************************************************************************************************/
void SetSingle(u8 Reset_Output,u8 Set_Output, u32 Detect_Flag)
{
	 //�ź��������λ , ��λ , �����ź�
	 if(Reset_Output ==60)
	 { //���ź�-��λĳIO
	 }
	 else
	 {
	 	SetOutput(Reset_Output);					   //��λ  ��ǰ�ź���Ϊ1 ��Ӧ����λ 1
	 	//Puls_Delay_Enable[Reset_Output]=DISABLE;	   //�����־λ
	 }
	 if(Set_Output ==60)
	 { //���ź�-��λĳIO
	 }
	 else
	 {
		 ResetOutput(Set_Output);						   //��λ	��ǰ�ź���Ϊ0 ��Ӧ����λ0
		 if(Detect_Flag==15)
		 {
		 	Puls_Delay_Num++;
			Puls_Delay_Enable[Set_Output]=ENABLE;	   //��λ��־λ
		 }
	 }
}

/**************************************************************************************************
**  ��������  Read_SaveProgram_IIC_Address()
**	�����������
**	�����������
**	�������ܣ��Զ�ģʽ�µĸ��ֲ���
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2016/01/10 
***************************************************************************************************/
void Read_SaveProgram_IIC_Address()
{
	 u8 i=0;
	 SaveProgram_IIC_Address=0;
	 SaveProgram_IIC_Num=0;
	 if(g_Run_Program_Num>0)
	 {
		 if(g_Run_Program_Num>SAVEPROGRAMNUM_2)
		 {
		     SaveProgram_IIC_Address=Program_IIC_Address[g_Run_Program_Num-1].Address;
			 SaveProgram_IIC_Num = Program_IIC_Address[g_Run_Program_Num-1].Num;
		 }
		 else
		 {
			 for(i=0;i<SAVEPROGRAMNUM_2;i++)
			 {
			 	//��ѯ������뵱ǰ�趨�Զ����б����ͬ����
				if(Program_IIC_Address[i].Code == g_Run_Program_Num)
				{
				  SaveProgram_IIC_Address=Program_IIC_Address[i].Address;
					SaveProgram_IIC_Num=Program_IIC_Address[i].Num;
					break;
				}
			 }
		 }
	 } 
}

/**************************************************************************************************
**  ��������  AutoRun()
**	�����������
**	�����������
**	�������ܣ��Զ�ģʽ�µĸ��ֲ���
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2015/08/10 
***************************************************************************************************/
void AutoRun()
{
	u8 k=0;
	switch(UsartReceiveData[1])
	{
		case P_CYCLE_MODE:
		     Auto_Mode = LOOP_MODE;
			 Loop_Mode_Enable = ENABLE;
			 break;	

		case P_SINGLE_MODE:		  //����ģʽ
		     Auto_Mode = SINGLE_MODE;
			 Single_Mode_Enable = ENABLE;
			 break;	

		case P_ACTION_RUN:
			 if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
			 {				 			
				g_Auto_Order_Start = TRUE;
				//AutoStart();
				if(Auto_Mode == SINGLE_MODE)
				{
					Single_Mode_Enable = ENABLE;
					Loop_Mode_Enable = DISABLE;
				}
				else if(Auto_Mode == LOOP_MODE)
				{
					Loop_Mode_Enable = ENABLE;
					Single_Mode_Enable = DISABLE;
				}
			 }
		     break;	

		case P_ACTION_PAUSE:
		     if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
			 {
				//AutoPause();
				g_Auto_Order_Pause = TRUE;
			 }
		     break;

		case P_ACTION_STOP:
		     if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
			 {
				 g_LCDelay_Step=0;
				 g_LCDelay_Timer=0;
				 
				 g_Delay_Timer_Flag=0;
				 g_Delay_Timer=0;				 
				 Confirm_Order_Num=0;
				 g_Auto_PresentLine=0;
				 Action_Step_Run_Num=0;		
				 g_Auto_Order_Stop = TRUE;
				 if(LCD_Control_Flag==1)
					LCD_Control_Flag=2;
			 }
		     break;

		case P_ACTION_PROGRAM:
			 g_Run_Program_Num =  UsartReceiveData[2];
			 //д��IIC
			 W25QXX_Write(&g_Run_Program_Num,0x40E0,1);	
			 Read_SaveProgram_IIC_Address();
			 if(SaveProgram_IIC_Address>0)
			 {
				 W25QXX_Read(Read_IIC_Auto_Program,SaveProgram_IIC_Address,7);
			 	 Free_Program_Operate.Flag  = Read_IIC_Auto_Program[0];
			 	 Free_Program_Operate.Code  = Read_IIC_Auto_Program[1];
				 Free_Program_Operate.Name  = (u32)( ((u32)Read_IIC_Auto_Program[2])|((u32)Read_IIC_Auto_Program[3]<<8)|((u32)Read_IIC_Auto_Program[4]<<16)|((u32)Read_IIC_Auto_Program[5]<<24) );
				 Free_Program_Operate.Num   = Read_IIC_Auto_Program[6];
				 for(j=0;j<Free_Program_Operate.Num;j++)
				 {
				 	W25QXX_Read(Read_IIC_Auto_Program,SaveProgram_IIC_Address+0x07+0x10*j,16);
					Free_Program_Operate.Program[j].Flag   = Read_IIC_Auto_Program[0];
					Free_Program_Operate.Program[j].List   = Read_IIC_Auto_Program[1];
				 	Free_Program_Operate.Program[j].Order  = Read_IIC_Auto_Program[2];
					Free_Program_Operate.Program[j].Key    = Read_IIC_Auto_Program[3];
					Free_Program_Operate.Program[j].Value1 = (u32)( ((u32)Read_IIC_Auto_Program[4])|((u32)Read_IIC_Auto_Program[5]<<8)|((u32)Read_IIC_Auto_Program[6]<<16)|((u32)Read_IIC_Auto_Program[7]<<24) );
				  Free_Program_Operate.Program[j].Value2 = (u32)( ((u32)Read_IIC_Auto_Program[8])|((u32)Read_IIC_Auto_Program[9]<<8)|((u32)Read_IIC_Auto_Program[10]<<16)|((u32)Read_IIC_Auto_Program[11]<<24) );	
					Free_Program_Operate.Program[j].Value3 = (u32)( ((u32)Read_IIC_Auto_Program[12])|((u32)Read_IIC_Auto_Program[13]<<8)|((u32)Read_IIC_Auto_Program[14]<<16)|((u32)Read_IIC_Auto_Program[15]<<24) ); 					
					Free_Program_Operate.Program[j].Value1 = Free_Program_Operate.Program[j].Value1 & 0x0fffffff;
				  Free_Program_Operate.Program[j].Value2 = Free_Program_Operate.Program[j].Value2 & 0x0fffffff;	
					Free_Program_Operate.Program[j].Value3 = Free_Program_Operate.Program[j].Value3 & 0x0fffffff;
					if(K_ZAXISPOSITION<=Free_Program_Operate.Program[j].Key && Free_Program_Operate.Program[j].Key<=K_OAXIS)	//������
					{
						Free_Program_Operate.Program[j].Value1 = Free_Program_Operate.Program[j].Value1+MINROBOTPOSITION;
					}
				 }
				 for(k=j;k<LARGESTPROGRAMNUM;k++)
				 {
					Free_Program_Operate.Program[k].Flag   = 0;
					Free_Program_Operate.Program[k].List   = 0;
				 	Free_Program_Operate.Program[k].Order  = 0;
					Free_Program_Operate.Program[k].Key    = 0;
					Free_Program_Operate.Program[k].Value1 = 0;
				  Free_Program_Operate.Program[k].Value2 = 0;	
					Free_Program_Operate.Program[k].Value3 = 0; 								 			 
				 }
				 //��ȡ�ϲ�ָ����
				 Read_LC_List(g_Run_Program_Num);				    
			 }
	     break;

		case P_ACTION_RESET:  //��е�ָ�λ	
			 Axis_Step=0;
			 Work_Status = AUTO_WORK_MODE;
			 g_Auto_Reset_Flag = TRUE;
			 Robot_Auto_Reset = FALSE;
			 g_Auto_LOrigin_Flag = FALSE;
		   break;

		case P_ACTION_DEBUG:  //��е�ֵ���			 
			 Work_Status = AUTO_WORK_MODE;			 
			 Auto_Mode = SINGLE_MODE;
			 Single_Mode_Enable = ENABLE;
			 g_Auto_PresentLine = UsartReceiveData[2];	  //��ǰ��������
			 g_Auto_ActionRun_Step=0;
			 Robot_Auto_Reset=TRUE;
			 //g_Auto_Reset_Flag=FALSE;
			 g_Auto_Order_Start = TRUE;
			 g_Robot_Has_Debuged_Flag=TRUE;
			 g_Program_Is_Debuging=TRUE;
		   break;

		case P_ACTION_LORIGIN:  //��е��L����			 
			 g_Auto_LOrigin_Flag = TRUE;
			 g_Auto_Reset_Flag = TRUE;
			 Robot_Auto_Reset = FALSE;
		   break;

		default:
		   break;
	}
}

/**************************************************************************************************
**  ��������  AutoMode()
**	�����������
**	�����������
**	�������ܣ��Զ�ģʽ�µĸ���ģʽѡ��
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void AutoMode()
{
	 switch(UsartReceiveData[1])
	 {
	    case ONCE_MODE:
		     Auto_Mode = ONCE_MODE;
			   UsartAcknowledge(AUTO_MODE);
         break;
		
			case SINGLE_MODE:
				 Auto_Mode = SINGLE_MODE;
				 UsartAcknowledge(AUTO_MODE);
				 break;
		
			case LOOP_MODE:
				 Auto_Mode = LOOP_MODE;
				 UsartAcknowledge(AUTO_MODE);
				 break;
			 		
			case AUTO_START:
				 if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
				 {				 
					AutoStart();
				 }	 
				 UsartAcknowledge(AUTO_MODE);
				 break;

			/*- ��һ������ͣ����������ֹͣ -*/
			case AUTO_STOP:
				 if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
				 {
					 AutoStop();
				 }
				 if(Work_Status == MANUAL_WORK_MODE)
				 {
					 Jog_Pause_Enable=ENABLE;
				 }
				 UsartAcknowledge(AUTO_MODE);
				 break;

 	   default:
		     break;	 
	 }
}

/**************************************************************************************************
**  ��������  AutoRunning()
**	�����������
**	�����������
**	�������ܣ��Զ�ģʽ�µĸ�ģʽ����
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void AutoRunning()
{
	switch(Auto_Mode)
	{
		case ONCE_MODE:		//����ģʽ	              				 
//		   if(Once_Mode_Enable == ENABLE)
//			 {   
//				 //���ڶ������֮����������һ�ڶ���  				 		 	
//			 }
			 break;
		
		case SINGLE_MODE:	//����ģʽ
		   if(Single_Mode_Enable == ENABLE)
			 {				
		 	   	AutoModeControl();				 
			 }
			 break;
		
		case LOOP_MODE:		//ѭ��ģʽ
		   if(Loop_Mode_Enable == ENABLE)
			 {
				  AutoModeControl(); 
			 }
			 break;

 		default:
		     break;	
	}
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
