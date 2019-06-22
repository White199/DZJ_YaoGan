/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : main.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 02/10/2017
* Description        : This is the ....
***************************************************************************************/
#include "CANopen.h"
#include "StatusControl.h"
#include "Auto.h"
#include "Manual.h"
#include "Parameter.h"
#include "Error.h"
#include "SpeedControl.h" 



/**************************************************************************************************
**  ��������  DirectionJudge()
**	�����������
**	�����������
**	�������ܣ���е�ֶ�������
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void DirectionSet_CAN(u8 Axis)
{
    if(Axsis_Move_Direction[Axis] == POSITIVE)
	{
	    Change_Polarity_Positive(Axis);
	}
	else
	{
	    Change_Polarity_Negtive(Axis);
	}
}


/**************************************************************************************************
**  ��������  MoveToTarget_CAN()
**	�����������
**	�����������
**	�������ܣ��˶���Ŀ��λ��,CAN���߷���λ�ú��ٶ���Ϣ
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2017/02/10 ---2017.8.1---zhang
***************************************************************************************************/
void MoveToTarget_CAN(u8 Axis,u32 Target_Position,u32 Axis_Speed)
{
	 u8 Node_ID=0;
	 Node_ID=Axis;
	//if(Min_Robot_Position <= Target_Position && Target_Position <= Max_Robot_Position)
//		 {
			switch(Axis)
			{
				case X_Axsis:
					if(Set_Halt_Flag[X_Axsis])//�Ƿ�����ͣ
					{
					   Reset_Halt(Node_ID);
					   Set_Halt_Flag[X_Axsis]=0;	
					}///--------------����Ӧ��Ҫ��һ���жϣ��Ƿ��Ǹ�λ�����ж��Ƿ�����ͣ״̬����-----------------/
				   ServoStart_PDO(Node_ID,Target_Position,Axis_Speed);
				   Target_Rreached[X_Axsis]=0;
				   //---2017.8.22
				   Timer_Enable_X = ENABLE;
				   break;

				case Z_Axsis:
					if(Set_Halt_Flag[Z_Axsis])//�Ƿ�����ͣ
					{
					   Reset_Halt(Node_ID);
					   Set_Halt_Flag[Z_Axsis]=0;				
					}
					 ServoStart_PDO(Node_ID,Target_Position,Axis_Speed);
				   Target_Rreached[Z_Axsis]=0;
			   //---2017.8.22
				   Timer_Enable_Z = ENABLE;
				   break;
				
				case L_Axsis:
					if(Set_Halt_Flag[L_Axsis])//�Ƿ�����ͣ
					{
					   Reset_Halt(Node_ID);
					   Set_Halt_Flag[L_Axsis]=0;				
					}
					 ServoStart_PDO(Node_ID,Target_Position,Axis_Speed);
				   Target_Rreached[L_Axsis]=0;
				 //---2017.8.22
				   Timer_Enable_L = ENABLE;
				   break;
				
				case O_Axsis:
					if(Set_Halt_Flag[O_Axsis])//�Ƿ�����ͣ
					{
					   Reset_Halt(Node_ID);
					   Set_Halt_Flag[O_Axsis]=0;				
					}
					 ServoStart_PDO(Node_ID,Target_Position,Axis_Speed);
				   Target_Rreached[O_Axsis]=0;
				 //---2017.8.22
				   Timer_Enable_O = ENABLE;
				   break;
				
				case A_Axsis:
					if(Set_Halt_Flag[A_Axsis])//�Ƿ�����ͣ
					{
					   Reset_Halt(Node_ID);
					   Set_Halt_Flag[A_Axsis]=0;				
					}
					ServoStart_PDO(Node_ID,Target_Position,Axis_Speed);
				    Target_Rreached[A_Axsis]=0;
				 //---2017.8.22
				   Timer_Enable_A = ENABLE;
				   break;
				
				case B_Axsis:
					if(Set_Halt_Flag[B_Axsis])//�Ƿ�����ͣ
					{
					   Reset_Halt(Node_ID);
					   Set_Halt_Flag[B_Axsis]=0;				
					}
					ServoStart_PDO(Node_ID,Target_Position,Axis_Speed);
					Target_Rreached[B_Axsis]=0;
				 //---2017.8.22
				   Timer_Enable_B = ENABLE;
				   break;
				default:
			 break;
             }			 		   
//     }
}

/**************************************************************************************************
**  ��������  Servo_Stop_CAN()
**	�����������
**	�����������
**	�������ܣ�ֹͣ��
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2017/02/13
***************************************************************************************************/
u8 CAN_Servo_Stop_Status[MAX_AXIS_NUM+1]={0};	 //�ŷ���ֹͣ״̬��ʶλ
u8 CAN_Servo_Stop_Action[MAX_AXIS_NUM+1]={0};	 //�ŷ���ֹͣ�����Ƿ�ִ�б�ʶλ
void ServoStop_CAN(u8 Axis)
{
	 u8 Node_ID=0;
	 if(Axis==MAX_AXIS_NUM)
	 {
	     Node_ID=0;
		 ServoStop_PDO(Node_ID);
		 CAN_Servo_Stop_Action[Axis]=TRUE;	//δ����
	 }
	 else if(CAN_Servo_Stop_Status[Axis]==FALSE && CAN_Servo_Stop_Action[Axis]==FALSE)
	 {
		 Node_ID=Axis+1;
		 switch(Axis)
		 {
				case X_Axsis:
					 Timer_Enable_X = DISABLE;
					 ServoStop_PDO(Node_ID);
					 break;
				case Z_Axsis:
					 Timer_Enable_Z = DISABLE;
					 ServoStop_PDO(Node_ID);
					 break;
				case L_Axsis:
					 Timer_Enable_L = DISABLE;
					 ServoStop_PDO(Node_ID);
					 break;
				case O_Axsis:
					 Timer_Enable_O = DISABLE;
					 ServoStop_PDO(Node_ID);
					 break;
				case A_Axsis:
					 Timer_Enable_A = DISABLE;
					 ServoStop_PDO(Node_ID);
					 break;
				case B_Axsis:
					 Timer_Enable_B = DISABLE;
					 ServoStop_PDO(Node_ID);
					 break;
			  default:
					 break;
		 }
		 CAN_Servo_Stop_Action[Axis]=TRUE;	//δ���� 
	 }
}

/**************************************************************************************************
**  ��������  ServoEmergencyStop_CAN()
**	�����������
**	�����������
**	�������ܣ�ֹͣ��
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2017/02/13
***************************************************************************************************/
void ServoEmergencyStop_CAN(u8 Axis)
{
	 u8 Node_ID=0;
	 if(Axis==MAX_AXIS_NUM)
	 {
	     Node_ID=0;
	 }
	 else
	 {
	     Node_ID=Axis+1;
	 }
	 ServoEmergencyStop_PDO(Node_ID);	 //����ָֹͣ��
}


/******************* (C) COPYRIGHT 2017 Kingrobot manipulator Team ******** END OF FILE ************************/

