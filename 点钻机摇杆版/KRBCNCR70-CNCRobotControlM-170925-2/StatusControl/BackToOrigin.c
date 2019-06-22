#include "stm32f10x_lib.h"
#include "Usart.h" 
#include "BackToOrigin.h"
#include "StatusControl.h"
#include "SpeedControl.h"
#include "in.h"
#include "Auto.h"
#include "SignalWatch.h"
#include "Parameter.h"
#include "Homing.h"
#include "PDO.h"
#include "CANopen.h"
#include "Manual.h"
#include "SDO.h"


u8 Axis_Step=0;
u32 Origin_Pulse = 0;	        //��ԭ��֮���ٴε��ٻ�ԭ�������ֵ
u8  Origin_Step = 0;	        //��ԭ�㲽��
u8  Origin_Time =FALSE;       //�¼������־λ
u8  Origin_Single_Flag=FALSE; //��ԭ���źű�־λ
u32 Origin_Speed[6] = 5;		      //��ԭ���ٶȣ�Ĭ��10%
u8  Robot_Enable = FALSE;
u8  Homing_Flag_Can=FALSE;    //---2017.8.30


//------------------------------------//
extern u16 Read_Point_Num,ShuZu_Count;
extern u8 Order_Read_Mode,A_List[6];

/**************************************************************************************************
**  ��������   
**	�����������
**	�����������
**	�������ܣ� 
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ� 
***************************************************************************************************/
void RobotEnable()
{
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
}

/**************************************************************************************************
**  ��������   
**	�����������
**	�����������
**	�������ܣ� 
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ� 
***************************************************************************************************/
void RobotDisable()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
}

/**************************************************************************************************
**  ��������   
**	�����������
**	�����������
**	�������ܣ� 
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ� 
***************************************************************************************************/
void RobotEnableOrigin()
{
	switch(UsartReceiveData[1])
	{
		case P_ROBOT_ORIGIN:
		     //Robot_Enable = TRUE;	//debug
			 if(Robot_Enable)  //��е����ʹ��-�����ԭ��
			 {
			  	Back_Origin_Flag = TRUE; //��е�ֻ�ԭ���־λʹ��
				  X_Origin_Backed=FALSE;
				  Z_Origin_Backed=FALSE;
				  L_Origin_Backed=FALSE;
				  O_Origin_Backed=FALSE;
				  Origin_Backed=FALSE;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
				  Origin_Time = FALSE;
				  Origin_Step = 0;
				  //---2017.8.30
				  Homing_Flag_Can=TRUE;
				  //---2017.9.27
			 }
			 break;	

		case P_X_AXSIS_ORIGIN:
		   break;	

		case P_Z_AXSIS_ORIGIN:
		   break;	

		case P_ORIGIN_SPEED:
		   Origin_Speed[0] = (u32)( ((u32)UsartReceiveData[2]) | ((u32)UsartReceiveData[3]<<8) | ((u32)UsartReceiveData[4]<<16) | ((u32)UsartReceiveData[5]<<24) );
			 break;

		case P_ROBOT_ENABLE:
		   if(UsartReceiveData[2])
			 {
			 	RobotEnable();
				Robot_Enable = TRUE;
			 }
			 else
			 {
			 	RobotDisable();
				Robot_Enable = FALSE;
				Origin_Backed = FALSE;			   //����ѻ�ԭ���־			
			 }
			 break;

		default:
		   break;
	}
}

/**************************************************************************************************
**  ��������   
**	�����������
**	�����������
**	�������ܣ� 
**	��ע��	  ��
**  ���ߣ�     
**  �������ڣ� 
***************************************************************************************************/
void BackToOrigin()
{
	u8 i=0;
	if(Homing_Flag_Can==TRUE)
	{
		Homing_attained_status[X_Axsis]=0;
		Homing_attained_status[Z_Axsis]=0;
		Homing_attained_status[L_Axsis]=0;
		Homing_attained_status[O_Axsis]=0;
		Homing_attained_status[A_Axsis]=0;
		Homing_attained_status[B_Axsis]=0;
		Flag_ARM_Homing_x=0;			
		Flag_ARM_Homing_z=0;			
		Flag_ARM_Homing_l=0;			
		Flag_ARM_Homing_o=0;			
		Flag_ARM_Homing_a=0;			
		Flag_ARM_Homing_b=0;		
		Axis_Step=0;
		Homing_Mode_Init();
		Delay(100);
		Homing_Flag_Can=FALSE;
	}
	if(Back_Origin_Flag == TRUE)
	{
		if(Axis_Step<6)
		{
			switch(JXS_Parameter.Origin[Axis_Step])
			{
				case 0x00:
					Axis_Step++;
					break;
				case 0x10://Y
					ARM_Homing(L_Axsis);
					break;
				case 0x20://X
					ARM_Homing(X_Axsis);
					break;
				case 0x08://Z
					ARM_Homing(Z_Axsis);
					break;
				case 0x04://O
					ARM_Homing(O_Axsis);
					break;
				case 0x02://A
					ARM_Homing(A_Axsis);
					break;
				case 0x01://B
					ARM_Homing(B_Axsis);
					break;
			}
		}
		else
		{
			Origin_Backed=TRUE;
		}
		if(Origin_Backed == TRUE)
		{
			PulseInit_Auto();					
			for(i=0;i<6;i++)
				A_List[i]=0;
			 Back_Origin_Flag = FALSE;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
			 Origin_Time = FALSE;        //�¼������־λ 
			 Origin_Step = 0;            //��ԭ�㲽��
			 Robot_Reset();
			 Read_Point_Num=0;
			 Order_Read_Mode=0;
			 ShuZu_Count=0;
			 Order_Package();
			 PP_Mode_Init();                                    //---2017.8.15������ɺ��ʼ����λ��ģʽ
		}	
    }	
}