/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "Usart.h" 
#include "Error.h"
#include "in.h" 
#include "out.h"
#include "w25qxx.h" 
#include "Manual.h"
#include "Auto.h"
#include "Auto_2.h"
#include "SpeedControl.h"
#include "signalWatch.h"
#include "StatusControl.h"
#include "Parameter.h"

#include "PDO.h"
#include "CANopen.h"

/*----- ȫ�ֱ��������������λ -----*/
u8 Error_Enable_Flag = FALSE;       
u8 Error_Status = NO_ERROR;
u8 Machine_BreakDown_Flag=FALSE;	 //�������ϱ�־λ
u8 EmergencyStop_Flag=FALSE;		 //��ͣ��־λ
u8 Full_Material_Flag=FALSE;		 //���ϱ�־λ
u8 Lack_Material_Flag=FALSE;		 //ȱ�ϱ�־λ
u8 XAxsis_Error_Flag=FALSE;
u8 ZAxsis_Error_FLAG_FASLE;
u8 Limit_Error_Flag[4]={FALSE};		 //��λ������־�����ڲ�������
u8 Cancle_Limit_Warning=FALSE;		 //ȡ����λ����ҳ����ʾ
u8 Cancle_Genaral_Warning=FALSE;   //ȡ����ǰ�������
u8 Cancle_Genaral_Warning_Num=0;
u8 Limit_Error=FALSE;
/*old-JR105*/
/*- �������ݣ�ÿһλ����һ�����ͱ�����1��ʾ������0��ʾ�ޱ���                -*/
/*   bit 15~8 																 */
/*   1     1        1      1        1        1           1            1      */
/* ��ͣ �������� X�ᱨ�� Z�ᱨ�� ȱ�ϱ��� ���ϱ���  �ӹ���ɱ���   �ϲּ�ͣ  */
/*   bit 7~0																 */
/*   1       1        1      1          1         1           1            1 */
/* X���� X�Ҽ��� Z�ϼ��� Z�¼���  ���޴��� �Զ���ȫ��                    */
u16 Error_Data = 0;



/*- �������ݣ�ÿһλ����һ�����ͱ�����1��ʾ������0��ʾ�ޱ���                -*/
/* Data[0] ��ͣ   ��   �������� ����λ ��ȫ�� ��ѹ�쳣 �󻬱���   Ԥ�� */
/* [7:0]    1      1      1      1       1       1        1         1 */
u8 Robot_Error_Data[4] = {0};			
u8 ServoEmergencyStop_Flag=0;//---2017.9.28
u8 Emerg_Flag=0;

/**************************************************************************************************
**  ��������  EmergencyStopJudge()
**	�����������
**	�����������
**	�������ܣ���ͣ�ж�
**	��ע��	  ��ͣ���£�Ϊ�ߵ�ƽ��Ч�ź�
**  ���ߣ�          
**  �������ڣ� 
***************************************************************************************************/
void EmergencyStopJudge(void)
{			
	if(ReadEmergencyStop())					          //��ȡ��ͣI��,��ͣΪ����״̬���ߵ�ƽ��Ч
	{	  
		EmergencyStop_Flag = TRUE;
		Robot_Error_Data[0] = Robot_Error_Data[0] | 0x80;
		Timer_Enable_X = DISABLE;
		Timer_Enable_Z = DISABLE;
		Timer_Enable_L = DISABLE;
		Timer_Enable_O = DISABLE;
		Timer_Enable_A = DISABLE;
		Timer_Enable_B = DISABLE;		
		
		//Error_Status = ERROR_EMERG_HAPPEN;	
		//---2017.9.22
		ServoEmergencyStop_CAN(X_Axsis);	
		ServoEmergencyStop_CAN(Z_Axsis);
		ServoEmergencyStop_CAN(L_Axsis);
		ServoEmergencyStop_CAN(O_Axsis);
		ServoEmergencyStop_CAN(A_Axsis);
		ServoEmergencyStop_CAN(B_Axsis);
		
		ServoEmergencyStop_Flag=1;//---2017.9.28
	}
	else						                       	  //��ͣ��Ϊ�ͷ�״̬
	{
		EmergencyStop_Flag = FALSE;
		Robot_Error_Data[0] = Robot_Error_Data[0] & 0x7f;
	}
}


/**************************************************************************************************
**  ��������  LimitSwitchJudge()
**	�����������
**	�����������
**	�������ܣ���⼫�޿���
**	��ע��	  ��
**  ���ߣ�          
**  �������ڣ� 
***************************************************************************************************/
void LimitSwitchJudge(void)
{			
	 /* X�����޼�� */
	 if(!ReadInput(X_LEFT_LIMIT))	 
	 {
		 if(Cancle_Limit_Warning==FALSE)
		 {
//		    Robot_Error_Data[0] = Robot_Error_Data[1] | 0x80;	
		 }
		 Limit_Error_Flag[0]=TRUE;
	 }
	 else
	 {
//	 	   Robot_Error_Data[0] = Robot_Error_Data[1] & 0x7f;
		   Limit_Error_Flag[0]=FALSE;
	 }
	 /* X���Ҽ��޼�� */
	 if(!ReadInput(X_RIGHT_LIMIT))
	 {
		 if(Cancle_Limit_Warning==FALSE)
		 {
		    Error_Data = Error_Data | 0x0040;	
		 }
		 Limit_Error_Flag[1]=TRUE;
	 }
	 else
	 {
		 Error_Data = Error_Data & 0xffbf;
		 Limit_Error_Flag[1]=FALSE;		 
	 }

	 /* Z���ϼ��޼�� */
	 if(!ReadInput(Z_LEFT_LIMIT))	 
	 {
		 if(Cancle_Limit_Warning==FALSE)
		 {
		    Error_Data = Error_Data | 0x0020;	
		 }
		 Limit_Error_Flag[2]=TRUE;		 	
	 }
	 else
	 {
	 	 Error_Data = Error_Data & 0xffdf;
		 Limit_Error_Flag[2]=FALSE;	
	 }

	 /* Z���¼��޼�� */
	 if(!ReadInput(Z_RIGHT_LIMIT))
	 {
		 if(Cancle_Limit_Warning==FALSE)
		 {
		    Error_Data = Error_Data | 0x0010;	
		 }
		 Limit_Error_Flag[3]=TRUE;	
	 }
	 else
	 {
		 Error_Data = Error_Data & 0xffef;
		 Limit_Error_Flag[3]=FALSE;
	 }
	 //
	 if((Limit_Error_Flag[0]==FALSE) && (Limit_Error_Flag[1]==FALSE) 
	     && (Limit_Error_Flag[2]==FALSE)&& (Limit_Error_Flag[3]==FALSE))
	 {
	 	 Limit_Error=FALSE;
		 Cancle_Limit_Warning=FALSE;
	 }
	 else  //��⵽���޿���
	 {
	 	 Origin_Backed=FALSE;		 //��ԭ���־���㣬��Ҫ���»�ԭ��
	 }

}


/**************************************************************************************************
**  ��������  SoftLimitJudge()
**	�����������
**	�����������
**	�������ܣ����޼��
**	��ע��	  ��
**  ���ߣ�          
**  �������ڣ� 
***************************************************************************************************/
void SoftLimitJudge()
{
	 if(Origin_Backed == TRUE)
	 {
	 	if( Pulse_TotalCounter[0]<X_Axsis_Minlength-1 | Pulse_TotalCounter[0]>X_Axsis_Maxlength |
		    Pulse_TotalCounter[1]<Z_Axsis_Minlength-1 | Pulse_TotalCounter[1]>Z_Axsis_Maxlength | 
			Pulse_TotalCounter[2]<L_Axsis_Minlength-1 | Pulse_TotalCounter[2]>L_Axsis_Maxlength |
			Pulse_TotalCounter[3]<O_Axsis_Minlength-1 | Pulse_TotalCounter[3]>O_Axsis_Maxlength  )
		{
			Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
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
			Robot_Error_Data[0] = Robot_Error_Data[0] & 0xef;
		}
	 }
}


/**************************************************************************************************
**  ��������  MachineBreakDown()
**	�����������
**	�����������
**	�������ܣ���������
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2013/12/17
***************************************************************************************************/
void MachineBreakDown()
{
	 if((ReadInput(11) & ReadInput(19))==0)	  //11==0(�͵�ƽ):����1����  19==0(�͵�ƽ):����2����
	 {
	 	 Robot_Error_Data[0] = Robot_Error_Data[0] | 0x20;
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
	 	 Robot_Error_Data[0] = Robot_Error_Data[0] & 0xdf;
	 }
}

/**************************************************************************************************
**  ��������  MaterialEmergencyStop()
**	�����������
**	�����������
**	�������ܣ��ϲּ�ͣ PE8
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2013/12/17
***************************************************************************************************/
void MaterialEmergencyStop()
{
	if(Input_Detect_Status[4]&0x03)//X32 -- X33
	{
		Robot_Error_Data[0] = Robot_Error_Data[0] | 0x01;
	}
	else
	{
		Robot_Error_Data[0] = Robot_Error_Data[0] & 0xfe;
	}
}

/**************************************************************************************************
**  ��������  LackMaterialJudge()
**	�����������
**	�����������
**	�������ܣ�ȱ���ж�
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2013/12/17
***************************************************************************************************/
void LackMaterialJudge()
{
   
	if(g_Material_Load_Disk_Finished)					  //ȡ�ϲ�ȱ��
	{	  
		 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x40;
	}
	else						                       	  //δȱ��
	{
		 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xbf;
	}
}

/**************************************************************************************************
**  ��������  FullMaterialJudge()
**	�����������
**	�����������
**	�������ܣ������ж�
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2013/12/17
***************************************************************************************************/
void FullMaterialJudge()
{
	if(g_Material_UnLoad_Disk_Finished)					  //ж�ϲ�����״̬
	{	  
		 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x80;
	}
	else						                       	  //δ����
	{
		 Robot_Error_Data[1] = Robot_Error_Data[1] & 0x7f;
	}
}

/**************************************************************************************************
**  ��������  XAxsisError()
**	�����������
**	�����������
**	�������ܣ�X����󣬸ߵ�ƽ����
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2013/12/17
***************************************************************************************************/
void  XAxsisError()
{
	if( ReadXAxsisError() )					          //��ȡX������ź�
	{	  
		Error_Data = Error_Data | 0x2000;
		TIM_Cmd(TIM2, DISABLE);
		TIM_Cmd(TIM3, DISABLE);
		TIM_Cmd(TIM1, DISABLE);
		TIM_Cmd(TIM4, DISABLE);
		Timer_Enable_X = DISABLE;
		Timer_Enable_Z = DISABLE;
		Timer_Enable_L = DISABLE;	
		Timer_Enable_O = DISABLE;
		//---2017.9.22
		ServoEmergencyStop_CAN(X_Axsis);	
		ServoEmergencyStop_CAN(Z_Axsis);
		ServoEmergencyStop_CAN(L_Axsis);
		ServoEmergencyStop_CAN(O_Axsis);
		ServoEmergencyStop_CAN(A_Axsis);
		ServoEmergencyStop_CAN(B_Axsis);
		//Error_Status = ERROR_EMERG_HAPPEN;	
	}
	else						                       	  
	{
		Error_Data = Error_Data & 0xdfff;
	}		
}


/**************************************************************************************************
**  ��������  ZAxsisError()
**	�����������
**	�����������
**	�������ܣ�Z����󣬸ߵ�ƽ����
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2013/12/17
***************************************************************************************************/
void  ZAxsisError()
{
	if( ReadZAxsisError() )					          //��ȡZ������ź�
	{	  
		Error_Data = Error_Data | 0x1000;
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
		//Error_Status = ERROR_EMERG_HAPPEN;
	}
	else						                       	  
	{
		Error_Data = Error_Data & 0xefff;
	}	


}

/**************************************************************************************************
**  ��������  AxsisError()
**	�����������
**	�����������
**	�������ܣ�Z����󣬸ߵ�ƽ����
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2013/12/17
***************************************************************************************************/
void  AxsisError()
{
	if( ReadAxsisError(ERROR_STATUS)==TRUE )					          //��ȡ��е������ź�
	{	  
		Robot_Error_Data[0] = Robot_Error_Data[0] | 0x40;
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
		Robot_Error_Data[0] = Robot_Error_Data[0] & 0xbf;
	}	
}

/**************************************************************************************************
**  ��������  HardLimitJudge()
**	�����������
**	�����������
**	�������ܣ����޼��
**	��ע��	  ��
**  ���ߣ�          
**  �������ڣ� 
***************************************************************************************************/
void HardLimitJudge()
{
    if((Origin_Backed==TRUE)&&((Input_Detect_Status[4]&0xf0)!=0x00))
	{
		Robot_Error_Data[0] = Robot_Error_Data[0] | 0x08;
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
}

/**************************************************************************************************
**  ��������  AirAlarm()
**	�����������
**	�����������
**	�������ܣ����ñ���
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2016/01/16
***************************************************************************************************/
void  AirAlarm()
{
	 if(ReadInput(41)==0)	  //41==0(�͵�ƽ):��Դ����
	 {
	 	 Robot_Error_Data[0] = Robot_Error_Data[0] | 0x08;
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
	 	 Robot_Error_Data[0] = Robot_Error_Data[0] & 0xf7;
	 }
	
}

/**************************************************************************************************
**  ��������  OilingAlarm()
**	�����������
**	�����������
**	�������ܣ��󻬱���
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2016/01/16
***************************************************************************************************/
void  OilingAlarm()
{
	 if(ReadInput(40)==0)	  //40==0(�͵�ƽ):�󻬱���
	 {
	 	 Robot_Error_Data[0] = Robot_Error_Data[0] | 0x04;
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
	 	 Robot_Error_Data[0] = Robot_Error_Data[0] & 0xfb;
	 }

}

/**************************************************************************************************
**  ��������  ManulActionError()
**	�����������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2016/01/23
***************************************************************************************************/
void  ManulActionError()
{
	 if(Safe_Area_Flag)	  //
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x08;
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xf7;
	 }
}

/**************************************************************************************************
**  ��������  CurrentWorkFinished()
**	�����������
**	�����������
**	�������ܣ�������ʱ����
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2016/01/18
***************************************************************************************************/
void  CurrentWorkFinished()
{
	 if(g_Auto_WorkFinished_Flag)	  //
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x20;
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xdf;
	 }
}

/**************************************************************************************************
**  ��������  AutoActionError()
**	�����������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2016/01/18
***************************************************************************************************/
void  AutoActionError()
{
	 if(g_Auto_ActionError_Flag)	  //
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x10;
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xef;
	 }

}
/**************************************************************************************************
**  ��������  AutoActionTimeOut()
**	�����������
**	�����������
**	�������ܣ�������ʱ����
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2016/01/18
***************************************************************************************************/
void  AutoActionTimeOut()
{
	 if(g_Auto_ActionTimeOut_Flag)	  //
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x04;
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xfb;
	 }
}

/**************************************************************************************************
**  ��������  USARTSyncTimeOut()
**	�����������
**	�����������
**	�������ܣ�����ͨ�ų�ʱ����
**	��ע��	  ��
**  ���ߣ�    ����   
**  �������ڣ�2016/01/18
***************************************************************************************************/
void  USARTSyncTimeOut()
{
    if(g_USART_Delay_Timer>300)//����3000ms�ˣ��򱨴�
	{
	    if(g_AutoStatue==AUTO_RUNNING)	//���ɱ��-���� ״̬Ҳ��������,��������ͣ״̬
	    {
	   		g_Auto_Order_Pause = TRUE;
	    }
		else if(Work_Status==MANUAL_WORK_MODE)
		{
			if(Timer_Enable_X|Timer_Enable_Z|Timer_Enable_L|Timer_Enable_O)
			{
//                Servo_Stop(Axsis_Chosen);
				          ServoStop_CAN(Axsis_Chosen);
				          CAN_Servo_Stop_Action[Axsis_Chosen]=FALSE;
            }
		}
		g_USART_Delay_Timer=0;
	}
}

/**************************************************************************************************
**  ��������  ErrorOperate()
**	�����������
**	�����������
**	�������ܣ���������
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void ErrorOperate()
{
	 u8 i=0;
	 u8 Temp_data[4]={0};
	 EmergencyStopJudge(); //��ͣ���
	 AxsisError();		   //��е�ᱨ��
	 #ifdef HARDLIMITJUDGE_INPUT
//	     HardLimitJudge();	   //Ӳ��λ����
	 #endif
	 //MachineBreakDown();	   //�������ϼ��
//	 SoftLimitJudge();		   //����λ���	 
	 //AirAlarm();		       //���ñ���
	 //OilingAlarm();		   //�󻬱���
//	 SafeAreaJudge();	       //��ȫ�����	 --����
//	 ManulActionError();	   //�ֶ���������-������ȫ��,�뷴�����
	 //USARTSyncTimeOut();	   //����ͨ�ű��� 
	 //MaterialEmergencyStop();  //�ϲ���λ����-
	 if(Work_Status == AUTO_WORK_MODE)
	 {		 		 
		 //LackMaterialJudge();	   //ȱ�ϼ��-һ�����̼ӹ����
		 //FullMaterialJudge();	   //���ϼ��
		 CurrentWorkFinished();	   //�ӹ����		 
		 AutoActionError();	       //�Զ����н���AUTO_ERROR
		 AutoActionTimeOut();	   //������ʱ			  
	 }
	 
	 //ȡ����ǰ����
     if(Cancle_Genaral_Warning==TRUE)
	 {
		 //������ͣ���������㣬��������ֱ������
//		 if(Safe_Area_Alarm)
//		 {
//		 	Safe_Area_Alarm=FALSE;
//		 }
		 for(i=0;i<4;i++)
		 {
		 	Robot_Error_Data[i]=0;
		 }
//		 if(Robot_Error_Data[0] & 0x08)	 //��λ����
//		 {
//		 	Robot_Error_Data[0] = Robot_Error_Data[0] & 0xf7;
//		 }
		 g_Auto_ActionError_Flag=FALSE;
		 g_Auto_ActionTimeOut_Flag=FALSE;
		 Safe_Area_Flag=FALSE;
		 Cancle_Genaral_Warning=FALSE;
		 g_Material_Load_Disk_Finished=FALSE;
		 g_Material_UnLoad_Disk_Finished=FALSE;
		 if(g_Auto_WorkFinished_Flag)	//�����ǰ������������
		 {
			SC_Parameter.SC_Num=0;
			Temp_data[0] = SC_Parameter.SC_Num;
			Temp_data[1] = SC_Parameter.SC_Num>>8;
			Temp_data[2] = SC_Parameter.SC_Num>>16;
			Temp_data[3] = SC_Parameter.SC_Num>>24;
			W25QXX_Write(Temp_data,0x102C,4);
			g_Auto_WorkFinished_Flag=FALSE;		 
		 }	
    
	 }
	 for(i=0;i<4;i++)
	 {
	 	if(Robot_Error_Data[i] != 0 )
		{
			if(Robot_Error_Data[0]!=0)
			{
				Error_Status = ERROR_EMERG_HAPPEN;
				//---2017.9.28
				Emerg_Flag=1;
			}
			else
			{
				Error_Status = ERROR_HAPPEN;
			}
			break;
		}
		else
		{
			Error_Status = NO_ERROR;
		}
	 }
	 if(Error_Status==NO_ERROR)	       //δ��������
	 {	                
			//---2017.9.28
			if(Emerg_Flag)
			{
			//��װ״̬ת��ͼʵ��״̬ת��
			 Change_Servo_State_Machine(X_Axsis,SWITCH_ON);
			 Change_Servo_State_Machine(Z_Axsis,SWITCH_ON);
			 Change_Servo_State_Machine(L_Axsis,SWITCH_ON);
			 Change_Servo_State_Machine(O_Axsis,SWITCH_ON);
			 Change_Servo_State_Machine(A_Axsis,SWITCH_ON);
			 Change_Servo_State_Machine(B_Axsis,SWITCH_ON);
			 Delay(10);
			 Change_Servo_State_Machine(X_Axsis,ENABLE_OPERATION);
			 Change_Servo_State_Machine(Z_Axsis,ENABLE_OPERATION);
			 Change_Servo_State_Machine(L_Axsis,ENABLE_OPERATION);
			 Change_Servo_State_Machine(O_Axsis,ENABLE_OPERATION);
			 Change_Servo_State_Machine(A_Axsis,ENABLE_OPERATION);
			 Change_Servo_State_Machine(B_Axsis,ENABLE_OPERATION);
			 Emerg_Flag=0;
			}	
		   
	 }
   else					      //��������
	 {		 
		 if( Error_Status == ERROR_EMERG_HAPPEN ) //��ͣ������ֱ��ֹͣ��е��   //||(Robot_Error_Data[0]&0x40)||(Robot_Error_Data[0]&0x20)||(Robot_Error_Data[0]&0x01))	
		 { 
			   AutoReset();
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
			 
			     ServoEmergencyStop_Flag=1;//---2017.9.28
			 
		 }
		 else			//��������������ֹͣ����
		 {			  
			   if(g_AutoStatue==AUTO_RUNNING)	//���ɱ��-���� ״̬Ҳ��������,��������ͣ״̬
			   {
			   		g_Auto_Order_Pause = TRUE;
			   }	   
			   Linked_Move_Enable=DISABLE;
			   Jog_Move_Enable=DISABLE;	
			   if(Back_Origin_Flag)
			   {
			   	   Back_Origin_Flag = FALSE;
			   }              			   		 
		 }
	 }
	 //---2017.9.28�Ƿ���������жϼ�ͣ���ʧ��
	 if(ServoEmergencyStop_Flag)
	 {
		  //�ж��ŷ���λ�õ����ֹͣ
				if(Timer_Enable_X )
				{		     
					 if(Axsis_Stop_Flag[X_Axsis])//--�ж�X���Ƿ��Ѿ�ͣ����
					 {
						Timer_Enable_X = DISABLE;					
						Axsis_Stop_Flag[X_Axsis]=0;
					 }
				}	 
				if(Timer_Enable_Z )
				{		     
					 if(Axsis_Stop_Flag[Z_Axsis])//--�ж�Z���Ƿ��Ѿ�ͣ����
					 {								
							Timer_Enable_Z = DISABLE;
							Axsis_Stop_Flag[Z_Axsis]=0;
					  }
				 }
				if(Timer_Enable_L )
				{		     
					 if(Axsis_Stop_Flag[L_Axsis])//--�ж�L���Ƿ��Ѿ�ͣ����
					 {								
							Timer_Enable_L = DISABLE;
							Axsis_Stop_Flag[L_Axsis]=0;
					 }
				 }
				if(Timer_Enable_O )
				{		     
					 if(Axsis_Stop_Flag[O_Axsis])//--�ж�O���Ƿ��Ѿ�ͣ����
					 {
							Timer_Enable_O = DISABLE;	
							Axsis_Stop_Flag[O_Axsis]=0;
					 }
				}
				if(Timer_Enable_A )
				{		     
					 if(Axsis_Stop_Flag[A_Axsis])//--�ж�L���Ƿ��Ѿ�ͣ����
					 {								
							Timer_Enable_A = DISABLE;
							Axsis_Stop_Flag[A_Axsis]=0;
					 }
				 }
				if(Timer_Enable_B )
				{		     
					 if(Axsis_Stop_Flag[B_Axsis])//--�ж�O���Ƿ��Ѿ�ͣ����
					 {
							Timer_Enable_B = DISABLE;	
							Axsis_Stop_Flag[B_Axsis]=0;
					 }
				}
			if((Timer_Enable_X && Timer_Enable_Z && Timer_Enable_L && Timer_Enable_O&& Timer_Enable_A&& Timer_Enable_B)==DISABLE)
			{
				ServoEmergencyStop_Flag=0;
			}
	 }
	 else
	 {}
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
