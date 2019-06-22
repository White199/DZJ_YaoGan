/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : Auto_2.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 21/10/2015
* Description        : �Զ����п����ļ�
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "Auto_2.h"
#include "Parameter.h"
#include "ActionOperate.h"
#include "StatusControl.h"
#include "Auto.h"
#include "in.h"
#include "out.h"
#include "PDO.h"
#include "w25qxx.h"
#include "Usart.h" 
#include "SpeedControl.h" 

#include "Delay.h"
#include "SpeedControl_CAN.h"
/***************************��������������λ����ѯ�����**************************************/
u8  g_AutoStatue = AUTO_WAITE;		//�Զ�����״̬
u16  g_Auto_PresentLine = 0;			//�Զ������к�
u32 g_Auto_LastCircleTime = 0;
u32 g_Auto_Circle_Timer = 0;		//�Զ���������ʱ��
u32 g_Auto_ProduceCounter = 0;
u8  g_LC_Program_Step_Run=FALSE;    //�Ƿ������ϲֳ���������������λ
u8  g_LC_Step_Control=FALSE;		//��ʶ�Ƿ���lc���������Զ���������
u8  g_LC_Load_Finish=FALSE;			//ȡ�����
u8  g_LC_Load_Permit=FALSE;			//����ȡ��
u8  g_LC_UnLoad_Finish=FALSE;		//ж�����
u8  g_LC_UnLoad_Permit=FALSE;	    //����ж��
u8  g_Read_LC_Load_Program=FALSE;
u8  g_Read_LC_UnLoad_Program=FALSE;
u8  g_LC_PresentLine=0;				//�ϲֵ�ǰ��
u8  g_Start_Action_PresentLine=0;	//�������ζ�����ǰ��
u8  g_Start_Step_Control=FALSE;		//��ʶ�Ƿ����������������Զ���������
u8  g_Reset_Action_PresentLine=0;	//��λ������ǰ��
u8  g_Reset_Step_Control=FALSE;		//��ʶ�Ƿ��Ǹ�λ���������Զ���������

u8  g_Auto_Order_Start = FALSE;		//�Զ���������ָ��
u8  g_Auto_Order_Pause = FALSE;		//�Զ�������ָͣ��
u8  g_Auto_Order_Stop  = FALSE;		//�Զ�����ָֹͣ��
u8  g_Auto_Order_StackReset = FALSE;//��⸴λָ��

/***************************���������ڲ�����**************************************************/
u32 g_Auto_ActionRun_Timer = 0;
u8  g_Auto_ActionRun_Step = 0;
u32	g_Auto_ActionTimeOut_Time = 1000; //10msΪ��λ		 20sû��⵽ȷ���źţ�����Ϊ��ʱ
u8 	g_Auto_ActionTimeOut_Flag = FALSE;//������ⳬʱ
u8 	g_Auto_ActionError_Flag = FALSE;  //�Զ����н��뵽����״̬AUTO_ERROR
u8 	g_Auto_WorkFinished_Flag = FALSE; //��ǰ�������������
u8  g_Material_Load_Disk_Finished    = FALSE; //��ǰ���̼ӹ����
u8  g_Material_UnLoad_Disk_Finished  = FALSE; //��ǰ���̼ӹ����

ActionStepList Action_Step_List[LISTNUM]={0}; //����ָ��
u8 	Action_Step_List_Num=0;					  //��������ָ����
u8 	Action_Step_Run_Num=0;					  //�Ѿ����е�ָ������
u8 	Action_Step_Confirm_Num=0;				  //ȷ�ϵ�ָ������
u8  g_Material_Disk_Mode = FALSE;	          //����ģʽ
u8  g_Read_Load_LC_Position = FALSE;		  //�����ϲ�λ�ñ�־λ
u8  g_Read_UnLoad_LC_Position = FALSE;		  //�����ϲ�λ�ñ�־λ
u8  g_LC_ActionRun_Step = 0;				  //�ϲֶ�������
u32 g_LC_ActionRun_Timer = 0;
u32	g_LC_ActionTimeOut_Time = 1000; //10msΪ��λ		 20sû��⵽ȷ���źţ�����Ϊ��ʱ
//u8 	g_LC_ActionTimeOut_Flag = FALSE;//������ⳬʱ
u8  g_Start_ActionRun_Step = 0;				  //�������ζ�������
u8  g_Reset_ActionRun_Step = 0;				  //�������ζ�������

u16 g_Load_LC_Key=0;				  //�Ƿ�������ָ��
u32 g_Load_LC_Present_Position_X=0;	  //�����ϲֳ�ʼλ��X
u32 g_Load_LC_Present_Position_L=0;	  //�����ϲֳ�ʼλ��L
u8  g_Load_LC_Position_XKey=0;		  //LCλ��ָ������X
u8  g_Load_LC_Position_LKey=0;	      //LCλ��ָ������L

u16 g_UnLoad_LC_Key=0;				  //�Ƿ���ж��ָ��
u32 g_UnLoad_LC_Present_Position_X=0; //�����ϲֳ�ʼλ��X
u32 g_UnLoad_LC_Present_Position_L=0; //�����ϲֳ�ʼλ��L
u8  g_UnLoad_LC_Position_XKey=0;	  //LCλ��ָ������X
u8  g_UnLoad_LC_Position_LKey=0;	  //LCλ��ָ������L

void ActionStepControl(void);
void ActionOverOperate(void);
void ErrorJudgeWhile(void);
//u8 ActionAllowJudge(SaveProgram,u8);
void AutoPauseOperate(void);
void AutoStopOperate(void);
//void AutoRunning2(void);
void StartActionControl(void);




//-------------------------------------------//
u32 HT_Height=0;//��̧�߶�
u32 LC_Height=0;//�ϲָ߶�
u32 Glue_Time=0;//������
u32 Delay_Time=0;//ȡ������ʱ
u8 HT_Flag=0,LC_Empty_Flag=0;//Z���Ƿ���Ҫ��̧
u8 LC_Count_X1[10],LC_Count_Y1[10];
u8 LC_Pause_Correct=0;//��ͣ�ϲֵ�����
u8 DJ_Over=0,DZ_Over=0;
u16 Confirm_Order_Num=0;//�㽺���ǵ���ģʽ
u16 X_Speed =0,Z_Speed=0,Y_Speed=0,O_Speed=0,A_Speed=0,B_Speed=0;


extern u8 Order_Read_Mode;
extern u16 ShuZu_Count;
extern u8 LC_ID;
extern u16 Read_Point_Num;
extern u32 g_DianJiao_Delay_Timer;
extern u8  g_DianJiao_Delay_Step;
extern u8 Action_Flag;
extern u8  g_LCDelay_Step;
extern u32 g_LCDelay_Timer;
/**************************************************************************************************
**  ��������  AutoModeControl()
**	�����������
**	�����������
**	�������ܣ��Զ�ģʽ���ƺ���
**	��ע��	  �����豸�Զ�����
**  ���ߣ�    �ܺ���    
**  �������ڣ�2015/10/21
***************************************************************************************************/
void AutoModeControl(void)
{
	switch(g_AutoStatue)
	{
		case AUTO_WAITE:{//�ȴ�״̬�����������
			 if(g_Auto_Order_Start == TRUE)		//���յ������Զ�����ָ��
			 {
				g_Auto_Order_Start = FALSE;		//��λ��־λ
				g_AutoStatue = AUTO_RUNNING;	//�����Զ�����״̬
				g_Auto_Circle_Timer = 0;
			 }
//			if(g_Auto_Order_StackReset == TRUE)	//���¸�λ������ָ��
//			{
//				g_Auto_Order_StackReset = FALSE;//��λ��־λ���������������������
//			}
		 }break; 
		case AUTO_RUNNING:{//�Զ�������ز���
			 if(g_Auto_Order_Pause == TRUE)		//���������Զ�����ָ��
			 {
				g_Auto_Order_Pause = FALSE;		//��λ��־λ
				g_AutoStatue = AUTO_PAUSE;		//�ص��Զ�����״̬
				AutoPauseOperate();				//��ͣ������ز���			
			 }
			 if(g_Auto_Order_Stop == TRUE)		//�Զ�����ָֹͣ��
			 {
				g_Auto_Order_Stop = FALSE;		//��λ��־λ
				g_AutoStatue = AUTO_WAITE;		//�ص��Զ��ȴ�״̬	
				g_Auto_PresentLine=0;
				Action_Step_Run_Num=0;
				AutoPauseOperate();				//��ͣ������ز���
				AutoStopOperate();				//ֹͣ������ز���
				Read_Point_Num=0;
				Order_Read_Mode=0;
				ShuZu_Count=0;
				Order_Package();	
			 }
			 if(g_AutoStatue == AUTO_RUNNING)
			 {
				 g_LC_Step_Control=FALSE;
				 g_Start_Step_Control=FALSE;
				 g_Reset_Step_Control=FALSE;
				 ActionStepControl();//���ж���������ƣ���Free_Program_Operate1�ṹ����п��ƣ�
				 //�������ζ���
				 if(g_Program_Start_Action==TRUE && g_Program_Start_Action_Done==FALSE)//�������ζ���
				 {
				   g_Start_Step_Control=TRUE;
				 }
				 //�ѵ���λ����
				 if(g_Program_Reset_Action==TRUE && g_Program_Reset_Action_Done==FALSE)//�ѵ���λ����
				 {
				     g_Reset_Step_Control=TRUE;
				 }
				 //�ϲֶ���
				 if(g_LC_Program_Step_Run==TRUE)  //�ϲֳ���
				 {
					 g_LC_Step_Control=TRUE;
				 }
			 }
		 }break;
		case AUTO_PAUSE:{ //��ͣת״̬����Ƿ�������ֹͣ
			 if(g_Auto_Order_Start == TRUE)		//���������Զ�����ָ��
			 {
					g_Auto_Order_Start = FALSE;		//��λ��־λ
					g_AutoStatue = AUTO_RUNNING;	//�ص��Զ�����״̬
			 }
			 if(g_Auto_Order_Stop == TRUE)		//�Զ�����ָֹͣ��
			 {
					g_Auto_Order_Stop = FALSE;		//��λ��־λ
					g_AutoStatue = AUTO_WAITE;		//�ص��Զ��ȴ�״̬
					AutoStopOperate();				//ֹͣ������ز���
			 }
		 }break;
		case AUTO_ERROR:{//�����⴦��
			 g_Auto_ActionError_Flag=TRUE;
			 g_Auto_Order_Stop = TRUE;
			 if(g_Auto_Order_Stop == TRUE)		//�Զ�����ָֹͣ��
			 {
					//g_Auto_ActionError_Flag=FALSE;
					g_Auto_Order_Stop = FALSE;		//��λ��־λ
					g_AutoStatue = AUTO_WAITE;		//�ص��Զ��ȴ�״̬
					AutoPauseOperate();
					AutoStopOperate();				//ֹͣ������ز���
			 }
		 }break;
		default:{
		}break;
	}		
}



/**************************************************************************************************
**  ��������  Speed_Compensate()
**	�����������
**	�����������
**	�������ܣ�ͨ��ʱ��ʵ�ָ���ͬ����ָ��Ԥ����
**	��ע��	  
**  ���ߣ�    White_L    
**  �������ڣ�2018/7/1
***************************************************************************************************/
void Speed_Compensate(u16 AutoPresentLine)
{
	u32 Axis_Distance[6]={0},Max_distance=0;
	u8 i=0;
	float Time1_Dis=0.0;

	if(AutoPresentLine<(Free_Program_Operate1.Num))
	{
		LC_ID      = Free_Program_Operate1.Program[ShuZu_Count].LC_ID;
		HT_Height  = LC_Parameter_Load[LC_ID-1].HT_Hight;
		LC_Height  = (LC_Parameter_Load[LC_ID-1].LC_Hight&0x0fffffff)+MINROBOTPOSITION;
		Glue_Time  = LC_Parameter_Load[LC_ID-1].Glue_Time;
		Delay_Time = LC_Parameter_Load[LC_ID-1].Diamonds_Time;	
		/*****************�ϲּ�⣬���������Ͽտ��Ƶ��̶�λ����ʾ����********************/
		if(Free_Program_Operate1.Program[ShuZu_Count].Key==3&&LC_Pause_Correct==0)
		{
			Free_Program_Operate1.Program[ShuZu_Count].Value1 = (LC_Load_Table[LC_ID-1].LC_Load_Position[LC_Count_X1[LC_ID-1]][LC_Count_Y1[LC_ID-1]].XPoint& 0x0fffffff)+MINROBOTPOSITION;
			Free_Program_Operate1.Program[ShuZu_Count].Value2 = (LC_Load_Table[LC_ID-1].LC_Load_Position[LC_Count_X1[LC_ID-1]][LC_Count_Y1[LC_ID-1]].YPoint& 0x0fffffff)+MINROBOTPOSITION;
			Free_Program_Operate1.Program[ShuZu_Count].Value3 = LC_Height;
			Free_Program_Operate1.Program[ShuZu_Count].Value4 = MINROBOTPOSITION;	
			Free_Program_Operate1.Program[ShuZu_Count].Value5 = MINROBOTPOSITION;
			 if(LC_Count_Y1[LC_ID-1]<LC_Parameter_Load[LC_ID-1].V_Num-1)
				 LC_Count_Y1[LC_ID-1]++;
			 else if(LC_Count_Y1[LC_ID-1]>=(LC_Parameter_Load[LC_ID-1].V_Num-1)&&LC_Count_X1[LC_ID-1]<=LC_Parameter_Load[LC_ID-1].H_Num-1)
			 {
				 LC_Count_X1[LC_ID-1]++;
				 LC_Count_Y1[LC_ID-1]=0;
			 }
			 else if(LC_Count_X1[LC_ID-1]>=LC_Parameter_Load[LC_ID-1].H_Num-1)//����ȡ�������ͣ״̬
			 {
				 LC_Empty_Flag=1;
				 LC_Count_X1[LC_ID-1]=0;
				 LC_Count_Y1[LC_ID-1]=0;					 
			 }
		}
		if(Free_Program_Operate1.Program[ShuZu_Count].Value1>Pulse_TotalCounter[X_Axsis])
			Axis_Distance[X_Axsis]=Free_Program_Operate1.Program[ShuZu_Count].Value1-Pulse_TotalCounter[X_Axsis];
		else
			Axis_Distance[X_Axsis]=Pulse_TotalCounter[X_Axsis]-Free_Program_Operate1.Program[ShuZu_Count].Value1;
		
		if(Free_Program_Operate1.Program[ShuZu_Count].Value2>Pulse_TotalCounter[L_Axsis])
			Axis_Distance[L_Axsis]=Free_Program_Operate1.Program[ShuZu_Count].Value2-Pulse_TotalCounter[L_Axsis];
		else
			Axis_Distance[L_Axsis]=Pulse_TotalCounter[L_Axsis]-Free_Program_Operate1.Program[ShuZu_Count].Value2;
		///------------------------------ֻҪZ���л�̧�ռ�ͻ�̧����-------------------------------//
		if((Pulse_TotalCounter[Z_Axsis]-MINROBOTPOSITION>HT_Height)&&(Pulse_TotalCounter[Z_Axsis]-HT_Height)<Free_Program_Operate1.Program[ShuZu_Count].Value3)
		{
			HT_Flag=1;
			Axis_Distance[Z_Axsis] = Pulse_TotalCounter[Z_Axsis]-HT_Height;
			if(Axis_Distance[Z_Axsis]>Free_Program_Operate1.Program[ShuZu_Count].Value3)
				Axis_Distance[Z_Axsis] = Axis_Distance[Z_Axsis]-Free_Program_Operate1.Program[ShuZu_Count].Value3+HT_Height;
			else
				Axis_Distance[Z_Axsis]=Free_Program_Operate1.Program[ShuZu_Count].Value3-Pulse_TotalCounter[Z_Axsis]+HT_Height;//ȡ��ÿ����ÿ�ζ���������Ҫ�ߵ�·��
		}
		else
		{
			HT_Flag=0;
			if(Free_Program_Operate1.Program[ShuZu_Count].Value3>Pulse_TotalCounter[Z_Axsis])
				Axis_Distance[Z_Axsis] = Free_Program_Operate1.Program[ShuZu_Count].Value3-Pulse_TotalCounter[Z_Axsis];
			else
				Axis_Distance[Z_Axsis]=Pulse_TotalCounter[Z_Axsis]-Free_Program_Operate1.Program[ShuZu_Count].Value3;
		}
		
			
		if(Axis_Distance[X_Axsis]>=Axis_Distance[L_Axsis]&&Axis_Distance[X_Axsis]>=Axis_Distance[Z_Axsis])
		{
			X_Speed = JXS_Parameter.AutoSpeed_H[0]*100;
			Time1_Dis = Axis_Distance[X_Axsis]*1.0/JXS_Parameter.AutoSpeed_H[0]*1.0;
			Y_Speed = (Axis_Distance[L_Axsis]*1.0/Time1_Dis*1.0)*100;
			Z_Speed = (Axis_Distance[Z_Axsis]*1.0/Time1_Dis*1.0)*100;
		}
		else if(Axis_Distance[L_Axsis]>=Axis_Distance[X_Axsis]&&Axis_Distance[L_Axsis]>=Axis_Distance[Z_Axsis])
		{
			Y_Speed = JXS_Parameter.AutoSpeed_H[0]*100;
			Time1_Dis = Axis_Distance[L_Axsis]*1.0/JXS_Parameter.AutoSpeed_H[0]*1.0;
			X_Speed = (Axis_Distance[X_Axsis]*1.0/Time1_Dis*1.0)*100;
			Z_Speed = (Axis_Distance[Z_Axsis]*1.0/Time1_Dis*1.0)*100;		
		}
		else if(Axis_Distance[Z_Axsis]>=Axis_Distance[X_Axsis]&&Axis_Distance[Z_Axsis]>=Axis_Distance[L_Axsis])
		{
			Z_Speed = JXS_Parameter.AutoSpeed_H[0]*100;
			Time1_Dis =  Axis_Distance[Z_Axsis]*1.0/JXS_Parameter.AutoSpeed_H[0]*1.0;
			X_Speed =  (Axis_Distance[X_Axsis]*1.0/Time1_Dis*1.0)*100;
			Y_Speed =  (Axis_Distance[L_Axsis]*1.0/Time1_Dis*1.0)*100;
		}
		A_Speed = JXS_Parameter.AutoSpeed_H[1]*100;
		B_Speed = JXS_Parameter.AutoSpeed_H[2]*100;
   }
}


/**************************************************************************************************
**  ��������  ActionStepControl()
**	�����������
**	�����������
**	�������ܣ����ж���������ƺ���
**	��ע��	  ����ÿ�г��򵥶�����
**  ���ߣ�    �ܺ���    
**  �������ڣ�2015/10/21
***************************************************************************************************/
void ActionStepControl(void)
{
	u8 temp1[10];//��ǰ��Ҫ�����к�
	u8 ActionSetResult = 0;//����ִ��״̬�Ĵ����
	u16 AutoPresentLine=0;
	u8 i=0;
	switch(g_Auto_ActionRun_Step)//����Stepȷ����Ӧ��ִ�л���
	{
		case 0:{		//�����������
			//ִ��ÿ�����������������ݶ���ִ��������أ�����ִ������붯��ȷ�ϻ��ڣ�
			//ִ�к�����Ҫ��g_Auto_ActionTimeOut_Time�������ö�Ӧֵ������Ϊ0�����ڶ���ִ�г�ʱ�ж�
			//ͳ�Ƶ�ǰһ��ִ��ָ����
			if(Auto_Mode == SINGLE_MODE)
			{
			    Action_Step_List_Num=1;//��������ָ��=1
			}
			else if(Action_Step_List_Num==0)// && Free_Program_Operate1.Program[0].Key==K_PROGRAMSTART)//�ǵ���ģʽ��û�й���������Ϣ��ʱ��
			{
//				Action_Step_List_Num=AutoActionStepList(g_Auto_PresentLine);	  //��ȡ�����е�����
				Action_Step_List_Num=1;
				Action_Step_Run_Num=0;											  //�Ѿ�����������0
			}
			if(g_Auto_PresentLine==Free_Program_Operate1.Num)
				g_Auto_PresentLine=0;
			AutoPresentLine = g_Auto_PresentLine+Action_Step_Run_Num;						//��ǰ��Ҫ�����к�=�Զ����е��к�+����������
			Speed_Compensate(AutoPresentLine);
			ActionSetResult = AutoActionOutControl(AutoPresentLine);	//��ȡִ�����
			switch(ActionSetResult)//��Բ�ͬ������д���
			{				
				case 0:		//����������ִ��
					 g_Auto_ActionRun_Step = 1;
					 Action_Step_List_Num=0;
					 Action_Step_Confirm_Num=0;
					 break;
				case 9:		//����ѭ������	  
					 ActionOverOperate();	 //g_Auto_ActionRun_Step = 3;
					 break;
				case 10:	//��ǰ�кų����쳣
					 g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨c����־λ
					 break;
				case 11:	//��Ҫָ�������쳣
					 g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨����־λ
					 break;
				case 12:	//����ָ�������쳣
					 g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨����־λ
					 break;
				case 13:	//���ָ�������쳣
					 g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨����־λ
					 break;
				case 14:	//IO����ָ�������쳣
					 g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨����־λ
					 break;
				case 15:
				     break;
				default:
					 break;
			}
		}break;
		case 1:{		//����ȷ�ϻ���			
			if(AutoActionOutConfirm(Free_Program_Operate1,ShuZu_Count) == TRUE)//�ж϶���ȷ�����
			{
				if(LC_Empty_Flag == 0)
				{
					Confirm_Order_Num++;//��¼�˶�ָ����Ŀ				
					Action_Step_Confirm_Num++;
					if(Action_Step_Confirm_Num==Action_Step_Run_Num)
					{
						g_Auto_ActionRun_Step = 2;
						g_Auto_ActionRun_Timer=0;
						Action_Step_Confirm_Num=0;
					}
				}
			}
			else//����ȷ�ϲ��ɹ�
			{
				if(g_Auto_ActionRun_Timer > g_Auto_ActionTimeOut_Time)	//����ִ�г�ʱ�ж�
				{
					if(Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Order<=OR_AXISORDER)
					{
					}
					else
					{
						g_Auto_ActionTimeOut_Flag=TRUE;
						g_Auto_Order_Pause = TRUE;
						if(g_Program_Is_Debuging)
						{
							g_Auto_Order_Stop = TRUE;
						}
					}
					g_Auto_ActionRun_Timer=0;
				}
			}
		}break;
		case 2:{		//������ʱ����
			 if(TRUE)//AutoActionOutDelay(Free_Program_Operate1,g_Auto_PresentLine+(Action_Step_Run_Num-1)) == TRUE
			 {
					Order_Package();				
					
					if(Order_Read_Mode==0)
					{
						ShuZu_Count++;	
						if(ShuZu_Count==Free_Program_Operate1.Num)
							ShuZu_Count=0;
					}
				 //��һ���㽺ָ��
					if(Free_Program_Operate1.Program[ShuZu_Count].Key==1)
					{
						 for(i=13;i<21;i++)		//ָʾ�Ʋ���λ
						 {
							SetOutput(i);
						 }	
					}
				 //����ָ��
					if(Free_Program_Operate1.Program[ShuZu_Count].Key==2||Free_Program_Operate1.Program[ShuZu_Count].Key==3)
					{
						 for(i=4;i<7;i++)		//ָʾ�Ʋ���λ
						 {
							SetOutput(i);
						 }	
					}
					if(LC_Empty_Flag == 0)
					{
						if(Confirm_Order_Num==Free_Program_Operate1.Num) 
						{
							ActionOverOperate();//һ��ָ�������
						}
						else
							g_Auto_ActionRun_Step = 0;
						Action_Step_List_Num=0;
						g_Auto_PresentLine=g_Auto_PresentLine+Action_Step_Run_Num;
						if(Auto_Mode == SINGLE_MODE) //����ģʽ-������ͣ״̬
						{
							g_Auto_Order_Pause = TRUE;	
						}
						if(g_Program_Is_Debuging)
						{
							g_Auto_Order_Stop = TRUE;
						}
					}
					else
					{
						g_Auto_Order_Pause = TRUE;
						temp1[0] =0xCC;
						USART1_SendData(1,0xCC,&temp1[0]);//��ͣ��������
						LC_Empty_Flag=0;
					}
		  }
		 }break;
		case 3:{		
			/***һ�׶�������****/
			 Confirm_Order_Num=0;
//			 Order_Package();		
			 g_Auto_ActionRun_Step=0;
			 Action_Step_List_Num=0;
			LC_Pause_Correct=0;
			 if(Auto_Mode == SINGLE_MODE||(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)) //����ģʽ-������ͣ״̬
			 {
				g_Auto_Order_Pause = TRUE;	
				 for(i=0;i<35;i++)		//ָʾ�Ʋ���λ
				 {
					SetOutput(i);
				 }	
				DJ_Over=0;
				DZ_Over=0;
			 }
			 if(g_Program_Is_Debuging)
			 {
				g_Auto_Order_Stop = TRUE;
				//g_Program_Is_Debuging=FALSE;
			 }
			 if(SC_Parameter.SC_Num==SC_Parameter.RW_Num)
			 {
			 	g_Auto_Order_Pause = TRUE;
				g_Auto_WorkFinished_Flag=TRUE;
			 }
		 }break;
		default:
			break;
	}
}


/**************************************************************************************************
**  ��������  ActionOverLCCalculate()
**	���������
**	���������
**	�������ܣ�
**	��ע��	  
**  ���ߣ�    wx   
**  �������ڣ�2015/10/21
***************************************************************************************************/
void ActionOverLCCalculate(void)
{
//	if(g_Load_LC_Key)
//	{
//		if(SC_Parameter.SC_Num%(LC_Parameter_Load.Layer*LC_Parameter_Load.H_Num*LC_Parameter_Load.V_Num)==0)	 //һ�����̼ӹ���� ���뱨������ͣ��״̬
//		{
//			g_Material_Load_Disk_Finished=TRUE;
////			Free_Program_Operate.Program[g_Load_LC_Position_XKey].Value1=g_Load_LC_Present_Position_X;
////			Free_Program_Operate.Program[g_Load_LC_Position_LKey].Value1=g_Load_LC_Present_Position_L;
//			//g_Read_Load_LC_Position=FALSE;
//		}
//		else
//		{
//			g_Material_Load_Disk_Finished=FALSE;
//		}	
//	}
//	if(g_UnLoad_LC_Key)
//	{
//		if(SC_Parameter.SC_Num%(LC_Parameter_UnLoad.Layer*LC_Parameter_UnLoad.H_Num*LC_Parameter_UnLoad.V_Num)==0)	 //һ�����̼ӹ���� ���뱨������ͣ��״̬
//		{
//			g_Material_UnLoad_Disk_Finished=TRUE;
////			Free_Program_Operate.Program[g_UnLoad_LC_Position_XKey].Value1=g_UnLoad_LC_Present_Position_X;
////			Free_Program_Operate.Program[g_UnLoad_LC_Position_LKey].Value1=g_UnLoad_LC_Present_Position_L;
//			//g_Read_UnLoad_LC_Position=FALSE;
//		}
//		else
//		{
//			g_Material_UnLoad_Disk_Finished=FALSE;
//		}	
//	}
}

/**************************************************************************************************
**  ��������  ActionOverLCCalculate()
**	�����������
**	�����������
**	�������ܣ�ѭ����������������
**	��ע��	  һ�����ڽ��������ݻָ�����
**  ���ߣ�    �ܺ���    
**  �������ڣ�2015/10/21
***************************************************************************************************/
void ActionOverOperate(void)
{
	u8 Temp_data[8]={0};
	g_Auto_PresentLine = 0;
	Action_Step_List_Num=0;
	Action_Step_Run_Num=0;
	Action_Step_Confirm_Num=0;
	g_Auto_LastCircleTime = g_Auto_Circle_Timer;
	g_Auto_Circle_Timer=0;
//	g_Auto_ProduceCounter ++;
	//дIIC,�ۼƲ����Լ���������
	if((g_Program_Is_Debuging==FALSE) && (g_Run_Program_Num<=8))
	{
		SC_Parameter.SC_Num++;
		SC_Parameter.LJ_Num++;	
		if(SC_Parameter.LJ_Num>MINROBOTPOSITION)
		{
			SC_Parameter.LJ_Num=0;
		}
		Temp_data[0] = SC_Parameter.SC_Num;
		Temp_data[1] = SC_Parameter.SC_Num>>8;
		Temp_data[2] = SC_Parameter.SC_Num>>16;
		Temp_data[3] = SC_Parameter.SC_Num>>24;
		Temp_data[4] = SC_Parameter.LJ_Num;
		Temp_data[5] = SC_Parameter.LJ_Num>>8;
		Temp_data[6] = SC_Parameter.LJ_Num>>16;
		Temp_data[7] = SC_Parameter.LJ_Num>>24;
		W25QXX_Write(Temp_data,0x1090+0x0C,8);
		//�������
		//�ϲֲ������㴦��
		//ActionOverLCCalculate();
	}
	g_Auto_ActionRun_Step = 3;
}

/**************************************************************************************************
**  ��������  ErrorJudgeWhile()
**	�����������
**	�����������
**	�������ܣ�ȫ���쳣��麯��
**	��ע��	  �ڳ������й����У�ʵʱ����쳣�����
**            ����������̱���
**  ���ߣ�    �ܺ���    
**  �������ڣ�2015/10/21
***************************************************************************************************/
void ErrorJudgeWhile(void)
{
} 

/**************************************************************************************************
**  ��������  AutoPauseOperate()
**	�����������
**	�����������
**	�������ܣ��Զ���ͣ����
**	��ע��	  ��ͣ�������е�ȫ�Զ�������������ر�־λ
**  ���ߣ�    �ܺ���    
**  �������ڣ�2015/10/21
***************************************************************************************************/
void AutoPauseOperate(void)
{	
//    switch(g_Auto_ActionRun_Step)
//	{

//		case 0:		//�����������
//			 Action_Step_List_Num=0;		 
//			 break;
//		case 1:		//����ȷ�ϻ���	
			 if(Timer_Enable_X==ENABLE)		//������ڷ�������
			 {
				ServoStop_CAN(X_Axsis);
				  CAN_Servo_Stop_Action[X_Axsis]=FALSE;
			 }					 
			 if(Timer_Enable_Z==ENABLE)		//������ڷ�������
			 {
			  	ServoStop_CAN(Z_Axsis);
				  CAN_Servo_Stop_Action[Z_Axsis]=FALSE;
			 }
			 if(Timer_Enable_L==ENABLE)		//������ڷ�������
			 {
			  	ServoStop_CAN(L_Axsis);
				  CAN_Servo_Stop_Action[L_Axsis]=FALSE;
			 }
			 if(Timer_Enable_O==ENABLE)		//������ڷ�������
			 {
			  	ServoStop_CAN(O_Axsis);
				  CAN_Servo_Stop_Action[O_Axsis]=FALSE;
			 }	 
			 if(Timer_Enable_A==ENABLE)		//������ڷ�������
			 {
			  	ServoStop_CAN(A_Axsis);
				  CAN_Servo_Stop_Action[A_Axsis]=FALSE;
			 }
			 if(Timer_Enable_B==ENABLE)		//������ڷ�������
			 {
			  	ServoStop_CAN(B_Axsis);
				  CAN_Servo_Stop_Action[B_Axsis]=FALSE;
			 }				 
			 Action_Step_List_Num=0;
			 Action_Step_Run_Num=0;
			 Action_Step_Confirm_Num=0;
			 g_Auto_ActionRun_Step = 0;
			 g_LC_ActionRun_Step = 0;
			 g_Reset_ActionRun_Step = 0;

			 Action_Flag=0;
			 g_LCDelay_Step=0;
			 g_LCDelay_Timer=0;
			 g_DianJiao_Delay_Step=0;
			 g_DianJiao_Delay_Timer=0;			 
//			 break;
//
//		case 2:		//������ʱ����
////			 g_Auto_ActionRun_Step = 0;
////			 g_Auto_PresentLine ++;
//			 break;
//		default:
//			break;
//	}
}

/**************************************************************************************************
**  ��������  AutoStopOperate()
**	�����������
**	�����������
**	�������ܣ�����ִ��ǰ���Ϸ����ж�
**	��ע��	  ֹͣȫ�Զ����ж���������������λ��־λ
**  ���ߣ�    �ܺ���    
**  �������ڣ�2015/10/21
***************************************************************************************************/
void AutoStopOperate(void)
{
	if(g_Program_Is_Debuging)
	{
	  g_Program_Is_Debuging=FALSE;
		g_AutoStatue = AUTO_WAITE;
	}
	else
	{
		g_AutoStatue = AUTO_WAITE;
		g_Auto_Order_Start = FALSE;
		g_Auto_Order_Pause = FALSE;
		g_Auto_Order_Stop = FALSE;
		g_Auto_PresentLine = 0;
		g_Auto_LastCircleTime = 0;
		g_Auto_Circle_Timer = 0;
		Single_Mode_Enable = DISABLE;
		Loop_Mode_Enable = DISABLE;
		g_Auto_ActionTimeOut_Flag=FALSE;
		Puls_Delay_Num=0;
		Action_Step_List_Num=0;
		Action_Step_Run_Num=0;
		Action_Step_Confirm_Num=0;
		g_Read_LC_Load_Program=FALSE;
		g_Read_LC_UnLoad_Program=FALSE;
		g_LC_PresentLine = 0;
		g_LC_Step_Control=FALSE;
		g_LC_Program_Step_Run=FALSE;
		g_LC_Load_Permit=FALSE;
		g_LC_Load_Finish=FALSE;
		g_LC_UnLoad_Permit=FALSE;
		g_LC_UnLoad_Finish=FALSE;
		g_Program_Start_Action=FALSE;
		g_Start_Step_Control=FALSE;
		g_Program_Reset_Action=FALSE;
		g_Reset_Step_Control=FALSE;
	}
}

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/
