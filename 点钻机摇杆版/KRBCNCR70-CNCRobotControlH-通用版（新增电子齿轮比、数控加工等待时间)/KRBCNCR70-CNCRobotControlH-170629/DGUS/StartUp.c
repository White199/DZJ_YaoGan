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
#include "key.h"

u8  Landing_Success = FALSE;		   //�����½��ʶλ
u8  Change_User = FALSE;			     //�л��û�
u8  PassWord_Input_Done = FALSE;
u32 Pass_Word_Temp = 0;		         //��������뻺��

/*----------------Start_Up--����1������־λ----------------*/
//static u8 gs_Start_Up_Page = 0;	             //��һ��ҳ���µ�ҳ����
static u8 gs_Welcom_Flag = 1;	                 //��ʾ��ӭ-��ʼ��ʾ��¼
//static u8 gs_Current_Alarm = 1;	             //������ʾ
static u8 gs_Robot_Enable_Flag = 0;	           //ʹ��
static u8 gs_Emergency_Stop_Flag = 0;	         //��ͣ
static u32 gs_SC_Parameter_SC_Num = 0;		     //�ӹ�����
static u8 gs_Cycle_Run_Flag = 0;	 		         //0:ѭ��  1������
//static u8 gs_X_Position_Flag = 0;	 		       //Xλ��ˢ��
//static u8 gs_Z_Position_Flag = 0;	 		       //Zλ��ˢ��
//static u8 gs_Robot_Origin_Flag = 0;	 		     //����
static u8 gs_Auto_Status = AUTO_ERROR;
//static u8 gs_Robot_Run_Flag = 0;			       //����
//static u8 gs_Robot_Pause_Flag = 0;			     //��ͣ
//static u8 gs_Robot_Stop_Flag = 0;			       //ֹͣ
 u8 gs_Program_Refresh_Flag = FALSE;	   //��ʾ����ˢ��

/*-----------����ͼ�궨��--------------*/
//����
//static PageCut PageCut_Origin_Y={0x1180,74,509,181,589,246,509,181};	 //����ͼ�������Ч��
//static PageCut PageCut_Origin_N={0x1180,1 ,509,181,589,246,509,181};	 //����ͼ�������Ч��
//��������ʾ
//static PageCut PageCut_Origin_Remind={0x1190,38,209,106,614,349,209,106};//��������ʾ

//---ԭ���� ע�͵�
//ѭ��-����
// static PageCut PageCut_CyCle1={0x1130, 1,690,48 ,776,86 ,690,48 };	 //ѭ��ͼ�����
// static PageCut PageCut_CyCle2={0x1170, 1,643,341,732,417,643,341};	 //ѭ��ͼ�����
// static PageCut PageCut_Sigle1={0x1130,74,690,48 ,776,86 ,690,48 };	 //����ͼ�����---��Ϊ0x1130�ڵ�һҳ�����м���74ҳ����Ϊ690,48��776,86 ����һҳ����690��48
// static PageCut PageCut_Sigle2={0x1170,74,643,341,732,417,643,341};	 //����ͼ�����
// //ʹ��-ʧ��
// static PageCut PageCut_Enable ={0x1110,74,460,48 ,540,86 ,460,48 };	 //ʹ��ͼ�����
// static PageCut PageCut_Disable={0x1110, 1,460,48 ,540,86 ,460,48 };	 //ʧ��ͼ�����
// //��ͣ
// static PageCut PageCut_Emerg_Y={0x1120,74,580,48 ,666,86 ,580,48 };	 
// static PageCut PageCut_Emerg_N={0x1120, 1,580,48 ,666,86 ,580,48 };	 
// //����
// static PageCut PageCut_Run_Y={0x1140,74,508,255,589,336,508,255};	 
// static PageCut PageCut_Run_N={0x1140, 1,508,255,589,336,508,255};	
// //��ͣ
// static PageCut PageCut_Pause_Y={0x1150,74,644,254,733,335,644,254};	 
// static PageCut PageCut_Pause_N={0x1150, 1,644,254,733,335,644,254};
// //ֹͣ
// static PageCut PageCut_Stop_Y={0x1160,74,506,339,587,419,506,339};	 
// static PageCut PageCut_Stop_N={0x1160, 1,506,339,587,419,506,339};

//---�ĺ��
//ѭ��-����
//static PageCut PageCut_CyCle1={0x1130, 1,690,48 ,776,86 ,690,48 };	 //ѭ��ͼ�����
static PageCut PageCut_CyCle2={0x1170, 1,640,310,730,390,640,310};	   //ѭ��ͼ�����
//static PageCut PageCut_Sigle1={0x1130,74,690,48 ,776,86 ,690,48 };	 //����ͼ�����---��Ϊ0x1130�ڵ�һҳ�����м���74ҳ����Ϊ690,48��776,86 ����һҳ����690��48
static PageCut PageCut_Sigle2={0x1170,74,640,310,730,390,574,310};	   //����ͼ�����
//ʹ��-ʧ��
//static PageCut PageCut_Enable ={0x1110,74,460,48 ,540,86 ,460,48 };	   //ʹ��ͼ�����
//static PageCut PageCut_Disable={0x1110, 1,460,48 ,540,86 ,460,48 };	   //ʧ��ͼ�����
//��ͣ
static PageCut PageCut_Emerg_Y={0x1120,74,580,48 ,666,86 ,580,48 };	 
static PageCut PageCut_Emerg_N={0x1120, 1,580,48 ,666,86 ,580,48 };	 
//����
static PageCut PageCut_Run_Y={0x1140,74,478,193,571,281,467,210};	 
static PageCut PageCut_Run_N={0x1140, 1,490,200,580,280,490,200};	
//��ͣ
static PageCut PageCut_Pause_Y={0x1150,74,636,200,716,280,569,208};	 
static PageCut PageCut_Pause_N={0x1150, 1,640,200,730,280,640,200};
//ֹͣ
static PageCut PageCut_Stop_Y={0x1160,74,490,310,567,390,470,310};	 
static PageCut PageCut_Stop_N={0x1160, 1,490,310,580,390,490,310};


//ÿһ�еı���ɫ
static PageCut Run_Program_Color[9] = {
									  {0x1100,74,10 ,100,418,110,10 ,100}, //����ʾʱ��
									  {0x1100,74,10 ,250,418,277,10 ,151}, //��һ��
									  {0x1100,74,10 ,250,418,277,10 ,184}, //�ڶ���
									  {0x1100,74,10 ,250,418,277,10 ,217}, //������
									  {0x1100,74,10 ,250,418,277,10 ,250}, //������
									  {0x1100,74,10 ,250,418,277,10 ,283}, //������
									  {0x1100,74,10 ,250,418,277,10 ,316}, //������
									  {0x1100,74,10 ,250,418,277,10 ,349}, //������
									  {0x1100,74,10 ,250,418,277,10 ,382}, //�ڰ���	
									}; 

//-----------��ʾ�����ַ�����---������ʽ��Send_String_XX--------------//
u8 Send_String_HY[]={"��ӭ"};
u8 Send_String_DL[]={"��¼"};

static u32 gs_StartUp_XAxis_Position = 0;  //X��λ��
static u32 gs_StartUp_ZAxis_Position = 0;  //Z��λ��
static u32 gs_StartUp_LAxis_Position = 0;  //L��λ��
static u32 gs_StartUp_OAxis_Position = 0;  //O��λ��

FreeProgram Run_Program[LARGESTPROGRAMNUM]={0};            //ѡ�����еĳ���

u8 gs_Run_Program_List[9]={1,2,3,4,5,1,2,3};	                     //��ǰ��ʾ��8��ָ��
u8 gs_Run_Program_Color = 1;				                 //��ǰ����ɫ
u16 gs_Run_Program_Code[8]={0};	                     //��ǰ��ʾ��8������
static u32  gs_Run_Program_Parameter_Display[9][2]={0};		 //���������Ӧ����λ��������ʾ
       u8 gs_Run_Program_Num = 0;				                   //��ǰѡ�����г���ı��
static u16 gs_Run_Program_Operater_List = 0;		             //��ǰѡ�����г����������е�ָ�����������ذ巵��
// static u32 gs_Origin_Speed = 10;				                   //�����ٶ�
// static u8 gs_Refresh_Origin_Speed = TRUE;	               //�����ٶ�ˢ��
       u8 g_Auto_Status = AUTO_WAITE;
u32 gs_Run_Program_Name=0,g_Run_Program_Name=0;

u8  SuZhu_Count1=8;
u16 Read_Program_Count=0;//���������ȡ����ļ���
u16 ShuZu_Count=0;//ָ���������λ��
u8  Waite_Reflesh=0;//
extern u8 LCD_Control_Flag;
extern u16 E_GearRatio[6];
extern u8 LC_Empty_Flag;
extern u8 Product_Num;
extern u8 LC_ID,Display_Flag;
extern u8 LCD_Main_Page_Pre;
/*************************************************************************
**  ��������  LCDStartUpProcess()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/5/28
**************************************************************************/
void LCDStartUpProcess(void)
{
	 //������ǵ�ǰҳ�棬���л�����ǰ����
	if( (LCD_Var_Address == 0x1000) && (LCD_Main_Page != START_UP) )
	{
		LCD_Main_Page_Pre = LCD_Main_Page;
		LCD_Main_Page = START_UP;                         //LCD��ҳ��Ϊ��������
		LCD_Current_Page = 	LCD_Main_Page;                //LCD��ǰҳ��Ϊ��������
		g_Watch_IO_Control = 0; 	                        //ʵʱIO����״̬
	  Order_Temp[0]=AUTO_MODE;                          //����Ϊ�Զ�ģʽ
	  USART1_SendData(1,WORK_MODE,&Order_Temp[0]);      //���ڷ��͹���ģʽ�е��Զ�ģʽ�����ذ�
//		g_Cycle_Run_Flag = 1;
//		Robot_Auto_Mode = CYCLE_MODE;							  
	}
	if(Robot_Origined == TRUE)                          //�����е�ֻ�ԭ��
	{
		//LCDStartUpDisply();	                              //�����������ʾ	
		switch(LCD_Var_Address)
		{
			case 0x1007:  				                          //�������
				 if(Robot_Status == ACTION_STOP)              //�����е��״̬Ϊֹͣ
				 {
					   Order_Temp[0]=ROBOT_ORIGIN;              //���û�е�ֻ�ԭ��
					   USART1_SendData(1,ROBOT_ENABLE_A_ORIGIN,&Order_Temp[0]);  //���ڷ��ͻ�е��ʹ�ܼ���ԭ���еĻ�е�ֻ�ԭ�㵽���ذ�
					   Robot_Origined = FALSE;                  //��е�ֻ�ԭ��
					   g_Robot_Stop_Flag = 0;                   //��е��ֹͣ
					   g_Run_Program_Operater_List = 0;         //��ǰѡ�����г����������е�ָ�����������ذ巵��             			   				   
				 }
				 else
				 {
				   	 LCD_Popup_Windows = PW_1_STOPROBOT;      //������ʾ��-����ֹͣ����
				 }
				 break;

			case 0x1009: 								 	                  //����
	       if(gs_Run_Program_Num)				 
			   {
				   	 if(g_Robot_Auto_Reset == FALSE)
					   {
					   	LCD_Current_Page = 88;
							Robot_Status = ACTION_RESET;            //��λ״̬
					   }
					   else
					   {
						   if(Robot_Status != ACTION_RUN)
						   {
							   Order_Temp[0]=ACTION_RUN;            //���û�е������
							   USART1_SendData(1,AUTO_RUN,&Order_Temp[0]);//���ڷ��ͻ�е���Զ������е����������ذ�
						   }
					   }				   	   	   
			     }
			     else
			     {
			   		  LCD_Popup_Windows = PW_1_CHOOSEPROGRAM;	//������ʾ��-���ȼ��س���		
			     }
			     break;	

			case 0x100A: 				                            //��ͣ
					   if(Robot_Status == ACTION_RUN)
					   {
						   Order_Temp[0]=ACTION_PAUSE;            //���û�е����ͣ
						   USART1_SendData(1,AUTO_RUN,&Order_Temp[0]);//���ڷ��ͻ�е���Զ������е���ͣ�����ذ�
						   Auto_Run_Flag = FALSE;					   
					   }							   
					   break;	

			case 0x100B: 				                            //ֹͣ
					   if((Robot_Status == ACTION_RUN) || (Robot_Status == ACTION_PAUSE))
					   {
					   	 Order_Temp[0]=ACTION_STOP;             //���û�е��ֹͣ
						   USART1_SendData(1,AUTO_RUN,&Order_Temp[0]);//���ڷ��ͻ�е���Զ������е�ֹͣ�����ذ�
							 if(Robot_Status == ACTION_RUN)
								g_Run_Program_Operater_List = 0;
						   Auto_Run_Flag = FALSE;
						   g_Robot_Auto_Reset = FALSE;
						 g_Auto_Status=0;
					   }					   				   
					   break;	

			case 0x100C: 				                            //����-ѭ��
					   if((Robot_Status == ACTION_STOP)||(Robot_Status == ACTION_PAUSE))
					   {
						   if(Robot_Auto_Mode == CYCLE_MODE)
						   {
						   	 Order_Temp[0]=SINGLE_MODE;           //���û�е��Ϊ����ģʽ
							   USART1_SendData(1,AUTO_RUN,&Order_Temp[0]);//���ڷ��ͻ�е���Զ������еĵ���ģʽ�����ذ�
							 }
						   else
						   {
						   	 Order_Temp[0]=CYCLE_MODE;            //���û�е��Ϊѭ��ģʽ
							   USART1_SendData(1,AUTO_RUN,&Order_Temp[0]);//���ڷ��ͻ�е���Զ������е�ѭ��ģʽ�����ذ�						   						   
						   }
					   }
					   else
					   {
					   	   LCD_Popup_Windows = PW_1_STOPROBOT;  //������ʾ��-����ֹͣ����
					   }
					   break;

// 			case 0x1008:	  //�����ٶ�ֵ
// 				 //���µ��ǼӼ���
// 				 if(LCD_Key_Value>=0x01)
// 				 {
// 					 gs_Origin_Speed=LCD_Key_Value;
// 				 }
// 				 //���µ�����������
// 				 else
// 				 {
// 					 gs_Origin_Speed=LCD_Var_Data;			 
// 				 }
// 				 Order_Temp[0] = ORIGIN_SPEED;
// 				 Order_Temp[1] = gs_Origin_Speed;
// 				 Order_Temp[2] = gs_Origin_Speed>>8;
// 				 Order_Temp[3] = gs_Origin_Speed>>16;
// 				 Order_Temp[4] = gs_Origin_Speed>>24;
// 				 USART1_SendData(5,ROBOT_ENABLE_A_ORIGIN,&Order_Temp[0]);//----ROBOT_ENABLE_A_ORIGIN:0x0A	//��е��ʹ�ܺͻ�ԭ��
// 				 gs_Refresh_Origin_Speed = TRUE;
// 				 break;

			case 0x1040: 				                             //L�������-ȷ��		   	     
					LCD_Current_Page = 	START_UP;		
					LC_Empty_Flag=0;//�ϲֿ�
			     break;

			case 0x1041: 				                             //L�������-ȡ��				 
				    LCD_Current_Page = START_UP;			           //LCD��ǰҳ��Ϊ��������		   
					LC_Empty_Flag=0;//�ϲֿ�
			     break;

			case 0x10A0: 				                             //��λ-ȷ��		   	     
				   Order_Temp[0]=ACTION_RESET;                 //���û�е�ָ�λ
			     USART1_SendData(1,AUTO_RUN,&Order_Temp[0]); //���ڷ��ͻ�е���Զ������еĸ�λ�����ذ�
				   LCD_Current_Page = 89;						   
			     break;

			case 0x10A1: 				                             //��λ-ȡ��
				 LCD_Current_Page = START_UP;                  //LCD��ǰҳ��Ϊ��������
				 Robot_Status = ACTION_STOP;		               //��е��״̬Ϊֹͣ
			     break;
			case 0x1EE1:				//����ƫ��
				 LCD_Current_Page = 143;
				break;
			
			case 0x1EE2:
				LCD_Current_Page=1;
				break;
			default:
			     break;		   	
		}
	}
	else                                                 //�������
	{
// 		if(LCD_Var_Address == 0x1008 )//�����ٶ�ֵ
// 		{	 
// 			 //���µ��ǼӼ���
// 			 if(LCD_Key_Value>=0x01)
// 			 {
// 				 gs_Origin_Speed=LCD_Key_Value;
// 			 }
// 			 //���µ�����������
// 			 else
// 			 {
// 				 gs_Origin_Speed=LCD_Var_Data;			 
// 			 }
// 			 Order_Temp[0] = ORIGIN_SPEED;
// 			 Order_Temp[1] = gs_Origin_Speed;
// 			 Order_Temp[2] = gs_Origin_Speed>>8;
// 			 Order_Temp[3] = gs_Origin_Speed>>16;
// 			 Order_Temp[4] = gs_Origin_Speed>>24;
// 			 USART1_SendData(5,ROBOT_ENABLE_A_ORIGIN,&Order_Temp[0]);
// 			 gs_Refresh_Origin_Speed = TRUE;
// 		 }
		 if(LCD_Var_Address == 0x1007 )	                    //��ԭ������
		 {			
			if(Robot_Enable)
			{
				Order_Temp[0]=ROBOT_ORIGIN;                     //���û�е�ֻ�ԭ��
				USART1_SendData(1,ROBOT_ENABLE_A_ORIGIN,&Order_Temp[0]);//---�·���ԭ�㵽���ذ�
				Robot_Origined = FALSE;
				g_Robot_Stop_Flag = 0;			
			}
			else
			{
				LCD_Popup_Windows = PW_1_ENABLEFIRST;	          //������ʾ��-����ʹ��
			}						 
		 }
		 if(Robot_Origining)                                //������
		 {
		 }
		 else if((LCD_Var_Address == 0x1009) || (LCD_Var_Address == 0x100A) || (LCD_Var_Address == 0x100B) || (LCD_Var_Address == 0x100C))
		 {
		 	 LCD_Popup_Windows = PW_1_HOMEFIRST;	            //������ʾ��-���Ȼ���		 
		 }
	
	}
	LCD_Key_Value=0;                                      //��ֵ����
	LCD_Var_Data=0;                                       //������������
}


void LandingProcess(void)
{
	if(LCD_Current_Page==START_UP && LCD_Var_Address != 0x1001)
	{
		LCD_Popup_Windows = PW_1_LOADFISRT ;	              //������ʾ��-���ȵ�¼	
	}
	if(LCD_Var_Address == 0x1001)		                      //���µ�¼��ť
	{		
		LCD_Current_Page=22;	                              //�л�����½����
		Refresh_Admin_Flag=TRUE;	                          //��ʾȨ��
	}
	//���µ�¼Ȩ��
	if(LCD_Var_Address == 0x1021)  
	{
	  LCD_Current_Page = 103;	                            //�л���Ȩ��ѡ��
	}
	if(LCD_Var_Address == 0x1022)  
	{
	  LCD_Current_Page = 22;	                            //�л�����½����
		Refresh_Admin_Flag=TRUE;	                          //��ʾȨ��
		Admin_Authority=0;                                  //ѡ����ͨȨ�ޡ�
	}
	if(LCD_Var_Address == 0x1023)  
	{
	  LCD_Current_Page = 22;	                            //�л�����½����
		Refresh_Admin_Flag=TRUE;                            //��ʾȨ��
		Admin_Authority=1;	                                //ѡ�񡰹���ԱȨ�ޡ�
	}

	if(LCD_Var_Address == 0x1004)                         //����ȡ����ť
	{
	  if(Change_User == FALSE)                            //�����û�Ȩ���л�
		{
			LCD_Current_Page = START_UP;	                    //�л�����½����
		}
		else					                                      //���û�Ȩ���л�
		{
			LCD_Current_Page = 49;	                          //�л����û�����
			Change_User=FALSE;
		}
		Refresh_Admin_Flag=TRUE;                            //��ʾȨ��
		if(Robot_Admin_Authority==MANAGE_AUTHORITY)
		{
			Admin_Authority=1;
		}
		else
		{
			Admin_Authority=0;
		}
	}
	if(LCD_Var_Address == 0x1005)                          //����ok,��������
	{
		PassWord_Input_Done = TRUE;
		Pass_Word_Temp = LCD_Var_Data;	
	}

	if(LCD_Var_Address==0x1003 && PassWord_Input_Done==TRUE)
	{
		PassWord_Input_Done=FALSE;
		if((Admin_Authority==0) && (Pass_Word[0] == Pass_Word_Temp))//����ͨȨ�ޡ���¼������ȷ
		{
			Landing_Success = TRUE;                             //��¼�ɹ�
			Robot_Admin_Authority=ORDINARY_AUTHORITY;           //��ͨȨ��
			Pass_Word_Temp=0;
			if(Change_User==FALSE)                              //�����û�Ȩ���л�
			{
			  LCD_Current_Page = START_UP;	                    //LCD��ǰҳ��Ϊ��������
				LCD_Main_Page = START_UP;                         //LCD��ҳ��Ϊ��������
				g_Welcom_Flag = TRUE;                             //��ʾ��ӭ
				g_Reflash_Progress_Num = TRUE;                    //ˢ�²�����־λ
				TIM_Cmd(TIM4,ENABLE);
				LCD_Key_Value=0;                                  //��ֵ����
				LCD_Var_Data=0;                                   //������������
				g_Key_Scan=FALSE;
				g_BackLight_Dark = FALSE;
			}
	    else
			{			
			  LCD_Current_Page = 49;	                          //�л����û�����
				Change_User=FALSE;
			}								
		}
		else if((Admin_Authority==1) && (Pass_Word[1] == Pass_Word_Temp))//������ԱȨ�ޡ���¼������ȷ
		{
			Landing_Success = TRUE;                             //��¼�ɹ�
			Robot_Admin_Authority=MANAGE_AUTHORITY;             //����ԱȨ��
			Pass_Word_Temp=0;
			if(Change_User==FALSE)                              //�����û�Ȩ���л�
			{
			  LCD_Current_Page = START_UP;	                    //LCD��ǰҳ��Ϊ��������
				LCD_Main_Page = START_UP;                         //LCD��ҳ��Ϊ��������
				g_Welcom_Flag = TRUE;                             //��ʾ��ӭ
				g_Reflash_Progress_Num = TRUE;                    //ˢ�²�����־λ
				TIM_Cmd(TIM4,ENABLE);
				LCD_Key_Value=0;                                  //��ֵ����
				LCD_Var_Data=0;                                   //������������
				g_Key_Scan=FALSE;
				g_BackLight_Dark = FALSE;
			}
	    else
			{			
			  LCD_Current_Page = 49;	                          //�л����û�����
				Change_User=FALSE;
			}	
		}
		else
		{
		  if(Change_User==FALSE)
			{
				Landing_Success = FALSE;                          //��¼δ�ɹ�
				g_Welcom_Flag = FALSE;                            //����ʾ��ӭ
			}
			LCD_Popup_Windows = PW_1_PASSWORDERROR ;	          //������ʾ��-�������			
		}			
	}
}


/*************************************************************************
**  ��������  LCDStartUpDisply()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/5/28
**************************************************************************/
extern u8 Point_String[6][8];
void LCDStartUpDisply()
{
	u8 i=0;
	//--�����л�--//
	if(LCD_Pre_Page != LCD_Current_Page )	                   //֮ǰ��ʾ���治�ǵ�ǰ����
	{
		LCD_Pre_Page = LCD_Current_Page;                       //����֮ǰ����Ϊ��ǰ����
		PageChange(LCD_Pre_Page);
//		gs_Run_Program_Num=0;
	}
	//����������ʾ
	if(g_Reflash_Progress_Num)
	{
		WriteNum4(0x5350,SC_Parameter.RW_Num);                 //����ƻ�������-�ӹ�����
		WriteNum4(0x535A,SC_Parameter.SC_Num);                 //����-�������
		g_Reflash_Progress_Num = FALSE;		
	}
	if((Landing_Success == TRUE)&&(Robot_Origined == TRUE)&&(gs_SC_Parameter_SC_Num != SC_Parameter.SC_Num) )
	{
		WriteNum4(0x535A,SC_Parameter.SC_Num);                 //����-�������
		gs_SC_Parameter_SC_Num = SC_Parameter.SC_Num;          //��������Ϊ����-�������
		//ˢ�����ϲ�λ��
		//if(LC_Parameter_Load.Switch==TRUE && LC_Parameter_Load.H_Num>0 && LC_Parameter_Load.V_Num>0)	//���̼������
		if((gs_Run_Program_Num>0) && (g_Calculate_LC_Load_Parameter_Flag==TRUE) )
		{
			LC_Load_Pre_PositionX=SC_Parameter.SC_Num%LC_Parameter_Load[LC_ID].H_Num;	
			LC_Load_Pre_PositionY=SC_Parameter.SC_Num/LC_Parameter_Load[LC_ID].H_Num%LC_Parameter_Load[LC_ID].V_Num;
			if(LC_Load_Pre_ListX>0)
			{
			    Free_Program_Operate.Program[LC_Load_Pre_ListX].Value1=LC_Load_Position[LC_Load_Pre_PositionX][LC_Load_Pre_PositionY].XPoint|0xf0000000;
			}
			if(LC_Load_Pre_ListY>0)
			{
			    Free_Program_Operate.Program[LC_Load_Pre_ListY].Value1=LC_Load_Position[LC_Load_Pre_PositionX][LC_Load_Pre_PositionY].YPoint|0xf0000000;
			}
		}
		//ˢ��ж�ϲ�λ��
		//if(LC_Parameter_UnLoad.Switch==TRUE && LC_Parameter_UnLoad.H_Num>0 && LC_Parameter_UnLoad.V_Num>0)	//���̼������
		if((gs_Run_Program_Num>0) && (g_Calculate_LC_UnLoad_Parameter_Flag==TRUE)) 
		{
		  LC_UnLoad_Pre_PositionX=SC_Parameter.SC_Num%LC_Parameter_UnLoad.H_Num;	
			LC_UnLoad_Pre_PositionY=SC_Parameter.SC_Num/LC_Parameter_UnLoad.H_Num%LC_Parameter_UnLoad.V_Num;
			if(LC_UnLoad_Pre_ListX>0)
			{
			    Free_Program_Operate.Program[LC_UnLoad_Pre_ListX].Value1=LC_UnLoad_Position[LC_UnLoad_Pre_PositionX][LC_UnLoad_Pre_PositionY].XPoint|0xf0000000;
			}
			if(LC_UnLoad_Pre_ListY>0)
			{
			    Free_Program_Operate.Program[LC_UnLoad_Pre_ListY].Value1=LC_UnLoad_Position[LC_UnLoad_Pre_PositionX][LC_UnLoad_Pre_PositionY].YPoint|0xf0000000;
			}
		}
	}
  //--��ӭ-��¼--//
	if(gs_Welcom_Flag != g_Welcom_Flag)			   
	{
		gs_Welcom_Flag = g_Welcom_Flag;
		if(gs_Welcom_Flag)
		{
			DisplayChineseString(0x100E,Send_String_HY,4);		     //��ʾ��ӭ 
		}
		else
		{
			DisplayChineseString(0x100E,Send_String_DL,4);         //��ʾ��¼
		}	
	}
	//--ʹ��-ʧ��--//
	if(gs_Robot_Enable_Flag != g_Robot_Enable_Flag)	 
	{
		gs_Robot_Enable_Flag = g_Robot_Enable_Flag;	
		if(gs_Robot_Enable_Flag)
		{
//			Display_PageCut(PageCut_Enable);                       //��ʾʹ��
		}
		else
		{
//			Display_PageCut(PageCut_Disable);                      //��ʾʧ��
		}	
	}
	//--��ͣ--//
	if(gs_Emergency_Stop_Flag != g_Emergency_Stop_Flag)	 
	{
		gs_Emergency_Stop_Flag = g_Emergency_Stop_Flag;
		if(gs_Emergency_Stop_Flag)
		{
			Display_PageCut(PageCut_Emerg_Y);                      //��ʾ��ͣ
		}
		else
		{
			Display_PageCut(PageCut_Emerg_N);		
		}		
	}
	//--ѭ��-����--//
	if(gs_Cycle_Run_Flag != g_Cycle_Run_Flag)	 
	{
		gs_Cycle_Run_Flag = g_Cycle_Run_Flag;
		if(gs_Cycle_Run_Flag)	                                   //ѭ��
		{
			//---Display_PageCut(PageCut_CyCle1);
			Display_PageCut(PageCut_CyCle2);                       //��ʾѭ��   
			gs_Run_Program_Code[0]=1;
			gs_Run_Program_Code[1]=2;
			gs_Run_Program_Code[2]=3;
			gs_Run_Program_Code[3]=4;
			gs_Run_Program_Code[4]=5;
			gs_Run_Program_Code[5]=6;
			gs_Run_Program_Code[6]=7;
			gs_Run_Program_Code[7]=8;
			gs_Run_Program_List[0]=1; 		
			gs_Run_Program_List[1]=2; 		
			gs_Run_Program_List[2]=3; 		
			gs_Run_Program_List[3]=4; 		
			gs_Run_Program_List[4]=5; 		
			gs_Run_Program_List[5]=1; 		
			gs_Run_Program_List[6]=2; 		
			gs_Run_Program_List[7]=3; 			
		}
		else					                                           //����
		{
			//---Display_PageCut(PageCut_Sigle1);
			Display_PageCut(PageCut_Sigle2);                       //��ʾ����		
		}				
	}
	//--Xλ��ˢ��--//
	if(gs_StartUp_XAxis_Position != g_XAxis_Position)	 
	{
		gs_StartUp_XAxis_Position = g_XAxis_Position;
		WriteNum4(0x101A,gs_StartUp_XAxis_Position);		
	}
	//--Zλ��ˢ��--//
	if(gs_StartUp_ZAxis_Position != g_ZAxis_Position)	 
	{
		gs_StartUp_ZAxis_Position = g_ZAxis_Position;
		WriteNum4(0x101C,gs_StartUp_ZAxis_Position);		
	}
	//--Lλ��ˢ��--//
	if(gs_StartUp_LAxis_Position != g_LAxis_Position)	 
	{
		gs_StartUp_LAxis_Position = g_LAxis_Position;
		WriteNum4(0x101E,gs_StartUp_LAxis_Position);		
	}
	//--Oλ��ˢ��--//
	if(gs_StartUp_OAxis_Position != g_OAxis_Position)	 
	{
		gs_StartUp_OAxis_Position = g_OAxis_Position;
		WriteNum4(0x1018,gs_StartUp_OAxis_Position);		
	}
	//�������У�ˢ����ʾ&&Free_Program_Operate.Num<=LARGESTPROGRAMNUM
	if(gs_Run_Program_Operater_List != g_Run_Program_Operater_List)
	{
		Waite_Reflesh=0;
		if(gs_Cycle_Run_Flag)//ѭ����ʾ�̶�����
		{
			gs_Run_Program_List[8]=gs_Run_Program_List[0];
			gs_Run_Program_Parameter_Display[8][0] = gs_Run_Program_Parameter_Display[0][0];
			gs_Run_Program_Parameter_Display[8][1] = gs_Run_Program_Parameter_Display[0][1];
			for(i=0;i<8;i++)
			{
				gs_Run_Program_List[i] = gs_Run_Program_List[i+1];
				gs_Run_Program_Parameter_Display[i][0] = gs_Run_Program_Parameter_Display[i+1][0];
				gs_Run_Program_Parameter_Display[i][1] = gs_Run_Program_Parameter_Display[i+1][1];
			}
			for(i=0;i<8;i++)
			{
				gs_Run_Program_Code[i] = g_Run_Program_Operater_List+i;
			}
		}
		else//������ʾ��ʵ����
		{
			for(i=0;i<8;i++)
			{
				AT24CXX_Read(CURRENT_PROGRAM_ADR1+ORDER_LENTH*(g_Run_Program_Operater_List+i-1),&Order_Temp[3],ORDER_LENTH);
				if(Free_Program_Operate.Num>=g_Run_Program_Operater_List+i)
				{
					gs_Run_Program_Code[i] = g_Run_Program_Operater_List+i;
					gs_Run_Program_List[i] = Order_Temp[26];
					gs_Run_Program_Parameter_Display[i][0] = Order_Temp[5]|Order_Temp[6]<<8|Order_Temp[7]<<16|Order_Temp[8]<<24;
					gs_Run_Program_Parameter_Display[i][1] = Order_Temp[9]|Order_Temp[10]<<8|Order_Temp[11]<<16|Order_Temp[12]<<24;
				}
				else
				{
					gs_Run_Program_Code[i] = 0;
					gs_Run_Program_List[i] = 0;
					gs_Run_Program_Parameter_Display[i][0] = 0;
					gs_Run_Program_Parameter_Display[i][1] = 0;
				}					
			}
		}
			gs_Run_Program_Operater_List = g_Run_Program_Operater_List;
			gs_Program_Refresh_Flag =TRUE;

	}
	//--����Ч��--//
	if((g_Auto_Status!=gs_Auto_Status) && (g_Auto_Status == AUTO_RUNNING) )	 
	{
		Display_PageCut(PageCut_Run_Y);                          //��ʾ����
		Display_PageCut(PageCut_Pause_N);
		Display_PageCut(PageCut_Stop_N);
		gs_Auto_Status=g_Auto_Status;	
		gs_Run_Program_Code[0]=1;
		gs_Run_Program_Code[1]=2;
		gs_Run_Program_Code[2]=3;
		gs_Run_Program_Code[3]=4;
		gs_Run_Program_Code[4]=5;
		gs_Run_Program_Code[5]=6;
		gs_Run_Program_Code[6]=7;
		gs_Run_Program_Code[7]=8;
		gs_Run_Program_List[0]=1; 		
		gs_Run_Program_List[1]=2; 		
		gs_Run_Program_List[2]=3; 		
		gs_Run_Program_List[3]=4; 		
		gs_Run_Program_List[4]=5; 		
		gs_Run_Program_List[5]=1; 		
		gs_Run_Program_List[6]=2; 		
		gs_Run_Program_List[7]=3;
	}
	//--��ͣЧ��ˢ��--//
	if((g_Auto_Status!=gs_Auto_Status) && (g_Auto_Status == AUTO_PAUSE) )	 
	{
		Waite_Reflesh=0;
		Robot_Status=ACTION_PAUSE;
		Display_PageCut(PageCut_Run_N);            
		Display_PageCut(PageCut_Pause_Y);                        //��ʾ��ͣ
		Display_PageCut(PageCut_Stop_N);
		gs_Auto_Status=g_Auto_Status;	
		
		for(i=0;i<8;i++)
		{
			AT24CXX_Read(CURRENT_PROGRAM_ADR1+ORDER_LENTH*(g_Run_Program_Operater_List+i-1),&Order_Temp[3],ORDER_LENTH);
			if(Free_Program_Operate.Num>=g_Run_Program_Operater_List+i)
			{
				gs_Run_Program_Code[i] = g_Run_Program_Operater_List+i;
				gs_Run_Program_List[i] = Order_Temp[26];
				gs_Run_Program_Parameter_Display[i][0] = Order_Temp[5]|Order_Temp[6]<<8|Order_Temp[7]<<16|Order_Temp[8]<<24;
				gs_Run_Program_Parameter_Display[i][1] = Order_Temp[9]|Order_Temp[10]<<8|Order_Temp[11]<<16|Order_Temp[12]<<24;
			}
			else
			{
				gs_Run_Program_Code[i] = 0;
				gs_Run_Program_List[i] = 0;
				gs_Run_Program_Parameter_Display[i][0] = 0;
				gs_Run_Program_Parameter_Display[i][1] = 0;
			}				
		}
		gs_Program_Refresh_Flag=TRUE;
	}
	//���³���ȷ��ǰ����
	if((Landing_Success == TRUE) && ((gs_Run_Program_Num != g_Run_Program_Num)||(gs_Run_Program_Name!=g_Run_Program_Name)))
	{	
		Waite_Reflesh=0;
		gs_Run_Program_Num = g_Run_Program_Num;
		g_Run_Program_List_Num = Free_Program_Operate.Num;
		gs_Run_Program_Name = g_Run_Program_Name;
		//��ʾ������
		if(gs_Run_Program_Name==0)
		{
			Temp_Display_Data[0]=0x20;
			Temp_Display_Data[1]=0x20;
			Temp_Display_Data[2]=0x20;
			Temp_Display_Data[3]=0x20;
			DisplayChineseString(0x11F0,Temp_Display_Data,4);
		}
		else
		{
			Temp_Display_Data[0]=gs_Run_Program_Name>>24;
			Temp_Display_Data[1]=gs_Run_Program_Name>>16;
			Temp_Display_Data[2]=gs_Run_Program_Name>>8;
			Temp_Display_Data[3]=gs_Run_Program_Name;
			DisplayChineseString(0x11F0,Temp_Display_Data,4);
		}
		//ȷ����ʾ����-����-����

		for(i=0;i<8;i++)
		{
			if(Free_Program_Operate.Program[i].Flag !=0)
			{
				gs_Run_Program_Code[i] = Free_Program_Operate.Program[i].List;
				gs_Run_Program_List[i] = Free_Program_Operate.Program[i].Key;
				gs_Run_Program_Parameter_Display[i][0] = Free_Program_Operate.Program[i].Value1;
				gs_Run_Program_Parameter_Display[i][1] = Free_Program_Operate.Program[i].Value2;
			}
			else
			{
				gs_Run_Program_Code[i] = 0;
				gs_Run_Program_List[i] = 0;
				gs_Run_Program_Parameter_Display[i][0] = 0;
				gs_Run_Program_Parameter_Display[i][1] = 0;
			}
		}
		gs_Program_Refresh_Flag =TRUE;
	}
	//--ֹͣЧ��--//
	else if((g_Auto_Status!=gs_Auto_Status) && (g_Auto_Status == AUTO_WAITE) )	 
	{
		Waite_Reflesh=0;
	    Display_PageCut(PageCut_Run_N);
		Display_PageCut(PageCut_Pause_N);
		Display_PageCut(PageCut_Stop_Y);                         //��ʾֹͣ
		gs_Auto_Status=g_Auto_Status;
		g_Run_Program_Operater_List = 0;
		Display_Flag=0;
	}
	else if(g_Auto_Status == AUTO_WAITE&&Waite_Reflesh==0)//����ֹͣ״̬ˢ��
	{
		Waite_Reflesh++;
		for(i=0;i<8;i++)
		{
			AT24CXX_Read(CURRENT_PROGRAM_ADR1+ORDER_LENTH*i,&Order_Temp[3],ORDER_LENTH);
			if(Free_Program_Operate.Num>=Order_Temp[3])
			{
				gs_Run_Program_Code[i] = i+1;
				gs_Run_Program_List[i] = Order_Temp[26];
				gs_Run_Program_Parameter_Display[i][0] = Order_Temp[5]|Order_Temp[6]<<8|Order_Temp[7]<<16|Order_Temp[8]<<24;
				gs_Run_Program_Parameter_Display[i][1] = Order_Temp[9]|Order_Temp[10]<<8|Order_Temp[11]<<16|Order_Temp[12]<<24;
			}
			else
			{
				gs_Run_Program_Code[i] = 0;
				gs_Run_Program_List[i] = 0;
				gs_Run_Program_Parameter_Display[i][0] = 0;
				gs_Run_Program_Parameter_Display[i][1] = 0;
			}		
		}
		gs_Program_Refresh_Flag=TRUE;
	}
	//--����ˢ��--//
	if(gs_Program_Refresh_Flag)	 
	{
		//��ʾ����
		Display_PageCut(Run_Program_Color[gs_Run_Program_Color]);
		//��ʾ����-����
	    for(i=0;i<8;i++)
	    {
			//��ʾ����
			DisplayChineseString(0x1200+0x40*i,ProgramCode_String[gs_Run_Program_Code[i]],4);		
			//��ʾ����
			if(gs_Run_Program_List[i]!=0)
			{
				DisplayChineseString(0x1210+0x40*i,Point_String[gs_Run_Program_List[i]],8);
			}
			else
			{
				DisplayChineseString(0x1210+0x40*i,Point_String[0],8);	
			} 	
			//��ʾ����
			if((gs_Run_Program_Parameter_Display[i][0] & 0xf0000000) != 0)
			{
				ValueToString(gs_Run_Program_Parameter_Display[i][0]);
				DisplayChineseString(0x1220+0x40*i,Parameter_Value,10);
			}
			else
			{			
				DisplayChineseString(0x1220+0x40*i,Parameter_String[0],10);
			}
			if((gs_Run_Program_Parameter_Display[i][1] & 0xf0000000) != 0)
			{
				ValueToString(gs_Run_Program_Parameter_Display[i][1]);
				DisplayChineseString(0x1230+0x40*i,Parameter_Value,10);
			}
			else
			{
				DisplayChineseString(0x1230+0x40*i,Parameter_String[0],10);
			}
	    }		
		gs_Program_Refresh_Flag = FALSE;		
	}
	//�����ٶ���ʾˢ��
// 	if(gs_Refresh_Origin_Speed)
// 	{
// 		WriteNum2(0x1008,gs_Origin_Speed); 
// 		gs_Refresh_Origin_Speed = FALSE;	
// 	}
	//Ȩ����ʾ
	if(Refresh_Admin_Flag)
	{
  	 DisplayChineseString(0x1030,Admin_Authority_Dis[Admin_Authority],10);
	   Refresh_Admin_Flag=FALSE;	
	}
	//�ֶ�����-L��������
	if((LCD_Current_Page==117)&&(g_LAxis_Position==0))
	{
		LCD_Current_Page=1;
	}

}



