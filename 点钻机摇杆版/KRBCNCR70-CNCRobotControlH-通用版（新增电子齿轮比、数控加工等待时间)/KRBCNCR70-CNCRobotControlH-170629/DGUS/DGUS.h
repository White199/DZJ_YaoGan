/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __DGUS_H_
#define __DGUS_H_

#include "Usart.h"

#include "StartUp.h"
#include "FreeProgramme.h"
#include "IOControl.h"
#include "ManulOperate.h"
#include "SystemSetting.h"
#include "AlarmHistory.h"
#include "IOCommand.h"

#include "string.h"

#include "Parameter.h"
#include "Usart1_Robot.h"
#include "24Cxx.h"

//------------������ҳ��------------//
#define START_UP       1
#define FREE_PROGRAMME 2
#define IO_CONTROL     12
#define MANUL_DEBUG    4
#define SYSTEM_SETTING 17
#define ALARM_HISTORY  16
//#define START_SYC_DATA 74

#define ORDINARY_AUTHORITY  1	//��ͨȨ��
#define MANAGE_AUTHORITY  2	//����ԱȨ��

extern u16 LCD_Var_Address ;			   //������ַ
extern u32 LCD_Var_Data ;				   //��������
extern u16 LCD_Key_Value ;				   //������ֵ
extern u16 LCD_GearRatio_Distance[6];   //����У׼������� mm
extern u16 LCD_RTC_Reg_len; 			   //ʵʱʱ�ӼĴ��������ݳ���
extern u32 LCD_RTC_Day;					   //��-��-��-��
extern u32 LCD_RTC_Time ;				   //ʱ-��-��
extern u8  LCD_New_Order ;		           //���յ�������
extern u8  LCD_Data_Decode_Flag;		   //�������ݱ�־λ
extern u8  LCD_Popup_Windows;			   //������ʶλ
extern u8  LCD_Current_Page;			   //��ǰ��ʾҳ��
extern u8  LCD_Pre_Page;
extern u8  LCD_Main_Page ;
extern u8  LCD_Page_Refresh;
extern u8  LCD_Display_ChineseString;
extern u8  LCD_Page_Cut;
extern u8  LCD_Hidden_Display;

extern u8  Dispaly_Order_Num ;		 //������ʾ��������
extern u8  Dispaly_Order[20] ;	 //������ʾ��������
extern u8  Order_Temp[60] ;			 //ָ��棬���ڴ��ڷ���
extern u8  IIC_Temp[60];
//��ʾ�����ַ��ṹ��
typedef struct 
{
	u16 addr ;
	u8 *data ;
	u8 len ;
}ChineseString;
extern ChineseString Display_ChineseString;

//ͼƬ���нṹ��
typedef struct 
{
	u16 Vary_addr ;
	u16 Pic_ID;
	u16 Left_X;
	u16 Up_Y;
	u16 Right_X;
	u16 Down_Y;
	u16 New_X;
	u16 New_Y;
}PageCut;
//extern PageCut  Display_PageCut;

//��ʾ��������-����ָ��
typedef struct 
{
	u16 Des_Point ;
	u16 Vary_addr ;
}UnHidden;
extern UnHidden UnHidden_Display;

//������ʾ����-����ָ��
typedef struct 
{
	u16 Des_Point ;
}Hidden;
extern Hidden Hidden_Display;

extern u32 Pass_Word[2];
extern u8  Refresh_Admin_Flag;
extern u8  Admin_Authority;					   //0:��ͨȨ�� 1:����ԱȨ��
extern u8  Admin_Authority_Dis[2][10];
extern u8  Robot_Admin_Authority;
extern u32 g_Pass_Word_Count;
extern u32 g_Pass_Word_Delay; 
/*----------------Start_Up--����1������־λ----------------*/
extern u8 g_Alarm_Refresh_Flag ;	     //������ʾ
extern u8 g_Welcom_Flag ;	             //��ʾ��ӭ
extern u8 g_Robot_Enable_Flag ;	         //ʹ��
extern u8 g_Emergency_Stop_Flag ;	     //��ͣ
extern u8 g_Cycle_Run_Flag ;	 		 //0:ѭ��  1������
extern u8 g_X_Position_Flag;	 		 //Xλ��ˢ��
extern u8 g_Z_Position_Flag ;	 		 //Zλ��ˢ��
extern u8 g_Robot_Origin_Flag ;	 		 //����
extern u8 g_Robot_Run_Flag ;			 //����
extern u8 g_Robot_Pause_Flag ;			 //��ͣ
extern u8 g_Robot_Stop_Flag ;			 //ֹͣ
extern u8 g_Program_Refresh_Flag;	     //��ʾ����ˢ��
extern u8 g_Run_Program_Num;
extern u16 g_Run_Program_Operater_List,g_Run_Program_Operater_List1;	 //��ǰѡ�����г����������е�ָ�����������ذ巵��
extern u8 g_Run_Step;
extern u8 Auto_Run_Flag;
extern u8 Auto_Run_Count;
extern u16 g_Run_Program_List_Num;
extern u8 g_Reflash_Progress_Num ;
extern u8 g_Robot_Auto_Reset;
extern u8 g_Robot_Auto_Reset_Scan;
extern u8 Axis_Position_Scan ;
/*----------------Free_Program--����2������־λ--------------*/
extern u8 g_Background_Color;			  //�༭����ɫ
extern u8 g_Background_Color_Blink ;      //������˸��־λ
extern u8 g_Background_Blink_Delay ;      //������˸��ʱ��־λ
extern u8 g_Background_Blink_Count;
extern u8 g_Event_Done_Flag  ; 
extern u8 g_SaveProgram_Flag ;            //���򱣴�
extern u32 Axis_Parameter_Speed;          //���ٶ�ֵ�ٷֱ�
extern u32 Delay_Parameter_Time ;				  //��ʱʱ�����
extern u8 g_Current_Paramter_Num ;		  //����������
extern u8 g_Current_Paramter_Key ;		  //��������ָ����
extern u8 g_Pre_SafeArea;   		      //��������ʱ-���浱ǰѡ��ȫ����
extern u8 g_FreeProgram_Setting_XZ;
extern u8 gs_FreeProgram_Setting_XZ;
/*----------------IO_Control--����3������־λ--------------*/
extern u8  g_Watch_IO_Control;
extern u16 g_LocalIO_Status ;	 //����IO״̬
extern u16 g_Input1_Status  ;	 //����1״̬
extern u16 g_Input2_Status  ;	 //����2״̬
extern u16 g_Input3_Status  ;	 //����3״̬
extern u16 g_Output1_Status ;	 //���1״̬
extern u16 g_Output2_Status ;	 //���2״̬
extern u16 g_Output3_Status ;	 //���3״̬
extern u8  g_Output1Ack_Flag;
extern u8  g_Output2Ack_Flag;
extern u8  g_Output3Ack_Flag;

/*----------------Manul_Operate--����4������־λ------------*/
extern u8  g_ManulOperate_Flag; 
extern u8  g_X_Origin_Flag;			     //X�����
extern u8  g_Z_Origin_Flag;			     //z�����
extern u8  g_L_Origin_Flag;			     //z�����
extern u8  g_X_SavePoint_Flag;	         //X����
extern u8  g_Z_SavePoint_Flag ;	         //Z����
extern u8  g_L_SavePoint_Flag ;	         //L����
extern u8  g_O_SavePoint_Flag ;	         //O����
extern u8  g_Current_Manul_Axis;		 //�ֶ�΢��ʱ��ǰѡ����
extern u16 g_Manule_Pulse ;
extern u16 gs_Manule_Pulse ;
/*---------------------------------------------------------*/

/*----------------SystemSetting--����5������־λ------------*/
extern u8  g_Buzzer_Switch_Flag;
extern u32 g_BackLight_Time;
extern u32 g_BackLight_Count;
extern u8  g_BackLight_Dark;
//extern u8  g_X_SoftLimit_flag; 
//extern u8  g_Z_SoftLimit_flag;
//extern u8  g_L_SoftLimit_flag;
extern PageCut Safe_Area_List[7];
extern u8  USBH_Process_Flag;       
extern u8  USBH_Direction;
extern u8  USBH_Dispaly_Fresh_Flag;
extern u8  USBH_And_Control[2][14];
extern u8  USBH_CopyStatus_String[12][22];
extern u8 USBH_Copy_Status;	   
extern u8 USB_Send_Program;

extern u8  USB_Update_Flag;               //---2018.5.28
extern u8  USB_Update_Status;             //---2018.5.28
extern u8  USB_Dispaly_Fresh_Flag;        //---2018.5.28
extern u8  USB_Update_Choice[4][14];      //---2018.5.28
extern u8  USB_UpdateStatus_String[8][24];//---2018.5.28
/*---------------------------------------------------------*/


//------------�����˵�------------//
#define PW_1_PASSWORDERROR      0x10   //�������
#define PW_1_LOADFISRT          0x20   //���ȵ�¼
#define PW_1_HOMEFIRST          0x30   //���Ȼ���
#define PW_1_ENABLEFIRST        0x40   //����ʹ��
#define PW_1_STOPROBOT          0x50   //����ֹͣ��е��
#define PW_4_POINTSAVEFULLX     0x60   //X�洢��λ������
#define PW_4_POINTSAVEFULLZ     0x70   //Z�洢��λ������
#define PW_5_PREPASSWORDERROR   0x80   //ԭ�������
#define PW_5_ENTERPASSWORD      0x90   //��������Ч��λ����
#define PW_5_NOTTHESAME         0xA0   //�����������벻һ��
#define PW_6_HOMEFIRST1         0xB0   //������ʷ-���Ȼ���1
#define PW_6_HOMEFIRST2         0xC0   //������ʷ-���Ȼ���2
#define PW_4_HOMEFIRST          0xD0   //ϵͳ����-���Ȼ���
#define PW_2_PROGRAMSAVEFULL    0xE0   //�洢����λ������
#define PW_2_PARAMETERUNSET     0xF0   //�������δ�������

#define PW_5_SOFTLIMITERROR     0x15   //����λ��������
#define PW_5_SAFEAREAERROE      0x25   //��ȫ����������
#define PW_1_CHOOSEPROGRAM      0x35   //����ѡ�����г���
#define PW_2_STARTENDERROR      0x45   //����ʼ-������ʽ����
#define PW_1_NOINFISRTPAGE      0x55   //��е��ʧ��-������ҳ
#define PW_5_UNLOCKCODEEROOR    0x65   //�����������
//ZJͨ�ð汾����
#define PW_5_GEARRATIO          0x75   //���ӳ��ֱȴ���
#define PW_5_SAFEAREAHOME       0x85   //��ȫ�������
#define PW_5_PASSWORDERROR      0xB5   //�̼������������
#define PW_5_REBOOTSYSTEM		0xC5	//�̼�ѡ����ɣ���ʾ����

void DisplayChineseString(u16,u8*,u8 );
void DisplayPageCut(u16 Pic_addr,u16 Pic_ID,u16 row1,u16 column1,u16 row2,u16 column2,u16 row3,u16 column3);
void Display_PageCut(PageCut Icon);

void WriteNum2(u16 addr,u16 num);
void WriteNum4(u16 addr,u32 num);
//void WriteNum8(u16 addr,u64 num);
void WriteNum9(u16 addr);

void LCDDisplayFuction(void);	//LCD��ʾ������ʵ�ֺ���
void LCDDisplay(void);			//LCD��ʾ����
void LCDProcess(void);			//��������

void BuzzerRing(u8);
void BackLight(u8);
void ReadRTC(void);
void PageChange(u8);
void Popup_Windows(u8);

void HiddenDisplay(u16);
void UnHiddenDisplay(u16,u16);






#endif

