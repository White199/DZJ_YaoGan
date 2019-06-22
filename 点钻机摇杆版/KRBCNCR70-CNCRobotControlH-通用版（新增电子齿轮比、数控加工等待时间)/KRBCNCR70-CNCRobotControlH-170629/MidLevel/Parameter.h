/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#ifndef __PARAMETER_H_
#define __PARAMETER_H_

#include "FreeProgramme.h"

/**------------����--------------**/
#define   ROBOT_ENABLE_A_ORIGIN  0x0A	//��е��ʹ�ܺͻ�ԭ��
#define   WORK_MODE              0x0B	//��е�ֹ���ģʽ
#define   AUTO_RUN               0x0C	//��е���Զ�����
#define   FREE_PROGRAM_SEND      0x0D	//���ɱ�̳����·�
#define   WATCH_COMMAND          0x0E	//��������
#define   READ_IIC	             0x0F	//������IIC
#define   IO_DEBUG_OUTPUT1       0x1A	//IO����-���1
#define   IO_DEBUG_OUTPUT2       0x1B	//IO����-���2
#define   IO_DEBUG_OUTPUT3       0x1C	//IO����-���3
#define   MANUL_OPERATE          0x1D	//�ֶ�����
#define   PARAMETER_ORDER        0x1E	//��������

/**-------0x0A-----ʹ�ܺͻ�ԭ��--------------**/
#define   ROBOT_ORIGIN   	     0x01	//��е�ֻ�ԭ��
#define   X_AXSIS_ORIGIN	     0x02	//X���ԭ��
#define   Z_AXSIS_ORIGIN	     0x03	//Z���ԭ��
#define   ORIGIN_SPEED	         0x04	//��е�ֻ�ԭ���ٶ�
#define   ROBOT_ENABLE	         0x05	//��е���Ƿ�ʹ��

/**-------0x0B-----����ģʽ--------------**/
#define   WAIT_MODE			     0x00   //�ȴ�ģʽ
#define   AUTO_MODE              0x01	//�Զ�ģʽ
#define   FREE_PROGRAM           0x02	//���ɱ��
#define   IO_MODE                0x03	//IO����
#define   MANUL_MODE             0x04	//�ֶ�����

/**-------0x0C-----�Զ�����--------------**/
#define   CYCLE_MODE             0x01	//ѭ��ģʽ
#define   SINGLE_MODE            0x02	//����ģʽ
#define   ACTION_RUN             0x03	//����
#define   ACTION_PAUSE           0x04	//��ͣ
#define   ACTION_STOP            0x05	//ֹͣ
#define   ACTION_PROGRAM         0x06	//ѡ�����еĳ���
#define   ACTION_RESET           0x07	//��е�ָ�λ
#define   ACTION_DEBUG           0x08	//��е�ֵ���-���ɱ�̵ĵ��԰�ť
#define   ACTION_LORIGIN         0x09	//��е��L�����

/**-------0x0D-----���ɱ��--------------**/
#define   PROGRAM_START          0xE1	//����ʼ
#define   PROGRAM_INFO           0xE2	//������Ϣ����
#define   PROGRAM_CONT           0xE3	//��������
#define   PROGRAM_DELETE         0xE4	//����ɾ��
#define   PROGRAM_FROM_USB_START 0xE5	//USB���򿽱����Ϳ�ʼ
#define   PROGRAM_FROM_USB_END   0xE6	//USB���򿽱����ͽ���
#define   PROGRAM_END            0xED	//�������

#define   P_PROGRAM_RENAME				 0xE7 //������
#define   P_PROGRAM_CHOOSE         0xE8	//��Ʒѡ��
#define   P_PROGRAM_DELETE         0xE9	//��Ʒɾ��
#define   P_PROGRAM_RENAME_COVER   0xEA //��Ʒ����������
#define   P_PROGRAM_NEW						 0xEB //��Ʒ�½�
#define   P_PROGRAM_NEWNAME_COVER	 0xEC //�½�ͬ������
#define   P_PROGRAM_RECEIVE				 0xED //�������ذ����

/**-------0x0E-----��������--------------**/
#define   IO_DEBUG_LOCAL         0x01	//IO����-����IO
#define   IO_DEBUG_INPUT1        0x02	//IO����-����1
#define   IO_DEBUG_INPUT2        0x03	//IO����-����2
#define   IO_DEBUG_INPUT3        0x04	//IO����-����3
#define   IO_DEBUG_OUTPUT1_LCD   0x05	//IO����-���1����
#define   IO_DEBUG_OUTPUT2_LCD   0x06	//IO����-���2����
#define   IO_DEBUG_OUTPUT3_LCD   0x07	//IO����-���3����
#define   ROBOT_ORIGINED		 0x08	//��е���Ƿ��ѻ�ԭ��
#define   ACTION_RESET_SCAN	     0x09	//��е�ָ�λ״̬��ѯ
#define   ALARM_CLEAR	         0x0A	//�����ǰ������Ϣ
#define   ROBOT_STATUS      	 0x0B	//��ѯ��е��״̬
#define   AUTO_PARAMETER         0x0C	//�Զ����в���-��ǰ��-����
#define   X_AXSIS_POSITION	     0x0D	//X������
#define   Z_AXSIS_POSITION       0x0E	//Z������
#define   ROBOT_PRE_STATUS       0x0F	//��е������״̬
#define   ROBOT_DEBUG_STATUS     0x10	//��е�ֵ���״̬
#define   L_AXSIS_POSITION       0x11	//L������
#define   O_AXSIS_POSITION       0x12	//O������
#define   A_AXSIS_POSITION       0x13	//V������
#define   B_AXSIS_POSITION       0x14	//W������

/**-------0x0F-----������IIC--------------**/
#define   READ_IIC_1	         0x01	//
#define   READ_IIC_2	         0x02	//
#define   READ_IIC_3	         0x03	//
#define   READ_IIC_4	         0x04	//
#define   READ_IIC_5	         0x05	//
#define   READ_IIC_6	         0x06	//
#define   READ_IIC_7	         0x07	//
///**-------0x1A-----IO����-���1--------------**/
//#define   STORAGE_LOAD_FINISH    0x01	//�ϲ�-�������
//#define   STORAGE_UNLOAD_FINISH  0x02	//�ϲ�-ж�����
//#define   STORAGE_LOAD_ALARM     0x03	//�ϲ�-���ϲֱ���
//#define   STORAGE_UNLOAD_ALARM   0x04	//�ϲ�-ж�ϲֱ���
//#define   STORAGE_RESERVE1       0x05	//�ϲ�-Ԥ��1
//
//#define   MACHINE1_KAPA_OPEN     0x06	//����1����A��
//#define   MACHINE1_KAPA_CLOSE    0x07	//����1����A��
//#define   MACHINE1_CUIQI         0x08	//����1����
//#define   MACHINE1_ALLOW_START   0x09	//����1��������
//#define   MACHINE1_START_PROCESS 0x0A	//����1��ʼ�ӹ�
//#define   MACHINE1_DOOR_OPEN     0x0B	//����1���ſ�
//#define   MACHINE1_DOOR_CLOSE    0x0C	//����1���Ź�
//#define   MACHINE1_RESERVE1      0x0D	//����1Ԥ��1	 
//#define   MACHINE1_RESERVE2      0x0E	//����1Ԥ��2
//#define   MACHINE1_RESERVE3      0x0F	//����1Ԥ��3

/**-------0x1A-----IO����-���1---0x01 Y0--0x0F Y14--------**/
#define   IODEBUG_OUTPUT1_1      0x01	//
#define   IODEBUG_OUTPUT1_2      0x02	//
#define   IODEBUG_OUTPUT1_3      0x03	//
#define   IODEBUG_OUTPUT1_4      0x04	//
#define   IODEBUG_OUTPUT1_5      0x05	//
#define   IODEBUG_OUTPUT1_6      0x06	//
#define   IODEBUG_OUTPUT1_7      0x07	//
#define   IODEBUG_OUTPUT1_8      0x08	//
#define   IODEBUG_OUTPUT1_9      0x09	//
#define   IODEBUG_OUTPUT1_10     0x0A	//
#define   IODEBUG_OUTPUT1_11     0x0B	//
#define   IODEBUG_OUTPUT1_12     0x0C	//
#define   IODEBUG_OUTPUT1_13     0x0D	//	 
#define   IODEBUG_OUTPUT1_14     0x0E	//
#define   IODEBUG_OUTPUT1_15     0x0F	//


/**-------0x1B-----IO����-���2--- 0X01 Y15---0X0F Y29--------**/
#define   IODEBUG_OUTPUT2_1      0x01	//
#define   IODEBUG_OUTPUT2_2      0x02	//
#define   IODEBUG_OUTPUT2_3      0x03	//
#define   IODEBUG_OUTPUT2_4      0x04	//
#define   IODEBUG_OUTPUT2_5      0x05	//
#define   IODEBUG_OUTPUT2_6      0x06	//
#define   IODEBUG_OUTPUT2_7      0x07	//
#define   IODEBUG_OUTPUT2_8      0x08	//
#define   IODEBUG_OUTPUT2_9      0x09	//
#define   IODEBUG_OUTPUT2_10     0x0A	//
#define   IODEBUG_OUTPUT2_11     0x0B	//
#define   IODEBUG_OUTPUT2_12     0x0C	//
#define   IODEBUG_OUTPUT2_13     0x0D	//	 
#define   IODEBUG_OUTPUT2_14     0x0E	//
#define   IODEBUG_OUTPUT2_15     0x0F	//

/**-------0x1C-----IO����-���3--- 0X01 Y30---0X0F Y35--------**/
#define   IODEBUG_OUTPUT3_1      0x01	//
#define   IODEBUG_OUTPUT3_2      0x02	//
#define   IODEBUG_OUTPUT3_3      0x03	//
#define   IODEBUG_OUTPUT3_4      0x04	//
#define   IODEBUG_OUTPUT3_5      0x05	//
#define   IODEBUG_OUTPUT3_6      0x06	//



/**-------0x1B-----IO����-���2--------------**/
#define   MACHINE2_KAPA_OPEN     0x01	//����1����A��
#define   MACHINE2_KAPA_CLOSE    0x02	//����1����A��
#define   MACHINE2_CUIQI         0x03	//����1����
#define   MACHINE2_ALLOW_START   0x04	//����1��������
#define   MACHINE2_START_PROCESS 0x05	//����1��ʼ�ӹ�
#define   MACHINE2_DOOR_OPEN     0x06	//����1���ſ�
#define   MACHINE2_DOOR_CLOSE    0x07	//����1���Ź�
#define   MACHINE2_RESERVE1      0x08	//����1Ԥ��1
#define   MACHINE2_RESERVE2      0x09	//����1Ԥ��2
#define   MACHINE2_RESERVE3      0x0A	//����1Ԥ��3

#define   ROLL_OVER_UPRIGHT      0x0B	//��ת̨-����
#define   ROLL_OVER_REVERSE      0x0C	//��ת̨-����
#define   ROLL_OVER_OPEN         0x0D	//��ת̨-�ɿ�
#define   ROLL_OVER_CLOSE        0x0E	//��ת̨-�н�
#define   ROLL_OVER_RESERVE1     0x10	//��ת̨-Ԥ��1

/**-------0x1C-----IO����-���3--------------**/
#define   CLAW_LOAD_OPEN         0x01	//��צ-����צ��
#define   CLAW_LOAD_CLOSE        0x02	//��צ-����צ��
#define   CLAW_UNLOAD_OPEN       0x03	//��צ-ж��צ��
#define   CLAW_UNLOAD_CLOSE      0x04	//��צ-ж��צ��
#define   CLAW_BAIGANG_UPRIGHT   0x05	//��צ-�ڸ�����
#define   CLAW_BAIGANG_RESET     0x06	//��צ-�ڸ׸�λ
#define   CLAW_YCYLINDER_STRETCH 0x07	//��צ-Y���������
#define   CLAW_YCYLINDER_RETRACT 0x08	//��צ-Y����������
#define   CLAW_BLOW              0x09	//��צ-��ץ����
#define   CLAW_RESERVE1          0x0A	//��צ-Ԥ��1

#define   OTHERS_OILING_ENABLE   0x0B	//����-��ʹ��
#define   OTHERS_RUN_LIGHT       0x0C	//����-���е�
#define   OTHERS_WAIT_LIGHT      0x0D	//����-������
#define   OTHERS_ALARM_LIGHT     0x0E	//����-������
#define   OTHERS_RESERVE1        0x10	//����-Ԥ��1


/**-------0x1D-----�ֶ�����-----------------**/
#define	  XAXIS_MANUL_SPEED		 0x01	//X��-�ֶ��ٶ�ֵ
#define   XAXIS_MOVE_LEFT        0x02	//X��-����
#define   XAXIS_MOVE_RIGHT       0x03	//X��-����
#define   XAXIS_STEP_STOP        0x04	//X��-�綯ֹͣ
#define   XAXIS_SAVE_POINT       0x05	//X��-����
#define	  ZAXIS_MANUL_SPEED		 0x06	//Z��-�ֶ��ٶ�ֵ
#define   ZAXIS_MOVE_LEFT        0x07	//Z��-����
#define   ZAXIS_MOVE_RIGHT       0x08	//Z��-����
#define   ZAXIS_STEP_STOP        0x09	//Z��-�綯ֹͣ
#define   ZAXIS_SAVE_POINT       0x0A	//Z��-����
#define   XZAXIS_SAVE_POINT      0x0B	//XZ��-����
#define   XZAXIS_DELETE_POINT   0x0C	//XZ��-ɾ��
#define	  XAXIS_STEP_MM		     0x0D	//X��-�綯����mmΪ��λ
#define	  ZAXIS_STEP_MM		     0x0E	//Z��-�綯����mmΪ��λ
#define	  XAXIS_BACK_ORIGIN		 0x0F	//X��-����
#define	  ZAXIS_BACK_ORIGIN		 0x10	//Z��-����
#define   MANUL_FINE_ADJUST		 0x11	//΢������

#define	  LAXIS_MANUL_SPEED		 0x12	//L��-�ֶ��ٶ�ֵ
#define   LAXIS_MOVE_LEFT        0x13	//L��-����
#define   LAXIS_MOVE_RIGHT       0x14	//L��-����
#define   LAXIS_STEP_STOP        0x15	//L��-�綯ֹͣ
#define	  LAXIS_STEP_MM		     0x16	//L��-�綯����mmΪ��λ
#define   LAXIS_SAVE_POINT       0x17	//L��-����
#define	  LAXIS_BACK_ORIGIN		 0x18	//L��-����

#define	  OAXIS_MANUL_SPEED		 0x19	//O��-�ֶ��ٶ�ֵ
#define   OAXIS_MOVE_LEFT        0x1A	//O��-����
#define   OAXIS_MOVE_RIGHT       0x1B	//O��-����
#define   OAXIS_STEP_STOP        0x1C	//O��-�綯ֹͣ
#define	  OAXIS_STEP_MM		     0x1D	//O��-�綯����mmΪ��λ
#define   OAXIS_SAVE_POINT       0x1E	//O��-����
#define	  OAXIS_BACK_ORIGIN		 0x1F	//O��-����
#define   LOAXIS_DELETE_POINT   0x20	//LO��-ɾ��

#define	  VAXIS_MANUL_SPEED		 0x21	//V��-�ֶ��ٶ�ֵ
#define   AAXIS_MOVE_LEFT        0x22	//V��-����
#define   AAXIS_MOVE_RIGHT       0x23	//V��-����
#define   VAXIS_STEP_STOP        0x24	//V��-�綯ֹͣ
#define	  VAXIS_STEP_MM		     0x25	//V��-�綯����mmΪ��λ
#define   VAXIS_SAVE_POINT       0x26	//V��-����
#define	  VAXIS_BACK_ORIGIN		 0x27	//V��-����

#define	  WAXIS_MANUL_SPEED		 0x28	//W��-�ֶ��ٶ�ֵ
#define   BAXIS_MOVE_LEFT        0x29	//W��-����
#define   BAXIS_MOVE_RIGHT       0x2A	//W��-����
#define   WAXIS_STEP_STOP        0x2B	//W��-�綯ֹͣ
#define	  WAXIS_STEP_MM		     0x2C	//W��-�綯����mmΪ��λ
#define   WAXIS_SAVE_POINT       0x2D	//W��-����
#define	  WAXIS_BACK_ORIGIN		 0x2E	//W��-����

#define   E_GEARRATIO_SET			  0x2F//���ӳ��ֱ�����
#define   XYAXIS_MANUL_SPEED    0x30//ʾ�̽���XY�����ٶ�����
#define   ZOAXIS_MANUL_SPEED    0x31//ʾ�̽���ZO�����ٶ�����
#define   ABAXIS_MANUL_SPEED    0x32//ʾ�̽���AB�����ٶ�����
#define   XYAXIS_STEP_MM				0x33//ʾ�̽���XY�綯�ٶ�����
#define   ZOAXIS_STEP_MM				0x34//ʾ�̽���ZO�綯�ٶ�����
#define   ABAXIS_STEP_MM				0x35//ʾ�̽���AB�綯�ٶ�����
#define   P_ACTION_SPEEDADJ      0x36	//ң���ٶȵ���20181205-L

/**-------0x1E-----��������-----------------**/
#define	  PARAMETER_SOFT_LIMIT		   0x01	//����λ
#define   PARAMETER_SAFE_AREA        0x02	//��ȫ��
#define   PARAMETER_FUCTION_SET      0x03	//���ܲ���
#define   PARAMETER_IO_DETECT_SET    0x04	//���ܲ���


#define   ALARMHISTORY_CLEAR         0x07	//��ʷ�������


#define   JOG_MOVE_MODE          0x01	//�綯ģʽ
#define   LINK_MOVE_MODE         0x02	//����ģʽ

#define   NO_AXIS    0x00
#define   X_AXIS     0x01
#define   Z_AXIS     0x02
#define   L_AXIS     0x03
#define   O_AXIS     0x04
#define   V_AXIS     0x05
#define   W_AXIS     0x06

#define CURRENT_PROGRAM_ADR1  0x1200//��Ļ�ϴ���ʱ����ĳ����ַ1
#define CURRENT_PROGRAM_ADR2  0x7000//��Ļ�ϴ���ʱ����ĳ����ַ2

#define ORDER_LENTH  30       //ָ����ֽ�

#define LARGESAVEORDERNUM  32//�洢�������Ŀ����(�������ݴ���)
#define SAVEPROGRAMNUM  104	 //�洢�������Ŀ����
#define SAVEPROGRAMNUM_2  8	 //�洢�������Ŀ����-�ϲֳ���
#define SAVEPROGRAMNUM_4  6	 //�洢�������Ŀ����-���Ρ���λ���ϲֳ���
#define LC_MAXNUM       10    //�ϲ���Ŀ����



#define X_Axsis				  0x00	   //X��
#define Z_Axsis				  0x01	   //Z��
#define L_Axsis				  0x02	   //L��
#define O_Axsis				  0x03	   //O��
#define A_Axsis				  0x04	   //V��
#define B_Axsis				  0x05	   //W��


extern u8  X_Axsis_Origin_Enable ;	 //X���ԭ���ʶλ
extern u8  Z_Axsis_Origin_Enable ;	 //Z���ԭ���ʶλ
extern u8  Robot_Origining;
//extern u8  X_Axsis_Origining;
//extern u8  Z_Axsis_Origining ;
extern u8  X_Axsis_Origned;	   //X���ѻ�ԭ��
extern u8  Z_Axsis_Origned;	   //Z���ѻ�ԭ��
extern u8  Robot_Origined;	   //��е�ֻ�ԭ��
extern u8  Robot_Enable;
extern u8  Robot_Enable_Switch_On;	  
extern u8  Robot_Enable_Pre_Switch;
extern u8  Program_Ready;
extern u8  Data_Syc_Ready;
extern u8  ProgramInf_Syc_Ready;
extern u8  Program_Syc_Ready;
extern u8  SoftLimit_Syc_Ready;
extern u8  SafeArea_Syc_Ready;
extern u8  Point_Syc_Ready;
extern u8  Setting_Syc_Ready;
extern u8  IODetect_Syc_Ready;
extern u8  Main_Init_Done;		 //���ذ忪�����
extern u8  Robot_Status ;      //��е��״̬
extern u8  Robot_Auto_Mode;

extern u32 g_XAxis_Position ;    //X���λ��
extern u32 g_ZAxis_Position ;		 //Z���λ��
extern u32 g_LAxis_Position ;		 //L���λ��
extern u32 g_OAxis_Position ;		 //O���λ��
extern u32 g_AAxis_Position ;		 //V���λ��
extern u32 g_BAxis_Position ;		 //W���λ��
extern u32 g_Pre_XAxis_Position;   //X��֮ǰʱ�̵�λ��
extern u32 g_Pre_ZAxis_Position ;	 //Z��֮ǰʱ�̵�λ��
extern u32 g_Pre_LAxis_Position ;	 //L��֮ǰʱ�̵�λ��
extern u32 g_Pre_OAxis_Position ;	 //O��֮ǰʱ�̵�λ��
extern u32 g_Pre_VAxis_Position ;	 //V��֮ǰʱ�̵�λ��
extern u32 g_Pre_WAxis_Position ;	 //W��֮ǰʱ�̵�λ��
extern u8 Axsis_Move_Direction[6];//���˶�����
extern u8  g_Servo_Is_Run,g_Axis_Is_Run;
extern u8  g_Axis_Choose ;
//extern u16 g_Robot_Error;
extern u8  Robot_Error_Data[5],Robot_Axis_State[6];
extern u8 g_Robot_Error_Flag;

//��洢
typedef struct 
{
	u8  Flag;		//�Ƿ��е�
	u32 Name;		//�������
	u32 Point;		//X��ֵ
}SavePoint;

//������Ϣ�洢
typedef struct 
{
	u8  Flag;	   //�Ƿ��г���
	u8  Code;	   //����ı�ţ��·��������ư���ʶ����
	u32 Name;	   //N
	u16  Num;	   //S:Scale�������Ĺ�ģ��С����������ʱ��ʾ�ж�
	u16 Address;   //IIC�Ĵ洢��ַ
}SaveProgramInf;   //7

//����洢
typedef struct 
{
	u8  Flag;	   //�Ƿ��г���
	u8  Code;	   //����ı�ţ��·��������ư���ʶ����
	u32 Name;	   //N
	u16  Num;	   //S:Scale�������Ĺ�ģ��С����������ʱ��ʾ�ж�
    FreeProgram	Program[LARGESTPROGRAMNUM];
}SaveProgram;	   //7+1500

/***************���������Ϣ*********************************/
typedef struct 
{
	u8  Flag;	   //�Ƿ��г���
	u8  Code;	   //����ı�ţ��·��������ư���ʶ����
	u32 Name;	   //N
	u16  Order_Num;//ָ����Ŀ
	u32 Time;			//ʱ��
}SavedProgramInf;   //13

typedef struct
{
	u8  Flag;	   //�Ƿ��г��� 1�� 0��
	u8  Code;	   //����ı�ţ��·��������ư���ʶ����
	u16 XuHao;
	u32 Name;	   //����
	u16  Order_Num;//ָ����Ŀ
	u32 time;      //����
}Save_Program;	   
extern Save_Program OperateProgram[SAVEPROGRAMNUM];
extern SavePoint Manul_Save_PointX[25];
extern SavePoint Manul_Save_PointZ[25];
extern SavePoint Manul_Save_PointL[25];
extern SavePoint Manul_Save_PointO[25];
extern SaveProgramInf Free_Program_Save[10];
extern SaveProgram Current_Operate_Program;
extern SaveProgram Free_Program_Operate,Free_Program_Operate;
void USART1_ReceiveProgram(u16 Num,u8 Code);	    //���ͳ�����
u8 USART1_ReadProgram(u8);


extern u8  UnLock_Forever_flag;	//�Ƿ�����Ч��־λ
extern u16 Unlock_LastDay;		//����ʱ��
extern u8  Reset_Flag;
extern u32 MachineQueueNum;		//��ʱ��ͻ��������ɵĻ�������
extern u8  Check_battery;

extern SavedProgramInf Saved_Program[];


void InitLockTime(void);
void CheckTime(void);
void UnlockCodeInput(void);

void ReadIICInitPar(void);
void ReadIICAlarm(void);
void LinkMoveDataInit(void);
void Read_Point(void);

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

