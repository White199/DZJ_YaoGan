/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#ifndef __statuscontrol_h_
#define __statuscontrol_h_

#define WAIT_MODE			  0x00     //�ȴ�ģʽ
#define AUTO_WORK_MODE        0x01	   //ѡ���Զ�����ģʽģʽ
#define MANUAL_WORK_MODE      0x02	   //ѡ���ֶ�����ģʽģʽ
#define FREE_PROGRAM_MODE	  0x03     //ѡ�����ɱ��ģʽ
#define IO_DEBUG_MODE		  0x04	   //ѡ��IO����ģʽ


/**----------------����ƶ�����------------------**/
#define NEGATIVE              0x00	   //����
#define	POSITIVE              0x01	   //����
#define NONE                  0x02	   //

#define X_Axsis				  0x00	   //X��
#define Z_Axsis				  0x01	   //Z��
#define L_Axsis				  0x02	   //L��
#define O_Axsis				  0x03	   //O��
#define A_Axsis				  0x04	   //V��
#define B_Axsis				  0x05	   //W��
//��ԭ�㷽ʽ
//#define Z_X_TYPE			  0x00	  //Z_X
//#define Z_X_L_TYPE			  0x01	  //Z_X_L
//��ԭ�㷽ʽ
#define FOM_Z_X     0
#define FOM_X_Z     1
#define FOM_O_X     2
#define FOM_Z_X_L   3
#define FOM_Z_L_X   4
#define FOM_Z_L_X_O 5
#define FOM_Z_X_L_O 6

//���ٶȵȼ�
#define FA_RANK1    0
#define FA_RANK2    1
#define FA_RANK3    2

extern u8  Synchronize;				  //���ֿ���ͬ����־λ
extern u8  Origin_Backed;			  //��ԭ�����
extern u8  X_Origin_Backed; 		  //X��ﵽԭ���־λ
extern u8  Z_Origin_Backed;   		  //Z��ﵽԭ���־λ
extern u8  L_Origin_Backed;   		  //L��ﵽԭ���־λ
extern u8  O_Origin_Backed;   		  //O��ﵽԭ���־λ
extern u8  Back_Origin_Flag; 		  //��ԭ�������־λ
extern u8  Origin_Speed_Down;         //��ԭ���⵽�źź�״̬Ϊ����״̬��־λ
extern u8  Initialize_Finished;		  //��ʼ�����
extern u32 Back_Origin_Speed;		  //��ԭ���ٶ�
extern u8  Robot_Enable_Flag;		  //��е��ʹ��
//extern u8  Origin_Type;				  //��ԭ�㷽ʽ

//extern u8 Servo_Is_Run ;			  //�ŷ����������б�־λ
extern u8 Work_Status ;				  //����״̬
extern u32 Input_Detect_Time;
extern u8 Input_Detect_Enable;
extern u8 Action_Detect_Flag;
extern u8 Jog_Pause_Enable; 
extern u8 Position_Handle_Flag;
//extern u8 Linked_Running_FLAG ;     //��������ģʽ
extern u8 SafeArea_Is_On;
extern u8 X_In_SafeArea_Flag;
extern u8 Safe_Area_Flag;
extern u8 Safe_Area_Alarm;		      //��ȫ���򱨾���������ȫ��
extern u8 g_Current_SafeArea_Num;	  //��ȫ�����
extern u8 Input15_16_Detect;
extern u8 Axis_Manul_BackToOrigin;    //�ֶ�����
extern u8 Axsis_Chosen;				  //�˶���ѡ��
extern u8 Axsis_Move_Direction[6] ;	  //�˶��᷽��ѡ��
extern u8 Input_Count15;
extern u8 Input_Count16;
extern u8 Servo_Stop_Done[6];
extern u8 Test_ServoStop;
extern u8 Timer_Enable_X;
extern u8 Timer_Enable_Z;
extern u8 Timer_Enable_L;
extern u8 Timer_Enable_O;
extern u8 g_Auto_Reset_Flag;
extern u8 Robot_Auto_Reset;
extern u8 g_Auto_LOrigin_Flag;
extern u32 g_USART_Delay_Timer;
void WorkMode(void);
extern void ActionControl(void);	  //��������
extern void DirectionJudge(u8);	  //�жϻ�е�ᶯ������
extern void StatusControl(void);	  //����״̬���
extern void Robot_Reset(void);
extern void SafeAreaDetection(void);

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

