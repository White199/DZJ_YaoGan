/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __auto_h_
#define __auto_h_


#define ONCE_MODE   0x01	  //����ģʽ
#define SINGLE_MODE 0x02    //����ģʽ
#define LOOP_MODE   0x03	  //ѭ��ģʽ
#define AUTO_START  0x04	  //����
#define AUTO_STOP   0x05	  //ֹͣ

#define RUN    0x00	   //����
#define PAUSE  0x01	   //��ͣ
#define STOP   0x02    //ֹͣ


//#define INTERNAL_POSITION  5000 //�ϲ���ȡ�ϺͰڷų�Ʒ�������

extern u32 Auto_Pulse_Count;
extern u8 Auto_Run_Enable ;		  //�Զ�ģʽʹ�ܱ�־λ
extern u8 Auto_Mode;			  //�Զ�ģʽ�µ�ģʽѡ��
extern u8 Stop_Time;
extern u8 Stop_Status;
extern u8 Single_Mode_Enable;
extern u8 Once_Mode_Enable;
extern u8 Loop_Mode_Enable;
extern u8 Section_Action_Complete;
extern u8 JHRobot_Section;
extern u8 Last_JHRobot_Section;
extern u32 JHRobot_Section_Time;
extern u8 JHRobot_Section_Delay;
extern u8 Section_Step;
extern u8 Manchine_Door_Flag;
extern u8 JHRobot_Warning;
extern u32 Max_Robot_Position;
extern u32 Min_Robot_Position;


extern u32 Action_Delay_Time;         //��е��ÿ�ζ������֮�����ʱ����Ϊ�ŷ��������Ǹ��涯��
extern u8  Puls_Delay_Time[50];		 //��������ź���ʱ
extern u8  Puls_Delay_Enable[50];	     //���������ʱʹ�ܱ�־λ
extern u8  Puls_Delay_Num;
extern u8  Action_Done_Flag ;		 //�ŷ���������ɱ�־
extern u8  Action_Delay_Flag;		 //������ʱ���
extern u8  Auto_Action_Delay;	 //ж�Ͻ������ζ�����ʱ
extern u32 Interval_Position;
extern u8  Load_Material_Row_Number;
extern u32 Total_Product_Number;
extern u32 Product_Number;		  //��Ʒ�ӹ�����
extern u32 Check_Number;			  //��Ʒ�������
extern u32 Test_Pulse_Xaxsis;	 //����X�ᶪ����
extern u8  Test_Pulse_Xaxsis_Flag;

/* ��Ʒȡԭ�Ϻͷų�Ʒλ�� */
extern u32 Product_Position[10];
//extern u32 Interval_Position;	
extern u8  Holder_Delay_Flag;
extern u16 Holder_Delay_Time;		      //�о���ʱ
extern u8  Holder_Delay_Done;		  //�о���ʱ��ɱ�־λ
extern u16 Holder_Delay;				  //�о��趨��ʱʱ��

extern u8  Claw_Delay_Flag;		  //�о���ʱ��־λ
extern u16 Claw_Delay_Time;		      //�о���ʱ
extern u8  Claw_Delay_Done;		  //�о���ʱ��ɱ�־λ
extern u16 Claw_Delay;				  //�о��趨��ʱʱ��

//��������ź���ʱ����
extern u16 Output_Delay_Time;		      //���������ʱ

extern u8 g_Robot_Has_Debuged_Flag;
extern u8 g_Program_Is_Debuging;


void Read_SaveProgram_IIC_Address(void);
void AutoRun(void);
extern void AutoMode(void);
extern void AutoRunning(void);
extern void AutoReset(void);
extern void AutoStart(void);
extern void AutoPause(void);
extern void AutoStop(void);
extern void SafeAreaJudge(void);
extern void SetSingle(u8,u8,u32);
void MoveToTarget(u32,u8,u32);
void RobotAutoReset(void);

#endif


/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/


