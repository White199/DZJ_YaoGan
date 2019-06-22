/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __manual_h_
#define __manual_h_


#define  M_REVERSING_UP              0x01     //������
#define  M_REVERSING_DOWN            0x02     //����
#define  M_REVERSING_CLAMP           0x03     //����н�
#define  M_REVERSING_RELEASE         0x04     //�������
#define  M_REVERSING_TURN_FRONT      0x05     //����ǰ��
#define  M_REVERSING_TURN_BACK       0x06     //�����
#define  M_TRASPOSITION1             0x07     //תλ1
#define  M_TRASPOSITION2             0x08     //תλ2
#define  M_CLAW1_CLAMP               0x09     //��ץ1��
#define  M_CLAW1_RELEASE             0x0a     //��ץ1��
#define  M_CLAW2_CLAMP               0x0b     //��ץ2��
#define  M_CLAW2_RELEASE             0x0c     //��ץ2��
#define  M_MACHINE1_HOLDER_CLAMP     0x0d     //1�Ż��о߽�
#define  M_MACHINE1_HOLDER_RELEASE   0x0e     //1�Ż��о���
#define  M_MACHINE1_DOOR_OPEN        0x0f     //1�Ż����ſ�
#define  M_MACHINE1_DOOR_CLOSE       0x10     //1�Ż����Ź�
#define  M_MACHINE1_SEND_GAS         0x11     //1�Ż�����
#define  M_MACHINE1_SEND_MATERIAL    0x12     //1�Ż�����
#define  M_MACHINE2_HOLDER_CLAMP     0x13     //2�Ż��о߽�
#define  M_MACHINE2_HOLDER_RELEASE   0x14     //2�Ż��о���
#define  M_MACHINE2_DOOR_OPEN        0x15     //2�Ż����ſ�
#define  M_MACHINE2_DOOR_CLOSE       0x16     //2�Ż����Ź�
#define  M_MACHINE2_SEND_GAS         0x17     //2�Ż�����
#define  M_MACHINE2_SEND_MATERIAL    0x18     //2�Ż�����
#define  M_LOAD_MARERIAL_FINISH      0x19     //ȡ�Ͻ����ź�
#define  M_UNLOAD_MATERIAL_FINISH    0x1a     //ж�Ͻ����ź�
#define  M_X_CHOSEN                  0x1b     //x��ѡ��
#define  M_Z_CHOSEN                  0x1c     //z��ѡ��
#define  M_JOG_MODE                  0x1d     //�綯ģʽ
#define  M_LINKED_MODE               0x1e     //����ģʽ
#define  M_POSITIVE_MOVE             0x1f     //�����ƶ�
#define  M_NEGATIVE_MOVE             0x20    //�����ƶ�

									   
extern u8 X_Axsis_Chosen ;			  //X��ѡ�б�־λ
extern u8 Z_Axsis_Chosen ;			  //Z��ѡ�б�־λ

extern u8 Manual_Mode_Enable ;		  //�ֶ�ģʽʹ�ܱ�־λ
extern u8 Jog_Move_Enable ;			  //�綯���б�־λ
extern u8 Jog_Mode_Enable ;			  //�綯ģʽʹ�ܱ�־λ
extern u8 Linked_Mode_Enable;		  //����ģʽʹ�ܱ�־λ
extern u8 Linked_Move_Enable;		  //�������б�־λ
extern u8 g_Link_Move_Delay;
extern u8 g_Link_Move_Count;

extern u8  Send_Material_Finish_Flag;
extern u32 Send_Material_Finish_Time ;       //���϶�ʱ����
extern u32 Load_Material_Finish_Time ;		//ȡ�϶�ʱ����
extern u8  Load_Material_Finish_Flag	;		//ȡ�Ͻ�����־λ
extern u32 Unload_Material_Finish_Time ;		//ж�϶�ʱ����
extern u8  Unload_Material_Finish_Flag ;		//ж�Ͻ�����־λ

extern u8 Manual_Mode_Error_Flag;	  //�ֶ�����ģʽ�����־λ
extern u32 Jog_Pulse_Count ;		  //�綯ģʽ��ÿ��������������
extern u32 Jog_Pulse_Count_Init ;
extern u32 Linked_Pulse_Count ;		  //����ģʽ��ÿ�η���������
extern u32 Linked_Pulse;
extern u8  Test_Key_Flag;
extern u8  XAxis_Manul_Speed;			  //X���ֶ��ٶ�
extern u8  ZAxis_Manul_Speed;			  //Z���ֶ��ٶ�
extern u8  LAxis_Manul_Speed;			  //Z���ֶ��ٶ�
extern u8  OAxis_Manul_Speed;			  //Z���ֶ��ٶ�
extern u8  AAxis_Manul_Speed;			  //Z���ֶ��ٶ�
extern u8  BAxis_Manul_Speed;			  //Z���ֶ��ٶ�
extern u8  XAxis_Step_Distance;		  //X��綯����5-100,Ĭ��50mm
extern u8  ZAxis_Step_Distance;		  //Z��綯����5-100,Ĭ��50mm
extern u8  LAxis_Step_Distance;		  //Z��綯����5-100,Ĭ��50mm
extern u8  OAxis_Step_Distance;		  //Z��綯����5-100,Ĭ��50mm
extern u8  g_Manul_Fine_Adjust;		  //�ֶ�΢��

void IODebugOutput1(void);
void IODebugOutput2(void);
void IODebugOutput3(void);
extern void ManulDebug(void);		  //�ֶ�����
extern void ManualMode(void);		  //�ֶ�ģʽ
extern void ManualJogRunnig(void);	  //�ֶ������㶯ģʽ
extern void ManualLinkedRunning(void);//�ֶ���������ģʽ
//---
extern u32 Real_Time_Position[6];	  //�
extern vu32 Real_Time_Position_1[6];


void Order_Package(void);//ָ���װ

#endif

/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/

