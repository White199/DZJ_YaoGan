/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : Auto_2.h
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 21/10/2015
* Description        : �Զ�����ͷ�ļ�
***************************************************************************************/
#ifndef _Auto_2_h_
#define _Auto_2_h_

#define AUTO_WAITE				0
#define AUTO_RUNNING			1
#define AUTO_PAUSE				2
#define AUTO_ERROR				3


typedef struct 
{
	u8  Flag;	   //
	u8  Line;	   //
}ActionStepList;
#define LISTNUM 6


extern u8  g_AutoStatue;
extern u16  g_Auto_PresentLine;
extern u32 g_Auto_Circle_Timer;
extern u32 g_Auto_ProduceCounter;
extern u8  g_LC_Program_Step_Run;
extern u8  g_LC_Step_Control;
extern u8  g_LC_Load_Finish;			
extern u8  g_LC_Load_Permit;		
extern u8  g_LC_UnLoad_Finish;	
extern u8  g_LC_UnLoad_Permit;	  
extern u8  g_Read_LC_Load_Program;
extern u8  g_Read_LC_UnLoad_Program;
extern u8  g_LC_PresentLine;
extern u8  g_Start_Action_PresentLine;
extern u8  g_Start_Step_Control;
extern u8  g_Reset_Action_PresentLine;
extern u8  g_Reset_Step_Control;

extern u8  g_Auto_Order_Start;
extern u8  g_Auto_Order_Pause;
extern u8  g_Auto_Order_Stop;
extern u8  g_Auto_Order_StackReset;


extern u32 g_Auto_ActionRun_Timer;
extern u8  g_Auto_ActionRun_Step;
extern u32 g_Auto_ActionTimeOut_Time;
extern u8  g_Auto_ActionTimeOut_Flag;
extern u8  g_Auto_ActionError_Flag;
extern u8  g_Auto_WorkFinished_Flag;
extern u8  g_Material_Load_Disk_Finished;
extern u8  g_Material_UnLoad_Disk_Finished;

extern u32 g_LC_ActionRun_Timer;
extern u8  g_LC_ActionRun_Step;
extern u32 g_LC_ActionTimeOut_Time;
//extern u8  g_LC_ActionTimeOut_Flag;
extern u8  g_Start_ActionRun_Step;
extern u8  g_Reset_ActionRun_Step;

extern ActionStepList Action_Step_List[LISTNUM];
extern u8  Action_Step_List_Num;
extern u8  Action_Step_Run_Num;
extern u8  Action_Step_Confirm_Num;
extern u8  g_Material_Disk_Mode;
extern u8  g_Read_Load_LC_Position;
extern u8  g_Read_UnLoad_LC_Position;
extern u16 g_Load_LC_Key;
extern u32 g_Load_LC_Present_Position_X;		
extern u32 g_Load_LC_Present_Position_L;	
extern u8  g_Load_LC_Position_XKey;
extern u8  g_Load_LC_Position_LKey;

extern u16 g_UnLoad_LC_Key;
extern u32 g_UnLoad_LC_Present_Position_X;		
extern u32 g_UnLoad_LC_Present_Position_L;	
extern u8  g_UnLoad_LC_Position_XKey;
extern u8  g_UnLoad_LC_Position_LKey;

void AutoModeControl(void);
void AutoPauseOperate(void);



//#define LARGESTPROGRAMNUM 100	//������ָ����

//��Ҫָ��--order------//
#define   OR_BASICORDER      0x01  //����ָ��
#define   OR_AXISORDER       0x02  //���ָ��
#define   OR_IOORDER         0x03  //IOָ��

//������---key------//
#define   K_PROGRAMSTART           0x01	 //������ʼ
#define   K_PROGRAMEND             0x02	 //���������
#define   K_DELAY                  0x03	 //��ʱ
#define   K_LOADPROGRAM            0x04  //���ϲֳ���
#define   K_UNLOADPROGRAM          0x05  //ж�ϲֳ���
#define   K_STARTACTION            0x06  //�������ζ���
#define   K_OAXISRESET             0x07  //�ѵ���ɶ���

#define   K_ZAXISPOSITION          0x0C	 //��ֱ������
#define   K_OAXISPOSITION          0x0D	 //�ϲ�������
#define   K_LOADPOSITION           0x0E	 //��������
#define   K_UNLOADPOSITION         0x0F	 //ж������
#define   K_XAXIS                  0x10	 //X��
#define   K_ZAXIS                  0x11	 //Z��
#define   K_LAXIS                  0x12	 //L��
#define   K_OAXIS                  0x13	 //O��


//---20161111------
#define   K_IOINSTRUCT_OUTPUT1     0x14	//���ָ��1-��λY0
#define   K_IOINSTRUCT_OUTPUT2     0x15	//���ָ��2-��λY0
#define   K_IOINSTRUCT_OUTPUT3     0x16	//���ָ��3
#define   K_IOINSTRUCT_OUTPUT4     0x17	//���ָ��4
#define   K_IOINSTRUCT_OUTPUT5     0x18	//���ָ��5
#define   K_IOINSTRUCT_OUTPUT6     0x19	//���ָ��6
#define   K_IOINSTRUCT_OUTPUT7     0x1A	//���ָ��7
#define   K_IOINSTRUCT_OUTPUT8     0x1B	//���ָ��8
#define   K_IOINSTRUCT_OUTPUT9     0x1C	//���ָ��9
#define   K_IOINSTRUCT_OUTPUT10    0x1D	//���ָ��10
#define   K_IOINSTRUCT_OUTPUT11    0x1E	//���ָ��11
#define   K_IOINSTRUCT_OUTPUT12    0x1F	//���ָ��12
#define   K_IOINSTRUCT_OUTPUT13    0x20	//���ָ��13
#define   K_IOINSTRUCT_OUTPUT14    0x21	//���ָ��14
#define   K_IOINSTRUCT_OUTPUT15    0x22	//���ָ��15
#define   K_IOINSTRUCT_OUTPUT16    0x23	//���ָ��16
#define   K_IOINSTRUCT_OUTPUT17    0x24	//���ָ��17
#define   K_IOINSTRUCT_OUTPUT18    0x25	//���ָ��18
#define   K_IOINSTRUCT_OUTPUT19    0x26	//���ָ��19
#define   K_IOINSTRUCT_OUTPUT20    0x27	//���ָ��20
#define   K_IOINSTRUCT_OUTPUT21    0x28	//���ָ��21-��λY10
#define   K_IOINSTRUCT_OUTPUT22    0x29	//���ָ��22-��λY10
#define   K_IOINSTRUCT_OUTPUT23    0x2A	//���ָ��23
#define   K_IOINSTRUCT_OUTPUT24    0x2B	//���ָ��24
#define   K_IOINSTRUCT_OUTPUT25    0x2C	//���ָ��25
#define   K_IOINSTRUCT_OUTPUT26    0x2D	//���ָ��26
#define   K_IOINSTRUCT_OUTPUT27    0x2E	//���ָ��27
#define   K_IOINSTRUCT_OUTPUT28    0x2F	//���ָ��28
#define   K_IOINSTRUCT_OUTPUT29    0x30	//���ָ��29
#define   K_IOINSTRUCT_OUTPUT30    0x31	//���ָ��30
#define   K_IOINSTRUCT_OUTPUT31    0x32	//���ָ��31
#define   K_IOINSTRUCT_OUTPUT32    0x33	//���ָ��32
#define   K_IOINSTRUCT_OUTPUT33    0x34	//���ָ��33
#define   K_IOINSTRUCT_OUTPUT34    0x35	//���ָ��34
#define   K_IOINSTRUCT_OUTPUT35    0x36	//���ָ��35
#define   K_IOINSTRUCT_OUTPUT36    0x37	//���ָ��36
#define   K_IOINSTRUCT_OUTPUT37    0x38	//���ָ��37
#define   K_IOINSTRUCT_OUTPUT38    0x39	//���ָ��38
#define   K_IOINSTRUCT_OUTPUT39    0x3A	//���ָ��39-��λY19
#define   K_IOINSTRUCT_OUTPUT40    0x3B	//���ָ��40-��λY19

//�˶�ָ��Ϊ��·��˫·Ԥ��ָ������
#define   K_IOINSTRUCT_OUTPUT41    0x3C	//���ָ��41
#define   K_IOINSTRUCT_OUTPUT42    0x3D	//���ָ��42
#define   K_IOINSTRUCT_OUTPUT43    0x3E	//���ָ��43
#define   K_IOINSTRUCT_OUTPUT44    0x3F	//���ָ��44
//�˶�ָ��Ϊ��·��˫·Ԥ��ָ������

#define   K_IOINSTRUCT_OUTPUT45    0x40	//���ָ��45-��λY20-��λY21
#define   K_IOINSTRUCT_OUTPUT46    0x41	//���ָ��46-��λY20-��λY21
#define   K_IOINSTRUCT_OUTPUT47    0x42	//���ָ��47-��λY22-��λY23
#define   K_IOINSTRUCT_OUTPUT48    0x43	//���ָ��48-��λY22-��λY23
#define   K_IOINSTRUCT_OUTPUT49    0x44	//���ָ��39-��λY24-��λY25
#define   K_IOINSTRUCT_OUTPUT50    0x45	//���ָ��40-��λY24-��λY25
#define   K_IOINSTRUCT_OUTPUT51    0x46	//���ָ��41-��λY26-��λY27
#define   K_IOINSTRUCT_OUTPUT52    0x47	//���ָ��42-��λY26-��λY27
#define   K_IOINSTRUCT_OUTPUT53    0x48	//���ָ��43-��λY28-��λY29
#define   K_IOINSTRUCT_OUTPUT54    0x49	//���ָ��44-��λY28-��λY29
#define   K_IOINSTRUCT_OUTPUT55    0x4A	//���ָ��45-��λY30-��λY31
#define   K_IOINSTRUCT_OUTPUT56    0x4B	//���ָ��46-��λY30-��λY31
#define   K_IOINSTRUCT_OUTPUT57    0x4C	//���ָ��47-��λY32-��λY33
#define   K_IOINSTRUCT_OUTPUT58    0x4D	//���ָ��48-��λY32-��λY33
#define   K_IOINSTRUCT_OUTPUT59    0x4E	//���ָ��49-��λY34-��λY35
#define   K_IOINSTRUCT_OUTPUT60    0x4F	//���ָ��50-��λY34-��λY35


#define   K_IOINSTRUCT_INPUT1      0x50	//����ָ��1
#define   K_IOINSTRUCT_INPUT2      0x51	//����ָ��2
#define   K_IOINSTRUCT_INPUT3      0x52	//����ָ��3
#define   K_IOINSTRUCT_INPUT4      0x53	//����ָ��4
#define   K_IOINSTRUCT_INPUT5      0x54	//����ָ��5
#define   K_IOINSTRUCT_INPUT6      0x55	//����ָ��6
#define   K_IOINSTRUCT_INPUT7      0x56	//����ָ��7
//#define   K_IOINSTRUCT_INPUT8      0x57	//����ָ��8
//#define   K_IOINSTRUCT_INPUT9      0x58	//����ָ��9
//#define   K_IOINSTRUCT_INPUT10     0x59	//����ָ��10



#define   A_QuLiao								 0x03  //ȡ��
#define   A_DianJiao							 0x01  //�㽺
#define   A_DianZuan							 0x02  //����
#define   A_KongYi								 0x05  //����
#define   A_Delay								 0x04  //��ʱ

#endif

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/

