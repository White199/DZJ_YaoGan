/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : ***.h
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 05/28/2015
* Description        : This is the ....
***************************************************************************************/

#ifndef __FREEPROGRAMME_H_
#define __FREEPROGRAMME_H_

#define LARGESTPROGRAMNUM 32	//������ָ����
#define LARGESTPOINTMNUM 104		//���������Ŀ


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
#define   K_OAXISRESET             0x07  //�ϲֶ�����λ

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
#define   K_IOINSTRUCT_OUTPUT23    0x2A	//���ָ��23-��λY11
#define   K_IOINSTRUCT_OUTPUT24    0x2B	//���ָ��24-��λY11
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
#define   K_IOINSTRUCT_OUTPUT49    0x44	//���ָ��49-��λY24-��λY25
#define   K_IOINSTRUCT_OUTPUT50    0x45	//���ָ��50-��λY24-��λY25
#define   K_IOINSTRUCT_OUTPUT51    0x46	//���ָ��51-��λY26-��λY27
#define   K_IOINSTRUCT_OUTPUT52    0x47	//���ָ��52-��λY27-��λY26
#define   K_IOINSTRUCT_OUTPUT53    0x48	//���ָ��53-��λY28-��λY29
#define   K_IOINSTRUCT_OUTPUT54    0x49	//���ָ��54-��λY28-��λY29
#define   K_IOINSTRUCT_OUTPUT55    0x4A	//���ָ��55-��λY30-��λY31
#define   K_IOINSTRUCT_OUTPUT56    0x4B	//���ָ��56-��λY30-��λY31
#define   K_IOINSTRUCT_OUTPUT57    0x4C	//���ָ��57-��λY32-��λY33
#define   K_IOINSTRUCT_OUTPUT58    0x4D	//���ָ��58-��λY32-��λY33
#define   K_IOINSTRUCT_OUTPUT59    0x4E	//���ָ��59-��λY34-��λY35
#define   K_IOINSTRUCT_OUTPUT60    0x4F	//���ָ��60-��λY34-��λY35


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

 
//���ɱ��
typedef struct 
{
	u8  Flag;  //��־λ
	u8  List;  //������
	u8  Order; //ָ������
	u8  Key;   //������
	u32 Value1;//����1-����XYZOVW
	u32 Value2;//����2-
	u32 Value3;//����3-
	u32 Value4;//����4-
	u32 Value5;//����5-
	u32 IO_Group;//IO����
	u8  LC_ID;//�ϲ�ѡ��
	u16 Delay_Time;//��ʱָ����ʱ
}FreeProgram;

//extern u8 g_Free_Program_Num;		 //��ǰѡ�����ɱ�̳���
//extern FreeProgram Current_Process_Program[LARGESTPROGRAMNUM];

extern u8 g_Refresh_Dispaly_List_Flag ;	 //ˢ�³�����ʾ�б�
//extern u8 gs_Current_Program_Num;
extern u8 gs_Current_Operate_List;
extern u8 ProgramCode_String[1001][4];
extern u8 Program_String[87][16];
extern u8 Parameter_Value[10];
extern u8 Parameter_String[24][10];
extern u8 g_Current_Program_Debug;			 //��ǰ�����Ƿ���Ե���
extern u8 g_Program_Is_Debuging;		     //���������,���а���ʧЧ
extern u8 g_USART1_SendProgram_Flag;

void LCDFreeProgramProcess(void);
void LCDFreeProgramDisply(void);
void ValueToString(u32);
void ProcessProgram(u8,u8);
void Delete_Order(void);
void Search_Order(void);
void Modify_Order(void);

void Delete_Point(void);
void Search_Point(void);
void Choose_Point(void);

void Select_Product(void);
void New_Product(void);
void Delete_Product(void);

void LCDProductManagementDisply(void);
void ValueToNumber(u16 Temp_Value);



#endif

