/******************** (C) COPYRIGHT 2011 YWPOLES ********************
---------------------------------------------------------------------------------------------------------
*********************************************************************************************************
** �ļ����� ��	KEY.H
** ����˵�� ��	�������弰��������
** ʹ��˵�� ��  �ļ�������STM32F103Ӳ��ƽ̨  
*********************************************************************************************************
---------------------------------------------------------------------------------------------------------
*/
#ifndef _KEY_H_
#define _KEY_H_

#define KeyCntTh  		2	//����ѭ��������
#define KeyStatusIdle	0	//����״̬��û�а������¡�����ֵ�ѱ���ȡ
#define KeyStatusPush	1	//��������״̬
#define KeyStatusWaite	2	//������ɶ������ȴ���ȡ


#define KEY_ERROR      0
#define KEY_PHYSICS1   1
#define KEY_PHYSICS2   2
#define KEY_PHYSICS3   3
#define KEY_PHYSICS4   4
#define KEY_PHYSICS5   5
#define KEY_PHYSICS6   6
#define KEY_PHYSICS7   7
#define KEY_PHYSICS8   8
#define KEY_PHYSICS9   9
#define KEY_PHYSICS10  10
#define KEY_PHYSICS11  11
#define KEY_PHYSICS12  12
#define KEY_PHYSICS13  13
#define KEY_PHYSICS14  14
#define KEY_PHYSICS15  15
#define KEY_PHYSICS16  16
#define KEY_PHYSICS17  17
#define KEY_PHYSICS18  18
#define KEY_PHYSICS19  19
#define KEY_PHYSICS20  20
#define KEY_PHYSICS21  21
#define KEY_PHYSICS22  22
#define KEY_PHYSICS23  23
#define KEY_PHYSICS24  24
#define KEY_PHYSICS25  25
#define KEY_PHYSICS26  26
#define KEY_PHYSICS27  27
#define KEY_PHYSICS28  28

#define KEY_CROSSOUT_C	29
#define KEY_CROSSIN_C	30

#define KEY_FUNCTION	   	KEY_PHYSICS1
#define KEY_PROGRAM		   	KEY_PHYSICS2
#define KEY_PARAMETER	   	KEY_PHYSICS3
#define KEY_INFO		   	KEY_PHYSICS4
#define KEY_NUMBER1	   		KEY_PHYSICS5
#define KEY_NUMBER2	   		KEY_PHYSICS6
#define KEY_NUMBER3	   		KEY_PHYSICS7
#define KEY_WATCH	   		KEY_PHYSICS8
#define KEY_NUMBER4	   		KEY_PHYSICS9
#define KEY_NUMBER5	   		KEY_PHYSICS10
#define KEY_NUMBER6	   		KEY_PHYSICS11
#define KEY_UP	   			KEY_PHYSICS12
#define KEY_NUMBER7	   		KEY_PHYSICS13
#define KEY_NUMBER8	   		KEY_PHYSICS14
#define KEY_NUMBER9	   		KEY_PHYSICS15
#define KEY_DOWN	   		KEY_PHYSICS16
#define KEY_HP				KEY_PHYSICS17
#define KEY_NUMBER0			KEY_PHYSICS18
#define KEY_OUT	   			KEY_PHYSICS19
#define KEY_ENTER  			KEY_PHYSICS20
#define KEY_SP				KEY_PHYSICS21
#define KEY_SWITCH	   		KEY_PHYSICS22
#define KEY_INS		   		KEY_PHYSICS23
#define KEY_DEL		   		KEY_PHYSICS24
#define KEY_MANUAL	   		KEY_PHYSICS25
#define KEY_AUTO	   		KEY_PHYSICS26
#define KEY_RUN		   		KEY_PHYSICS27
#define KEY_STOP	   		KEY_PHYSICS28


extern u16 keyTempValue,keyValueStatus;
extern u16 keyFinishFlag;
extern u16 g_Key_Value;
extern u8  g_Key_Scan;
extern u8  g_Key_Scan_Count;
extern u8  Key_Scan_Delay_Count;
extern void KeyInit(void);
extern void KeyOutput7(u16 key);
extern u16 KeyInput4(void);
extern u16 KeyScan(void);
extern u16 KeyValue2Physics(u16 keyValueTemp);
extern u16 ReadKey(void);
extern void KeyProcess(void);

void  RodProcess(void);
void RodValueProcess(void);

extern u16 cntKeyScan;
extern u32 g_LCD_Off_Timer;

#endif
/******************* (C) COPYRIGHT 2011 YWPOLES *****END OF FILE****/


