/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : ***.h
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 05/28/2015
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x.h"

#ifndef __ALARMHISTORY_H_
#define __ALARMHISTORY_H_


typedef struct{
	u8  Flag;  //�Ƿ��д���
	u16 year;   //����
	u8  month;  //ʱ��
	u8  day;
	u8  week;
	u8  hour;
	u8  minute;
	u8  second;
	u8  Type;  //��������
}AlarmHistory;
//extern AlarmHistory Alarm_History_Display;

typedef struct{
	u8  Flag; 
	u8  Day[10];
 	u8  Time[8];
	u8  Value;
	u8  Type;  //��������
}AlarmHistoryDisplay;

//extern u8  Alarm_Code;			 //�������
extern u8  Alarm_String[40][30]; //��ʾ�����ĵ�
extern u8  g_Current_Alarm;		 //��ǰ��������-���
extern u8 g_Alarm_Buzzer;
extern u8  g_Alarm_Buzzer_Timer;
extern u8 g_Alarm_Buzzer_Count;
extern AlarmHistoryDisplay Alarm_History_Display[21];
extern AlarmHistory Alarm_History_Time[20];
void LCDAlarmHistoryProcess(void);
void LCDAlarmHistoryDisply(void);
void LCDError(void);				//����1�������














#endif

