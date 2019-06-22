/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __speedcontrol_h_
#define __speedcontrol_h_


/*******�ŷ�����***********/
#define DIRECTION_POSITIVE	0
#define DIRECTION_NEGATIVE	1

/**---------�ŷ�������������ж�����ģʽ---------**/
#define AUTORUNNINGMODE       0x00	  //�Զ�����ģʽ
#define JOGRUNNINGMODE        0x01	  //�ֶ��綯����ģʽ
#define LINKEDRUNNINGMODE	  0x02    //�ֶ���������ģʽ
#define BACKTOORIGIN		  0x03	  //��ԭ��ģʽ

#define SPEED_UP		0x01        //�����ٶ�
#define SPEED_HOLD		0x02        //�����ٶ�
#define SPEED_DOWN		0x03        //�����ٶ�

//#define PLUS_ACCEL		0x01        //�Ӽ��ٶ�
//#define HOLD_ACCEL		0x02        //�ȼ��ٶ�
//#define MINUS_ACCEL		0x03        //�����ٶ�
//#define ZERO_ACCEL		0x04        //����ٶ�-���ֶ�

//36000:1k 360:100k
#define PULSE_MINHALFPERIOD 180	   //����ٶȶ�Ӧ��С������



extern u16 *SpeedUp_Jerk;
extern u16 *SpeedUp_Max_Acceleration;
extern u16 SpeedUp_Jerk1[10];
extern u16 SpeedUp_Max_Acceleration1[10];
extern u16 SpeedUp_Jerk2[10];
extern u16 SpeedUp_Max_Acceleration2[10];
extern u16 SpeedUp_Jerk3[10];
extern u16 SpeedUp_Max_Acceleration3[10];
extern u16 Present_SpeedUp_Jerk[6];               
extern u16 Present_SpeedUp_Max_Acceleration[6]; 
extern u32 Present_Pulse_SpeedControlStep_Counter[6];
// extern u16 SpeedUp_Max_A;6.29
//--6.29
extern u16 SpeedUp_Max_A[6];
extern u16 SpeedUp_Max_A_Z;
extern u16 SpeedUp_Max_A_X;
extern u16 SpeedUp_Max_A_O;
extern u16 SpeedUp_Max_A_Y;

extern u16 SpeedUp_Max_Acceleration_Sheet[81];

extern u8  Timer_Enable_X;
extern u8  Timer_Enable_Z;
extern u8  Timer_Enable_L;
extern u8  Timer_Enable_O;
extern u8  Timer_Enable_A;
extern u8  Timer_Enable_B;
extern u32 Servo_Pulse_Count[6];	      //���ŷ���������������
extern u8  Servo_Control_Mode ; 		  //�ŷ�������ģʽ״̬
extern u32 Pulse_Counter[6] ;
extern u32 Pulse_ControlStep[6];
extern u32 SpeedUp_Pulse_Counter[6];
//extern u32 Pulse_MinHalfPeriod;		  // ����ٶ�����ֵ
extern u32 Pulse_Maxspeed_HalfPeriod[6]; // ����ٶȶ�Ӧ������
extern u32 PULSE_MINHALF_PERIOD;
extern u32 Pulse_InitialHalfPeriod[6] ;
extern u32 Pulse_TotalCounter[6];
extern u16 Pulse_PresentAcceleration[6];

extern void SendPulse(u8);  //�������庯��


#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

