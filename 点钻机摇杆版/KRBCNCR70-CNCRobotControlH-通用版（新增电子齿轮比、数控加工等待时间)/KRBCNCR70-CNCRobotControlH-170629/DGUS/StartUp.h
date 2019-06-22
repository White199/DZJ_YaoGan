/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : ***.h
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 05/28/2015
* Description        : This is the ....
***************************************************************************************/

#ifndef __STARTUP_H_
#define __STARTUP_H_

#define DIS_STARTUP_PAGE    0x00	 //��ʾ��ǰҳ
#define DIS_AlARM_DISPLAY   0x01	 //������ʾ
#define DIS_ROBOT_ENABLE    0x02	 //ʹ��
#define DIS_ROBOT_DISABLE   0x03	 //ʧ��
#define DIS_EMERGENCY_STOP  0x04	 //��ͣ
#define DIS_NOEMERGENCY     0x05	 //δ��ͣ
#define DIS_CYCLE_RUN       0x06	 //ѭ��
#define DIS_SINGLE_RUN      0x07	 //����
#define DIS_X_POSITION      0x08	 //Xλ��
#define DIS_Z_POSITION      0x09	 //Zλ��
#define DIS_ORIGIN_Y        0x0A	 //����Ч��
#define DIS_ORIGIN_N        0x0B	 //������Ч��
#define DIS_RUN_Y           0x0C	 //����Ч��
#define DIS_RUN_N  		    0x0D	 //������Ч��
#define DIS_PAUSE_Y         0x10	 //��ͣЧ��
#define DIS_PAUSE_N 	    0x11	 //��ͣ��Ч��
#define DIS_STOP_Y          0x12	 //ֹͣЧ��
#define DIS_STOP_N 	        0x13	 //ֹͣ��Ч��
#define DIS_PROGRAM	        0x14	 //��ʾ����



#define AUTO_WAITE				0
#define AUTO_RUNNING			1
#define AUTO_PAUSE				2
#define AUTO_ERROR				3

extern u8 g_Auto_Status;
extern u8 Landing_Success ; 
extern u8  Change_User;
extern u8 gs_Run_Program_Num;


extern u8 Send_String_DL[];

void LCDStartUpDisply(void);
void LCDStartUpProcess(void);
void LandingProcess(void);
void BackToOrigin(void);
















#endif

