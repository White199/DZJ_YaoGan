/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __error_h_
#define __error_h_


#define NO_ERROR					0x00
#define ERROR_HAPPEN				0x01
#define ERROR_EMERG_HAPPEN			0x02
#define ERROR_STATUS                0x00  //�ᱨ���������ƽ��־λ  0x00�͵�ƽ��Ч  0x01�ߵ�ƽ��Ч

#define HARDLIMITJUDGE_EXTI         //Ӳ��λɨ�跽ʽ-�ⲿ�ж�
//#define HARDLIMITJUDGE_INPUT        //Ӳ��λɨ�跽ʽ-����ɨ��

#define EMERGENCY_STOP_ERROR		0x01
#define MACHINE_BREAKDOWN_ERROR     0x02
#define LACK_MATERIAL_ERROR         0x03
#define FULL_MATERIAL_ERROR         0x04 
//#define USART_IS_ERROR				0x05

extern u8 Error_Status;				  //����״̬
extern u16 Error_Data;				  //������������
extern u8 Robot_Error_Data[4];			  //��������
extern u8 Error_Enable_Flag; 			
extern u8 Limit_Error_Flag[4];
extern u8 Cancle_Limit_Warning;	
extern u8 Cancle_Genaral_Warning;
extern u8 Cancle_Genaral_Warning_Num;
extern u8 Limit_Error;	
extern void ErrorOperate(void);	      //�����⼰����
extern void EmergencyStopJudge(void);



#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/
