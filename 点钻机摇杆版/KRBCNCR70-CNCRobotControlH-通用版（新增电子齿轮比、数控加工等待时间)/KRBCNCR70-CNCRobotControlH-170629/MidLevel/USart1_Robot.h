/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : 
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 
* Description        : This is the ....
***************************************************************************************/

#ifndef __USART1_ROBOT_H_
#define __USART1_ROBOT_H_

/*-------�������ظ�����궨��-MainBoard Reply-------------*/
#define M_ROBOT_ENABLE_A_ORIGIN  0xA0	//��е��ʹ�ܺͻ�ԭ��
#define M_WORK_MODE              0xB0	//��е�ֹ���ģʽ
#define M_AUTO_RUN               0xC0	//��е���Զ�����
#define M_FREE_PROGRAM_SEND      0xD0	//���ɱ�̳����·�
#define M_FREE_PROGRAM_RECEIVE	 0xE0 //��Ʒѡ��������

//�����ظ�
#define M_IO_DEBUG_OUTPUT1        0xA1	//IO����-����ظ�
#define M_IO_DEBUG_OUTPUT2        0xB1	//IO����-����ظ�
#define M_IO_DEBUG_OUTPUT3        0xC1	//IO����-����ظ�
#define M_MANUL_OPERATE			  0xD1	//�ֶ����Իظ�
#define M_PARAMETER_ORDER		  0xE1	//��������ظ�

//��������ظ�
#define M_IO_DEBUG_LOCAL         0xE1	//IO����-����IO
#define M_IO_DEBUG_INPUT1        0xE2	//IO����-����1
#define M_IO_DEBUG_INPUT2        0xE3	//IO����-����2
#define M_IO_DEBUG_INPUT3        0xE4	//IO����-����3
#define M_IO_DEBUG_OUTPUT1_LCD   0xE5	//IO����-���1����
#define M_IO_DEBUG_OUTPUT2_LCD   0xE6	//IO����-���2����
#define M_IO_DEBUG_OUTPUT3_LCD   0xE7	//IO����-���3����
#define M_ROBOT_ORIGINED		 0xE8	//��е���Ƿ��ѻ�ԭ��
#define M_ACTION_RESET_SCAN	     0xE9	//��е�ָ�λ��ѯ
#define M_ALARM_CLEAR    	     0xEA	//�����ǰ������־λ
#define M_ROBOT_STATUS      	 0xEB	//״̬��ѯ:��е��״̬����е������״̬��������Ϣ
#define M_AUTO_PARAMETER         0xEC	//
#define M_X_AXSIS_POSITION	     0xED	//X������
#define M_Z_AXSIS_POSITION       0xEE	//Z������
#define M_ROBOT_PRE_STATUS       0xEF	//��е������״̬
#define M_ROBOT_DEBUG_STATUS     0xF0	//��е�ֵ���״̬
#define M_L_AXSIS_POSITION       0xF1	//L������
#define M_O_AXSIS_POSITION       0xF2	//O������
#define M_A_AXSIS_POSITION       0xF3	//V������
#define M_B_AXSIS_POSITION       0xF4	//W������
#define M_KEY_RUN					       0xF5	//���ư尴������
#define M_KEY_PAUSE					     0xF6	//���ư尴����ͣ
#define M_KEY_RESET					     0xF7	//���ư尴����λ


//��ȡIIC�ظ�
#define   M_READ_IIC_1	         0xF1	//
#define   M_READ_IIC_2	         0xF2	//
#define   M_READ_IIC_3	         0xF3	//
#define   M_READ_IIC_4	         0xF4	//
#define   M_READ_IIC_5	         0xF5	//
#define   M_READ_IIC_6	         0xF6	//
#define   M_READ_IIC_7	         0xF7	//



extern u8 LCD_RealTime_Count;
extern u8 LCD_RealTime_Watch;
extern u8 LCD_Position_Watch;
extern u8 Manual_Pulse_Count;
extern u8 Manual_Pulse_Watch;
extern u8 Read_IIC_Done;
extern u8 Read_IIC_Count ;
extern u8 Read_IIC_Flag ;
							    
void LCDRealTimeWatch(void);	    //ʵʱ���Ӳ���
void USART1RecieveDataDecode(void);	//����1ָ����뺯��
void USART1Acknowledge(void);		//����1����Ӧ������
u8   DataSyc(void);
void MainBoard_InitDone(void);
void Program_Receive(void);//������պ���

#endif

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/

