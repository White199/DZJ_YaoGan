/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __signalwatch_h_
#define __signalwatch_h_

/******************USART��������Command***************************/
#define WORK_MODE         0x0a	   //����ģʽ
#define AUTO_MODE         0x0b	   //�Զ�ģʽ
#define MANUAL_OPERATE	  0x0c	   //�ֶ�������������
#define PARAMETER_COMMAND 0x0d	   //��������
#define WATCH_COMMAND     0x0e	   //��������
#define READ_IIC		  0x0f	   //������ȡIIC����
#define REAL_TIME_WATCH   0x10	   //ʵʱ�������
#define BACK_TO_ORIGIN    0x11	   //��ԭ������
#define CANCLE_ALARM      0x12	   //ȡ������
#define INITIALIZE_FINISH 0xbd     //��ʼ�����
#define ORIGIN_BACK		  0xbe     //��ԭ�����

/**-------0x0E-----��������--------------**/
#define   IO_DEBUG_LOCAL         0x01	//IO����-����IO
#define   IO_DEBUG_INPUT1        0x02	//IO����-����1
#define   IO_DEBUG_INPUT2        0x03	//IO����-����2
#define   IO_DEBUG_INPUT3        0x04	//IO����-����3
#define   IO_DEBUG_OUTPUT1_LCD   0x05	//IO����-���1����
#define   IO_DEBUG_OUTPUT2_LCD   0x06	//IO����-���2����
#define   IO_DEBUG_OUTPUT3_LCD   0x07	//IO����-���3����
#define   ROBOT_ORIGINED		 0x08	//��е���Ƿ��ѻ�ԭ��
#define   ACTION_RESET_SCAN	     0x09	//��λɨ��
#define   ALARM_CLEAR    	     0x0A	//��������
#define   ROBOT_STATUS      	 0x0B	//��е��״̬
#define   AUTO_PARAMETER         0x0C	//�Զ����в���-��ǰ��-����
#define   X_AXSIS_POSITION	     0x0D	//X������
#define   Z_AXSIS_POSITION       0x0E	//Z������
#define   ROBOT_PRE_STATUS       0x0F	//��е������״̬
#define   ROBOT_DEBUG_STATUS     0x10	//��е�ֵ���״̬
#define   L_AXSIS_POSITION       0x11	//L������
#define   O_AXSIS_POSITION       0x12	//L������
#define   A_AXSIS_POSITION       0x13	//V������
#define   B_AXSIS_POSITION       0x14	//W������

/******************��������***************************/
#define IN_PAGE1		    0x01   //����ҳһ
#define IN_PAGE2			0x02   //����ҳ��
#define	IN_PAGE3			0x03   //����ҳ��
#define	OUT_PAGE1			0x04   //���ҳһ
#define	OUT_PAGE2			0x05   //���ҳ��
#define	MANUAL_PARAMETER	0x06   //�ֶ�����
#define	PROCESS				0x07   //�ӹ�
#define	TIME_POINTS_OTHERS	0x08   //ʱ��-��-����


/**------------P_XX�ֳ���������ͨ�Ų���--------------**/
/**------------P_XX�ֳ���������ͨ�Ų���--------------**/
/**------------��������--------------**/
#define   P_ROBOT_ENABLE_A_ORIGIN  0x0A	//��е��ʹ�ܺͻ�ԭ��
#define   P_WORK_MODE              0x0B	//��е�ֹ���ģʽ
#define   P_AUTO_RUN               0x0C	//��е���Զ�����
#define   P_FREE_PROGRAM_SEND      0x0D	//���ɱ�̳����·�
#define   P_WATCH_COMMAND          0x0E	//��������
#define   P_READ_IIC	             0x0F	//������IIC
#define   P_IO_DEBUG_OUTPUT1       0x1A	//IO����-���1
#define   P_IO_DEBUG_OUTPUT2       0x1B	//IO����-���2
#define   P_IO_DEBUG_OUTPUT3       0x1C	//IO����-���3
#define   P_MANUL_DEBUG            0x1D	//�ֶ�����
#define   P_PARAMETER_ORDER        0x1E	//��������
#define   P_AUTO_RUN_LCD           0x1F	//��е���Զ�����

/**-------0x0A-----ʹ�ܺͻ�ԭ��--------------**/
#define   P_ROBOT_ORIGIN   	     0x01	//��е�ֻ�ԭ��
#define   P_X_AXSIS_ORIGIN	     0x02	//X���ԭ��
#define   P_Z_AXSIS_ORIGIN	     0x03	//Z���ԭ��
#define   P_ORIGIN_SPEED	     0x04	//��е�ֻ�ԭ���ٶ�
#define   P_ROBOT_ENABLE	     0x05	//��е���Ƿ�ʹ��

/**-------0x0B-----����ģʽ--------------**/
#define   P_AUTO_MODE              0x01	//�Զ�ģʽ
#define   P_FREE_PROGRAM           0x02	//���ɱ��
#define   P_IO_MODE                0x03	//IO����
#define   P_MANUL_MODE             0x04	//�ֶ�����

/**-------0x0C-----�Զ�����--------------**/
#define   P_CYCLE_MODE             0x01	//ѭ��ģʽ
#define   P_SINGLE_MODE            0x02	//����ģʽ
#define   P_ACTION_RUN             0x03	//����
#define   P_ACTION_PAUSE           0x04	//��ͣ
#define   P_ACTION_STOP            0x05	//ֹͣ
#define   P_ACTION_PROGRAM         0x06	//ѡ�����еĳ���
#define   P_ACTION_RESET           0x07	//��е�ָ�λ
#define   P_ACTION_DEBUG           0x08	//��е�ֵ���-���ɱ�̵ĵ��԰�ť
#define   P_ACTION_LORIGIN         0x09	//��е��L�����

/**-------0x0D-----���ɱ��--------------**/
#define   P_PROGRAM_START          0xE1	//����ʼ
#define   PROGRAM_INFO             0xE2	//������Ϣ����
#define   PROGRAM_CONT             0xE3	//��������
#define   PROGRAM_DELETE           0xE4	//����ɾ��
#define   PROGRAM_FROM_USB_START   0xE5	//USB���򿽱����Ϳ�ʼ
#define   PROGRAM_FROM_USB_END     0xE6	//USB���򿽱����ͽ���
//#define   P_PROGRAM_END            0xED	//�������

#define   P_PROGRAM_RENAME			0xE7 //������
#define   P_PROGRAM_CHOOSE         0xE8	//��Ʒѡ��
#define   P_PROGRAM_DELETE         0xE9	//��Ʒɾ��
#define   P_PROGRAM_RENAME_COVER   0xEA //��Ʒ��������
#define   P_PROGRAM_NEW				0xEB	//��Ʒ�½�
#define   P_PROGRAM_NEWNAME_COVER	0xEC //�½�ͬ������-ȷ��
#define   P_PROGRAM_SEND			0xED //�������ذ���������
/**-------0x0E-----��������--------------**/
#define   P_IO_DEBUG_LOCAL         0x01	//IO����-����IO
#define   P_IO_DEBUG_INPUT1        0x02	//IO����-����1
#define   P_IO_DEBUG_INPUT2        0x03	//IO����-����2
#define   P_IO_DEBUG_INPUT3        0x04	//IO����-����3
#define   P_IO_DEBUG_OUTPUT1_LCD   0x05	//IO����-���1����
#define   P_IO_DEBUG_OUTPUT2_LCD   0x06	//IO����-���2����
#define   P_IO_DEBUG_OUTPUT3_LCD   0x07	//IO����-���3����
#define   P_ROBOT_ORIGINED		   0x08	//��е���Ƿ��ѻ�ԭ��
#define   P_X_AXSIS_ORIGINED	   0x09	//X���Ƿ��ѻ�ԭ��
#define   P_Z_AXSIS_ORIGINED	   0x0A	//Z���Ƿ��ѻ�ԭ��
#define   P_ALARM_STATUS      	   0x0B	//������־λ
#define   P_PRE_PROGRAM_NUMBER     0x0C	//Z���Ƿ��ѻ�ԭ��
#define   P_X_AXSIS_POSITION	   0x0D	//X������
#define   P_Z_AXSIS_POSITION       0x0E	//Z������

/**-------0x0F-----������IIC--------------**/
#define   P_READ_IIC_1	         0x01	//
#define   P_READ_IIC_2	         0x02	//
#define   P_READ_IIC_3	         0x03	//
#define   P_READ_IIC_4	         0x04	//
#define   P_READ_IIC_5	         0x05	//
#define   P_READ_IIC_6	         0x06	//


/**-------0x1A-----IO����-���1--- 0x01 Y0--0x0F Y14---------**/
#define   P_IODEBUG_OUTPUT1_1      0x01	//
#define   P_IODEBUG_OUTPUT1_2      0x02	//
#define   P_IODEBUG_OUTPUT1_3      0x03	//
#define   P_IODEBUG_OUTPUT1_4      0x04	//
#define   P_IODEBUG_OUTPUT1_5      0x05	//
#define   P_IODEBUG_OUTPUT1_6      0x06	//
#define   P_IODEBUG_OUTPUT1_7      0x07	//
#define   P_IODEBUG_OUTPUT1_8      0x08	//
#define   P_IODEBUG_OUTPUT1_9      0x09	//
#define   P_IODEBUG_OUTPUT1_10     0x0A	//
#define   P_IODEBUG_OUTPUT1_11     0x0B	//
#define   P_IODEBUG_OUTPUT1_12     0x0C	//
#define   P_IODEBUG_OUTPUT1_13     0x0D	//	 
#define   P_IODEBUG_OUTPUT1_14     0x0E	//
#define   P_IODEBUG_OUTPUT1_15     0x0F	//

/**-------0x1B-----IO����-���2--- 0X01 Y15---0X0F Y29--------**/
#define   P_IODEBUG_OUTPUT2_1      0x01	//
#define   P_IODEBUG_OUTPUT2_2      0x02	//
#define   P_IODEBUG_OUTPUT2_3      0x03	//
#define   P_IODEBUG_OUTPUT2_4      0x04	//
#define   P_IODEBUG_OUTPUT2_5      0x05	//
#define   P_IODEBUG_OUTPUT2_6      0x06	//
#define   P_IODEBUG_OUTPUT2_7      0x07	//
#define   P_IODEBUG_OUTPUT2_8      0x08	//
#define   P_IODEBUG_OUTPUT2_9      0x09	//
#define   P_IODEBUG_OUTPUT2_10     0x0A	//
#define   P_IODEBUG_OUTPUT2_11     0x0B	//
#define   P_IODEBUG_OUTPUT2_12     0x0C	//
#define   P_IODEBUG_OUTPUT2_13     0x0D	//	 
#define   P_IODEBUG_OUTPUT2_14     0x0E	//
#define   P_IODEBUG_OUTPUT2_15     0x0F	//

/**-------0x1C-----IO����-���2--- 0X01 Y30---0X0A Y39--------**/
#define   P_IODEBUG_OUTPUT3_1      0x01	//
#define   P_IODEBUG_OUTPUT3_2      0x02	//
#define   P_IODEBUG_OUTPUT3_3      0x03	//
#define   P_IODEBUG_OUTPUT3_4      0x04	//
#define   P_IODEBUG_OUTPUT3_5      0x05	//
#define   P_IODEBUG_OUTPUT3_6      0x06	//
#define   P_IODEBUG_OUTPUT3_7      0x07	//
#define   P_IODEBUG_OUTPUT3_8      0x08	//
#define   P_IODEBUG_OUTPUT3_9      0x09	//
#define   P_IODEBUG_OUTPUT3_10     0x0A	//

/**-------0x1B-----IO����-���2--------------**/
#define   P_MACHINE2_KAPA_OPEN     0x01	//����1����A��
#define   P_MACHINE2_KAPA_CLOSE    0x02	//����1����A��
#define   P_MACHINE2_CUIQI         0x03	//����1����
#define   P_MACHINE2_ALLOW_START   0x04	//����1��������
#define   P_MACHINE2_START_PROCESS 0x05	//����1��ʼ�ӹ�
#define   P_MACHINE2_DOOR_OPEN     0x06	//����1���ſ�
#define   P_MACHINE2_DOOR_CLOSE    0x07	//����1���Ź�
#define   P_MACHINE2_OBLIGATE1     0x08	//����1Ԥ��1
#define   P_MACHINE2_OBLIGATE2     0x09	//����1Ԥ��2
#define   P_MACHINE2_OBLIGATE3     0x0A	//����1Ԥ��3

#define   P_ROLL_OVER_UPRIGHT      0x0B	//��ת̨-����
#define   P_ROLL_OVER_REVERSE      0x0C	//��ת̨-����
#define   P_ROLL_OVER_OPEN         0x0D	//��ת̨-�ɿ�
#define   P_ROLL_OVER_CLOSE        0x0E	//��ת̨-�н�
#define   P_ROLL_OVER_OBLIGATE1    0x10	//��ת̨-Ԥ��1

/**-------0x1C-----IO����-���3--------------**/
#define   P_CLAW_LOAD_OPEN         0x01	//��צ-����צ��
#define   P_CLAW_LOAD_CLOSE        0x02	//��צ-����צ��
#define   P_CLAW_UNLOAD_OPEN       0x03	//��צ-ж��צ��
#define   P_CLAW_UNLOAD_CLOSE      0x04	//��צ-ж��צ��
#define   P_CLAW_BAIGANG_UPRIGHT   0x05	//��צ-�ڸ�����
#define   P_CLAW_BAIGANG_RESET     0x06	//��צ-�ڸ׸�λ
#define   P_CLAW_YCYLINDER_STRETCH 0x07	//��צ-Y���������
#define   P_CLAW_YCYLINDER_RETRACT 0x08	//��צ-Y����������
#define   P_CLAW_BLOW              0x09	//��צ-��ץ����
#define   P_CLAW_OBLIGATE1         0x0A	//��צ-Ԥ��1

#define   P_OTHERS_OILING_ENABLE   0x0B	//����-��ʹ��
#define   P_OTHERS_RUN_LIGHT       0x0C	//����-���е�
#define   P_OTHERS_WAIT_LIGHT      0x0D	//����-������
#define   P_OTHERS_ALARM_LIGHT     0x0E	//����-������
#define   P_OTHERS_OBLIGATE1       0x10	//����-Ԥ��1	
	  
/**-------0x1D-----�ֶ�����-----------------**/
#define	  P_XAXIS_MANUL_SPEED	   0x01	//X��-�ֶ��ٶ�ֵ
#define   P_XAXIS_MOVE_LEFT        0x02	//X��-����
#define   P_XAXIS_MOVE_RIGHT       0x03	//X��-����
#define   P_XAXIS_STEP_STOP        0x04	//X��-ֹͣ
#define   P_XAXIS_SAVE_POINT       0x05	//X��-����
#define	  P_ZAXIS_MANUL_SPEED	   0x06	//Z��-�ֶ��ٶ�ֵ
#define   P_ZAXIS_MOVE_LEFT        0x07	//Z��-����
#define   P_ZAXIS_MOVE_RIGHT       0x08	//Z��-����
#define   P_ZAXIS_STEO_STOP        0x09	//Z��-ֹͣ
#define   P_ZAXIS_SAVE_POINT       0x0A	//Z��-����
#define   P_XZAXIS_SAVE_POINT      0x0B	//XZ��-����
#define   P_XZAXIS_DELETE_POINT   0x0C	//XZ��-ɾ��
#define	  P_XAXIS_STEP_MM	       0x0D	//X��-�綯����
#define	  P_ZAXIS_STEP_MM	       0x0E	//Z��-�綯����
#define	  P_XAXIS_BACK_ORIGIN	   0x0F	//X��-����
#define	  P_ZAXIS_BACK_ORIGIN	   0x10	//Z��-����
#define   P_MANUL_FINE_ADJUST	   0x11	//΢������

#define	  P_LAXIS_MANUL_SPEED	   0x12	//L��-�ֶ��ٶ�ֵ
#define   P_LAXIS_MOVE_LEFT        0x13	//L��-����
#define   P_LAXIS_MOVE_RIGHT       0x14	//L��-����
#define   P_LAXIS_STEP_STOP        0x15	//L��-�綯ֹͣ
#define	  P_LAXIS_STEP_MM		   0x16	//L��-�綯����mmΪ��λ
#define   P_LAXIS_SAVE_POINT       0x17	//L��-����
#define	  P_LAXIS_BACK_ORIGIN	   0x18	//L��-����

#define	  P_OAXIS_MANUL_SPEED	   0x19	//O��-�ֶ��ٶ�ֵ
#define   P_OAXIS_MOVE_LEFT        0x1A	//O��-����
#define   P_OAXIS_MOVE_RIGHT       0x1B	//O��-����
#define   P_OAXIS_STEP_STOP        0x1C	//O��-�綯ֹͣ
#define	  P_OAXIS_STEP_MM		   0x1D	//O��-�綯����mmΪ��λ
#define   P_OAXIS_SAVE_POINT       0x1E	//O��-����
#define	  P_OAXIS_BACK_ORIGIN	   0x1F	//O��-����
#define   P_LOAXIS_DELETE_POINT   0x20	//LO��-ɾ��


#define	  P_VAXIS_MANUL_SPEED	   0x21	//V��-�ֶ��ٶ�ֵ
#define   P_VAXIS_MOVE_LEFT        0x22	//V��-����
#define   P_VAXIS_MOVE_RIGHT       0x23	//V��-����
#define   P_VAXIS_STEP_STOP        0x24	//V��-�綯ֹͣ
#define	  P_VAXIS_STEP_MM		   0x25	//V��-�綯����mmΪ��λ
#define   P_VAXIS_SAVE_POINT       0x26	//V��-����
#define	  P_VAXIS_BACK_ORIGIN	   0x27	//V��-����

#define	  P_WAXIS_MANUL_SPEED	   0x28	//W��-�ֶ��ٶ�ֵ
#define   P_WAXIS_MOVE_LEFT        0x29	//W��-����
#define   P_WAXIS_MOVE_RIGHT       0x2A	//W��-����
#define   P_WAXIS_STEP_STOP        0x2B	//W��-�綯ֹͣ
#define	  P_WAXIS_STEP_MM		   0x2C	//W��-�綯����mmΪ��λ
#define   P_WAXIS_SAVE_POINT       0x2D	//W��-����
#define	  P_WAXIS_BACK_ORIGIN	   0x2E	//W��-����

#define   E_GEARRATIO_SET			0x2F//���ӳ��ֱ�����
#define   XYAXIS_MANUL_SPEED    	0x30//ʾ�̽���XY�����ٶ�����
#define   ZOAXIS_MANUL_SPEED    	0x31//ʾ�̽���ZO�����ٶ�����
#define   ABAXIS_MANUL_SPEED    	0x32//ʾ�̽���AB�����ٶ�����
#define   XYAXIS_STEP_MM			0x33//ʾ�̽���XY�綯�ٶ�����
#define   ZOAXIS_STEP_MM			0x34//ʾ�̽���ZO�綯�ٶ�����
#define   ABAXIS_STEP_MM			0x35//ʾ�̽���AB�綯�ٶ�����
#define   P_ACTION_SPEEDADJ     0x36//���ٶȵ���20181120-lin
/**-------0x1E-----��������-----------------**/
#define	  P_PARAMETER_SOFT_LIMIT	   0x01	//����λ
#define   P_PARAMETER_SAFE_AREA        0x02	//��ȫ��
#define   P_PARAMETER_FUCTION_SET      0x03	//���ܲ���


extern u8 Watch_Data;				   //ʵʱ�����������

extern void WatchFunction(void);
extern void OrderDecoding(void);


#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

