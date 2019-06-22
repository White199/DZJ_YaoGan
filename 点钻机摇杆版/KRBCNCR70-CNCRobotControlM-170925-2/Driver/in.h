//******************************************************//
//*****************  �����źż��  *********************//
//**   �����ź�ȫ��JHR210�����룬�����עΪJHR105     **//
//**   �������ź�ΪС�ͺŻ���ʹ�õ�����               **//
//******************************************************//
#ifndef __in_h_
#define __in_h_

#define  DETECT_REVERSING_UP              0       //���������
#define  DETECT_REVERSING_DOWN            1       //���򽵼��
#define  DETECT_REVERSING_CLAMP           2       //����н����
#define  DETECT_REVERSING_RELEASE         3       //������ɼ��
#define  DETECT_REVERSING_TURN_FRONT      4       //����ǰ�����
#define  DETECT_REVERSING_TURN_BACK       5       //����󷭼��

#define  DETECT_TRASPOSITION1             6       //��תλ1���				   JHR105
#define  DETECT_TRASPOSITION2             7       //��תλ2���				   JHR105
#define  DETECT_CLAW1_CLAMP               8       //��ץ1�����				   JHR105
#define  DETECT_CLAW1_RELEASE             9       //��ץ1�ɼ��				   JHR105
#define  DETECT_CLAW2_CLAMP               10      //��ץ2�����				   JHR105
#define  DETECT_CLAW2_RELEASE             11      //��ץ2�ɼ��				   JHR105
#define  DETECT_X_ORIGIN				  12	  //����X�����				   JHR105
#define  DETECT_Z_ORIGIN				  13	  //����Z�����				   JHR105
#define  DETECT_MACHINE_BREAKDOWN		  14	  //��������				   JHR105
#define  DETECT_ROBOT_START				  15	  //����������				   JHR105
#define  DETECT_ROBOT_STOP				  16	  //������ֹͣ				   JHR105
#define  DETECT_MACHINE1_DOOR_BUTTON	  17	  //1�Ż����Ű�ť			   JHR105
#define	 DETECT_MACHINE1_HOLDER_BUTTON	  18	  //1�Ż��о߰�ť			   JHR105
#define	 DETECT_MACHINE1_NEED_MATERIAL	  19	  //1�Ż�Ҫ�ϰ�ť			   JHR105
#define  DETECT_MACHINE1_HOLDER_CLAMP     20      //1�Ż��о߽����			   JHR105
#define  DETECT_MACHINE1_HOLDER_RELEASE   21      //1�Ż��о��ɼ��			   JHR105
#define  DETECT_MACHINE1_DOOR_OPEN        22      //1�Ż����ſ����			   JHR105
#define  DETECT_MACHINE1_DOOR_CLOSE       23      //1�Ż����Źؼ��			   JHR105

#define  DETECT_MACHINE2_DOOR_BUTTON	  24	  //1�Ż����Ű�ť
#define	 DETECT_MACHINE2_HOLDER_BUTTON	  25	  //1�Ż��о߰�ť
#define	 DETECT_MACHINE2_NEED_MATERIAL	  26	  //1�Ż�Ҫ�ϰ�ť
#define  DETECT_MACHINE2_HOLDER_CLAMP     27      //2�Ż��о߽����
#define  DETECT_MACHINE2_HOLDER_RELEASE   28      //2�Ż��о��ɼ��
#define  DETECT_MACHINE2_DOOR_OPEN        29      //2�Ż����ſ����
#define  DETECT_MACHINE2_DOOR_CLOSE       30      //2�Ż����Źؼ��

/*------------�ϲ������ź�----------------*/
#define	 ALLOW_LOAD_MATERIAL			  31	  //����ȡ��				  JHR105
#define	 ALLOW_UNLOAD_MARERIAL			  32	  //����ж��				  JHR105
#define	 LACK_MATERIAL_ALARM			  33	  //ȱ�ϱ���				  JHR105
#define	 FULL_MATERIAL_ALARM			  34	  //���ϱ���				  JHR105
#define  EMERGENCY_STOP_ALARM	          35	  //��ͣ����				  JHR105

/*------------�ŷ������ޱ��������ź�----------------*/
#define	 X_LEFT_LIMIT			          36	  //X����				  JHR105
#define	 X_RIGHT_LIMIT			          37	  //X�Ҽ���				  JHR105
#define	 Z_LEFT_LIMIT			          38	  //Z����				  JHR105
#define	 Z_RIGHT_LIMIT			          39	  //Z�Ҽ���				  JHR105
		
#define  I_DETECT_O_ORIGIN				  40	  //����O�����				   JR
#define  I_DETECT_L_ORIGIN				  41	  //����L�����				   JR
#define  I_DETECT_X_ORIGIN				  42	  //����X�����				   JR
#define  I_DETECT_Z_ORIGIN				  43	  //����Z�����				   JR

extern u8 ReadInput(u8 IO_Num);

extern u8 ReadEmergencyStop(void);
extern u8 ReadXAxsisError(void) ;
extern u8 ReadZAxsisError(void);
extern u8 ReadAxsisError(u8) ;
extern u8 ReadIOPort(GPIO_TypeDef* GPIOx, u16 PortNum, u8 PortPreviousStatus);

extern u8 Local_IO_Status[2];
extern u8 Input_Detect_Status[6];

#endif
/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/
