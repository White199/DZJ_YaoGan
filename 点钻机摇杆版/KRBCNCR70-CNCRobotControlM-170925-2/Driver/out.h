/******************** (C) COPYRIGHT 2011 zjutǶ��ʽ������  ********************
* File Name          : out.h
* Author             :  
* Version            :  
* Date               :  
* Description        : This file 
*******************************************************************************/
#ifndef __out_h_
#define __out_h_

#define  OUT_REVERSING_UP                   0      //������
#define  OUT_REVERSING_DOWN                 1      //����
#define  OUT_REVERSING_CLAMP                2      //����н�
#define  OUT_REVERSING_RELEASE              3      //�������
#define  OUT_REVERSING_TURN_FRONT           4      //����ǰ��
#define  OUT_REVERSING_TURN_BACK            5      //�����

#define  OUT_TRASPOSITION_1                 6      //תλ1				  JHR105
#define  OUT_TRASPOSITION_2                 7      //תλ2				  JHR105
#define  OUT_CLAW1_CLAMP                    8      //��ץ1��			  JHR105
#define  OUT_CLAW1_RELEASE                  9      //��ץ1��			  JHR105
#define  OUT_CLAW2_CLAMP                    10     //��ץ2��			  JHR105
#define  OUT_CLAW2_RELEASE                  11     //��ץ2��			  JHR105
#define  OUT_BREAKOUT_OUTLIGHT		        12	   //����������㱨����	  JHR105
#define  OUT_MACHINE1_HOLDER_CLAMP          13     //1�Ż��о߽�		  JHR105
#define  OUT_MACHINE1_HOLDER_RELEASE        14     //1�Ż��о���		  JHR105
#define  OUT_MACHINE1_DOOR_OPEN             15     //1�Ż����ſ�		  JHR105
#define  OUT_MACHINE1_DOOR_CLOSE            16     //1�Ż����Ź�		  JHR105
#define  OUT_MACHINE1_SEND_GAS              17     //1�Ż�����			  JHR105
#define  OUT_MACHINE1_SEND_MATERIAL_FINISH  18     //1�Ż��������		  JHR105

#define  OUT_MACHINE2_HOLDER_CLAMP          19     //2�Ż��о߽�
#define  OUT_MACHINE2_HOLDER_RELEASE        20     //2�Ż��о���
#define  OUT_MACHINE2_DOOR_OPEN             21     //2�Ż����ſ�
#define  OUT_MACHINE2_DOOR_CLOSE            22     //2�Ż����Ź�
#define  OUT_MACHINE2_SEND_GAS              23     //2�Ż�����
#define  OUT_MACHINE2_SEND_MATERIAL_FINISH  24     //2�Ż��������

/*------------�ϲ�����ź�----------------*/
#define  OUT_LOAD_MATERIAL_FINISH           26     //ȡ�Ͻ����ź�
#define  OUT_UNLOAD_MATERIAL_FINISH         27     //ж�Ͻ����ź�


/**-------0x1A-----IO����-���1--------------**/
#define   O_IODEBUG_OUTPUT_0       0	//
#define   O_IODEBUG_OUTPUT_1       1	//
#define   O_IODEBUG_OUTPUT_2       2	//
#define   O_IODEBUG_OUTPUT_3       3	//
#define   O_IODEBUG_OUTPUT_4       4	//
#define   O_IODEBUG_OUTPUT_5       5	//
#define   O_IODEBUG_OUTPUT_6       6	//
#define   O_IODEBUG_OUTPUT_7       7	//
#define   O_IODEBUG_OUTPUT_8       8	//
#define   O_IODEBUG_OUTPUT_9       9	//
#define   O_IODEBUG_OUTPUT_10      10	//
#define   O_IODEBUG_OUTPUT_11      11	//
#define   O_IODEBUG_OUTPUT_12      12	//
#define   O_IODEBUG_OUTPUT_13      13	//
#define   O_IODEBUG_OUTPUT_14      14	//
#define   O_IODEBUG_OUTPUT_15      15	//
#define   O_IODEBUG_OUTPUT_16      16	//
#define   O_IODEBUG_OUTPUT_17      17	//
#define   O_IODEBUG_OUTPUT_18      18	//
#define   O_IODEBUG_OUTPUT_19      19	//
#define   O_IODEBUG_OUTPUT_20      20	//
#define   O_IODEBUG_OUTPUT_21      21	//
#define   O_IODEBUG_OUTPUT_22      22	//
#define   O_IODEBUG_OUTPUT_23      23	//
#define   O_IODEBUG_OUTPUT_24      24	//
#define   O_IODEBUG_OUTPUT_25      25	//
#define   O_IODEBUG_OUTPUT_26      26	//
#define   O_IODEBUG_OUTPUT_27      27	//
#define   O_IODEBUG_OUTPUT_28      28	//
#define   O_IODEBUG_OUTPUT_29      29	//
#define   O_IODEBUG_OUTPUT_30      30	//
#define   O_IODEBUG_OUTPUT_31      31	//
#define   O_IODEBUG_OUTPUT_32      32	//
#define   O_IODEBUG_OUTPUT_33      33	//
#define   O_IODEBUG_OUTPUT_34      34	//
#define   O_IODEBUG_OUTPUT_35      35	//
#define   O_IODEBUG_OUTPUT_36      36	//
#define   O_IODEBUG_OUTPUT_37      37	//
#define   O_IODEBUG_OUTPUT_38      38	//
#define   O_IODEBUG_OUTPUT_39      39	//
/**-------0x1A-----IO����-���1--------------**/
#define   O_STORAGE_LOAD_FINISH    0	//�ϲ�-�������
#define   O_STORAGE_UNLOAD_FINISH  1	//�ϲ�-ж�����
#define   O_STORAGE_LOAD_ALARM     2	//�ϲ�-���ϲֱ���
#define   O_STORAGE_UNLOAD_ALARM   3	//�ϲ�-ж�ϲֱ���

#define   O_MACHINE1_KAPA_OPEN     4	//����1����A��
#define   O_MACHINE1_KAPA_CLOSE    5	//����1����A��
#define   O_MACHINE1_CUIQI         6	//����1����
#define   O_MACHINE1_ALLOW_START   7	//����1��������
#define   O_MACHINE1_START_PROCESS 8	//����1��ʼ�ӹ�
#define   O_MACHINE1_DOOR_OPEN     9	//����1���ſ�
#define   O_MACHINE1_DOOR_CLOSE    10	//����1���Ź�
#define   O_MACHINE1_OBLIGATE1     11	//����1Ԥ��1

/**-------0x1B-----IO����-���2--------------**/
#define   O_MACHINE2_KAPA_OPEN     12	//����2����A��
#define   O_MACHINE2_KAPA_CLOSE    13	//����2����A��
#define   O_MACHINE2_CUIQI         14	//����2����
#define   O_MACHINE2_ALLOW_START   15	//����2��������
#define   O_MACHINE2_START_PROCESS 16	//����2��ʼ�ӹ�
#define   O_MACHINE2_DOOR_OPEN     17	//����2���ſ�
#define   O_MACHINE2_DOOR_CLOSE    18	//����2���Ź�
#define   O_MACHINE2_OBLIGATE1     19	//����2Ԥ��1

#define   O_ROLL_OVER_UPRIGHT      20	//��ת̨-����
#define   O_ROLL_OVER_REVERSE      21	//��ת̨-����
#define   O_ROLL_OVER_OPEN         22   //��ת̨-�ɿ�
#define   O_ROLL_OVER_CLOSE        23	//��ת̨-�н�


/**-------0x1C-----IO����-���3--------------**/
#define   O_CLAW_LOAD_OPEN         24	//��צ-����צ��
#define   O_CLAW_LOAD_CLOSE        25	//��צ-����צ��
#define   O_CLAW_UNLOAD_OPEN       26	//��צ-ж��צ��
#define   O_CLAW_UNLOAD_CLOSE      27	//��צ-ж��צ��
#define   O_CLAW_BAIGANG_UPRIGHT   28	//��צ-�ڸ�����
#define   O_CLAW_BAIGANG_RESET     29	//��צ-�ڸ׸�λ
#define   O_CLAW_YCYLINDER_STRETCH 30	//��צ-Y���������
#define   O_CLAW_YCYLINDER_RETRACT 31	//��צ-Y����������
#define   O_CLAW_BLOW              32	//��צ-��ץ����

#define   O_OTHERS_OILING_ENABLE   33	//����-��ʹ��
#define   O_OTHERS_RUN_LIGHT       34	//����-���е�
#define   O_OTHERS_WAIT_LIGHT      35	//����-������
#define   O_OTHERS_ALARM_LIGHT     36	//����-������



/*------------�����������ź�----------------*/
#define X_AXSIS_DIRECTION                    40	   //
#define Z_AXSIS_DIRECTION					 41	   //
#define L_AXSIS_DIRECTION                    42	   //
#define O_AXSIS_DIRECTION					 43	   //


extern void SetOutput(u8 IO_Num);
extern void ResetOutput(u8 IO_Num);
extern u8 Output_Status[5];



#endif

/******************* (C) COPYRIGHT 2011 zjutǶ��ʽ������ *****END OF FILE****/
