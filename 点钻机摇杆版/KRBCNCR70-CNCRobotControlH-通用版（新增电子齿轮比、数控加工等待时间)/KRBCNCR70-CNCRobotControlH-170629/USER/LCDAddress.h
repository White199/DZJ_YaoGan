#ifndef __LCD_ADDRESS_H
#define __LCD_ADDRESS_H


#define ADDR_PAGE_MAIN					0
#define ADDR_PAGE_BACK					1
#define ADDR_PAGE_MANUAL				2
#define ADDR_PAGE_MANUAL_PARAMETER		3
#define ADDR_PAGE_AUTO					4
#define ADDR_PAGE_FUNCTION				5
#define ADDR_PAGE_CIPHER				6
#define ADDR_PAGE_FUNCTION_2011_1		7
#define ADDR_PAGE_FUNCTION_2011_2		8
#define ADDR_PAGE_FUNCTION_2011_3		9
#define ADDR_PAGE_FUNCTION_2011_4		25
#define ADDR_PAGE_FUNCTION_2012			10
#define ADDR_PAGE_FUNCTION_2012_2		27
#define ADDR_PAGE_FUNCTION_2013			11
#define ADDR_PAGE_FUNCTION_2014			12
#define ADDR_PAGE_FUNCTION_LIGHT		13
#define ADDR_PAGE_TEACH_MODULE_SELECT	14
#define ADDR_PAGE_TEACH_MODULE_CHOOSE	15
#define ADDR_PAGE_TEACH					16
#define ADDR_PAGE_ERROR_RECORD			17
#define ADDR_PAGE_RUN_RECORD			18
#define ADDR_PAGE_WATCH_INPUT_1			19
#define ADDR_PAGE_WATCH_INPUT_2			20
#define ADDR_PAGE_WATCH_INPUT_3			21
#define ADDR_PAGE_WATCH_OUTPUT_1		22
#define ADDR_PAGE_WATCH_OUTPUT_2		23
#define ADDR_PAGE_WATCH_OUTPUT_3		26
#define ADDR_PAGE_MANUAL_HELP			24
#define ADDR_PAGE_DECODE				28
#define ADDR_PAGE_BUTTOM				100



#define ADDR_PICTURE_STATUS		0x0000	//��ǰģʽ
#define ADDR_PICTURE_MOLD_OPEN	0x0010	//��ģ��
#define ADDR_PICTURE_SAFE_GATE	0x0020	//��ȫ��
#define ADDR_PICTURE_EN_MOLD	0x0030	//�ɹ�ģ
#define ADDR_PICTURE_THINBLE	0x0040	//�ɶ���
#define ADDR_PICTURE_ERROR		0x0050	//������ʶ
#define ADDR_PICTURE_LIGHT		0x0060	//�����Ͽ�
#define ADDR_PICTURE_ARROW		0x0070	//�̵���ǰ�м�ͷ
#define ADDR_PICTURE_BUTTON		0x0080	//��ɫ
#define ADDR_PICTURE_W_X10		0x0600	//
#define ADDR_PICTURE_W_X11		0x0610	//
#define ADDR_PICTURE_W_X12		0x0620	//
#define ADDR_PICTURE_W_X13		0x0630	//
#define ADDR_PICTURE_W_X14		0x0640	//
#define ADDR_PICTURE_W_X15		0x0650	//
#define ADDR_PICTURE_W_X16		0x0660	//
#define ADDR_PICTURE_W_X17		0x0670	//
#define ADDR_PICTURE_W_X20		0x0680	//
#define ADDR_PICTURE_W_X21		0x0690	//
#define ADDR_PICTURE_W_X22		0x06A0	//
#define ADDR_PICTURE_W_X23		0x06B0	//
#define ADDR_PICTURE_W_X24		0x06C0	//
#define ADDR_PICTURE_W_X25		0x06D0	//
#define ADDR_PICTURE_W_X26		0x06E0	//
#define ADDR_PICTURE_W_X27		0x06F0	//
#define ADDR_PICTURE_W_X30		0x0600	//
#define ADDR_PICTURE_W_X31		0x0610	//
#define ADDR_PICTURE_W_X32		0x0620	//
#define ADDR_PICTURE_W_X33		0x0630	//
#define ADDR_PICTURE_W_X34		0x0640	//
#define ADDR_PICTURE_W_X35		0x0650	//
#define ADDR_PICTURE_W_X36		0x0660	//
#define ADDR_PICTURE_W_X37		0x0670	//
#define ADDR_PICTURE_W_X40		0x0680	//
#define ADDR_PICTURE_W_X41		0x0690	//
#define ADDR_PICTURE_W_X42		0x06A0	//
#define ADDR_PICTURE_W_X43		0x06B0	//
#define ADDR_PICTURE_W_X44		0x06C0	//
#define ADDR_PICTURE_W_X45		0x06D0	//
#define ADDR_PICTURE_W_X46		0x06E0	//
#define ADDR_PICTURE_W_X47		0x06F0	//
#define ADDR_PICTURE_W_X50		0x0600	//
#define ADDR_PICTURE_W_X51		0x0610	//
#define ADDR_PICTURE_W_X52		0x0620	//
#define ADDR_PICTURE_W_X53		0x0630	//
#define ADDR_PICTURE_W_X54		0x0640	//
#define ADDR_PICTURE_W_X55		0x0650	//
#define ADDR_PICTURE_W_X56		0x0660	//
#define ADDR_PICTURE_W_X57		0x0670	//
#define ADDR_PICTURE_W_Y10		0x0600	//
#define ADDR_PICTURE_W_Y11		0x0610	//
#define ADDR_PICTURE_W_Y12		0x0620	//
#define ADDR_PICTURE_W_Y13		0x0630	//
#define ADDR_PICTURE_W_Y14		0x0640	//
#define ADDR_PICTURE_W_Y15		0x0650	//
#define ADDR_PICTURE_W_Y16		0x0660	//
#define ADDR_PICTURE_W_Y17		0x0670	//
#define ADDR_PICTURE_W_Y20		0x0680	//
#define ADDR_PICTURE_W_Y21		0x0690	//
#define ADDR_PICTURE_W_Y22		0x06A0	//
#define ADDR_PICTURE_W_Y23		0x06B0	//
#define ADDR_PICTURE_W_Y24		0x06C0	//
#define ADDR_PICTURE_W_Y25		0x06D0	//
#define ADDR_PICTURE_W_Y26		0x06E0	//
#define ADDR_PICTURE_W_Y27		0x06F0	//
#define ADDR_PICTURE_W_Y30		0x0600	//
#define ADDR_PICTURE_W_Y31		0x0610	//
#define ADDR_PICTURE_W_Y32		0x0620	//
#define ADDR_PICTURE_W_Y33		0x0630	//
#define ADDR_PICTURE_W_Y34		0x0640	//
#define ADDR_PICTURE_W_Y35		0x0650	//
#define ADDR_PICTURE_W_Y36		0x0660	//
#define ADDR_PICTURE_W_Y37		0x0670	//
#define ADDR_PICTURE_W_Y40		0x0680	//
#define ADDR_PICTURE_W_Y41		0x0690	//
#define ADDR_PICTURE_W_Y42		0x06A0	//
#define ADDR_PICTURE_W_Y43		0x06B0	//
#define ADDR_PICTURE_W_Y44		0x06C0	//
#define ADDR_PICTURE_W_Y45		0x06D0	//
#define ADDR_PICTURE_W_Y46		0x06E0	//
#define ADDR_PICTURE_W_Y47		0x06F0	//

#define ADDR_NUM_MODULE_NUM		0x0100	//��ǰģ���
#define ADDR_NUM_POSITION_NOW	0x0101	//��ǰ����
#define ADDR_NUM_MANUAL_SPEED	0x0103	//�ֶ��ٶ� 
#define ADDR_NUM_MANUAL_POSITION	0x0104	//�ֶ�λ��
#define ADDR_NUM_INCH_SEPPD		0x0106	//�綯�ٶ�
#define ADDR_NUM_REAL_PRODUCT	0x0107	//ʵ�ʳ�Ʒ
#define ADDR_NUM_GET_POSITION	0x0109	//ȡ��λ��
#define ADDR_NUM_SET_HARVEST	0x010B	//�趨����
#define ADDR_NUM_CIRCLE_TIME	0x010D	//�Զ�����
#define ADDR_NUM_GET_TIME		0x010F	//ȡ��ʱ��
#define ADDR_NUM_OPERATE_TIME	0x0111	//����ʱ��
#define ADDR_NUM_SET_COUNTER	0x0113	//�趨ģ��
#define ADDR_NUM_OPEN_DELAY		0x0115	//��ģ��ʱ
#define ADDR_NUM_THINBLE_DELAY	0x0116	//������ʱ
#define ADDR_NUM_CODE			0x0117	//����
#define ADDR_NUM_MACHINE_TIME	0x0118	//�ܻ�����ʱ��
#define ADDR_NUM_POINT			0x011A	//����
#define ADDR_NUM_SPACE			0x011B	//���
#define ADDR_NUM_SEND_COUNTER	0x011D	//���ͼ��
#define ADDR_NUM_ONE_COUNTER	0x011E	//Ԥһ���
#define ADDR_NUM_LIGHT_OFF_TIME	0x011F	//����ʱ��
#define ADDR_NUM_TWO_COUNTER	0x0120	//Ԥ�����
#define ADDR_NUM_GATE_POSITION	0x0121	//��ȫ��λ��
#define ADDR_NUM_O_W_POSITION	0x0123	//���������
#define ADDR_NUM_I_SAFE_AREA	0x0125	//ģ�ڰ�ȫ��
#define ADDR_NUM_REDUCE_DELAY	0x0127	//������ʱ
#define ADDR_NUM_RANK_POSITION	0x0128	//������ʼ��
#define ADDR_NUM_V_OPEN_DELAY	0x012A	//���۷���ʱ
#define ADDR_NUM_L_POSITION		0x012B	//���λ��
#define ADDR_NUM_GEAR_S			0x012D	//���ֱȷ���
#define ADDR_NUM_GEAR_M			0x012E	//���ֱȷ�ĸ
#define ADDR_NUM_RETURN_SPEED	0x012F	//�����ٶ�
#define ADDR_NUM_WARNNING_TIME	0x0130	//����ʱ��
#define ADDR_NUM_SPEED_C_TIME	0x0131	//�Ӽ���ʱ��
#define ADDR_NUM_FAST_SPEED		0x0132	//����ٶ�
#define ADDR_NUM_WHOLE_SPEED	0x0133	//ȫ���ٶ�
#define ADDR_NUM_ANSWER_SHOW	0x0134	//������ʾ
#define ADDR_NUM_RETURN_IN		0x0136	//����������
#define ADDR_NUM_RETURN_OUT		0x0137	//����������
#define ADDR_NUM_M_UPDOWN_TIME	0x0138	//������ʱ��
#define ADDR_NUM_M_FR_TIME		0x0139	//������ʱ��
#define ADDR_NUM_V_UPDOWN_TIME	0x013A	//������ʱ��
#define ADDR_NUM_V_FR_TIME		0x013B	//������ʱ��
#define ADDR_NUM_CROSS_TIME		0x013C	//����ʱ��
#define ADDR_NUM_POSTURE_TIME	0x013D	//����ʱ��
#define ADDR_NUM_OPERATE1_TIME	0x013E	//�ӹ�1ʱ��
#define ADDR_NUM_OPERATE2_TIME	0x013F	//�ӹ�2ʱ��
#define ADDR_NUM_LIGHT			0x0140	//����
#define ADDR_NUM_READ_MODULE	0x0141	//����ģ��
#define ADDR_NUM_WRITE_MODULE	0x0142	//д��ģ��
#define ADDR_NUM_POSITION1		0x0143	//����1
#define ADDR_NUM_POSITION2		0x0145	//����2
#define ADDR_NUM_POSITION3		0x0147	//����3
#define ADDR_NUM_POSITION4		0x0149	//����4
#define ADDR_NUM_POSITION5		0x014B	//����5
#define ADDR_NUM_SPEED1			0x014D	//�ٶ�1
#define ADDR_NUM_SPEED2			0x014E	//�ٶ�2
#define ADDR_NUM_SPEED3			0x014F	//�ٶ�3
#define ADDR_NUM_SPEED4			0x0150	//�ٶ�4
#define ADDR_NUM_SPEED5			0x0151	//�ٶ�5
#define ADDR_NUM_TIME1			0x0152	//ʱ��1
#define ADDR_NUM_TIME2			0x0153	//ʱ��2
#define ADDR_NUM_TIME3			0x0154	//ʱ��3
#define ADDR_NUM_TIME4			0x0155	//ʱ��4
#define ADDR_NUM_TIME5			0x0156	//ʱ��5
#define ADDR_NUM_LINE			0x0157	//��ǰ�к�
#define ADDR_NUM_POSITION		0x0158	//����0
#define ADDR_NUM_SPEED			0x015A	//�ٶ�0
#define ADDR_NUM_TIME			0x015B	//ʱ��0
#define ADDR_NUM_SERIAL1		0x015C	//���1
#define ADDR_NUM_SERIAL2		0x015D	//���2
#define ADDR_NUM_SERIAL3		0x015E	//���3
#define ADDR_NUM_SERIAL4		0x015F	//���4
#define ADDR_NUM_NO1			0x0160	//���1
#define ADDR_NUM_NO2			0x0161	//���2
#define ADDR_NUM_NO3			0x0162	//���3
#define ADDR_NUM_NO4			0x0163	//���4
#define ADDR_NUM_RUN_SERIAL1	0x0164	//�������1
#define ADDR_NUM_RUN_SERIAL2	0x0165	//�������2
#define ADDR_NUM_RUN_SERIAL3	0x0166	//�������3
#define ADDR_NUM_RUN_SERIAL4	0x0167	//�������4
#define ADDR_NUM_MODULE1		0x0168	//ģ��1
#define ADDR_NUM_MODULE2		0x0169	//ģ��2
#define ADDR_NUM_MODULE3		0x016A	//ģ��3
#define ADDR_NUM_MODULE4		0x016B	//ģ��4
#define ADDR_NUM_RUN_TIME1		0x016C	//����ʱ��1
#define ADDR_NUM_RUN_TIME2		0x016E	//����ʱ��2
#define ADDR_NUM_RUN_TIME3		0x0171	//����ʱ��3
#define ADDR_NUM_RUN_TIME4		0x0173	//����ʱ��4
#define ADDR_NUM_SP1_TIME		0x0175	//Ԥһͨʱ
#define ADDR_NUM_SEND_TIME		0x0177	//����ͨʱ
#define ADDR_NUM_TRY_COUNTER	0x0179	//�Բ�ģ��
#define ADDR_NUM_CHECK_CODE		0x017B	//У����
#define ADDR_NUM_DECODE_NUM		0x017D	//��������

#define ADDR_WORD_ERROR_INFO	0x0200	//������ʾ��Ϣ
#define ADDR_WORD_RUN_STATUS	0x0220	//����״̬
#define ADDR_WORD_ACTION1		0x0230	//ִ�ж���1
#define ADDR_WORD_ACTION2		0x0240	//ִ�ж���2
#define ADDR_WORD_ACTION3		0x0250	//ִ�ж���3
#define ADDR_WORD_ACTION4		0x0260	//ִ�ж���4
#define ADDR_WORD_CROSS_MODE	0x0270	//����ģʽ
#define ADDR_WORD_ACTION_NOW	0x0280	//�Զ���ǰ����
#define ADDR_WORD_LANGUAGE		0x0290	//�����趨
#define ADDR_WORD_M_CLIP		0x02A0	//���м��
#define ADDR_WORD_V_CLIP		0x02B0	//���м��
#define ADDR_WORD_SNAP			0x02C0	//��ռ��
#define ADDR_WORD_HUG			0x02D0	//���߼��
#define ADDR_WORD_CLEAN_HARVEST	0x02E0	//��Ʒ����
#define ADDR_WORD_BEEP			0x02F0	//������
#define ADDR_WORD_THINBLE		0x0300	//����
#define ADDR_WORD_SAFE_GATE		0x0310	//��ȫ��
#define ADDR_WORD_MID_MODE		0x0320	//�а�ģ
#define ADDR_WORD_WAITE_POSTURE	0x0330	//��������
#define ADDR_WORD_WAITE_AREA	0x0340	//����λ��
#define ADDR_WORD_STOP_CONTROL	0x0350	//ֹͣ״̬
#define ADDR_WORD_OPEN_WARN		0x0360	//�ſ�����
#define ADDR_WORD_MOVE_INSIDE	0x0370	//ģ��Ƕ��
#define ADDR_WORD_AIR_JUDGE		0x0380	//ѹ�����
#define ADDR_WORD_BAD_JUDGE		0x0390	//����Ʒ���
#define ADDR_WORD_THINBLE_JUDGE	0x03A0	//������
#define ADDR_WORD_CLIP_SNAP		0x03B0	//�������
#define ADDR_WORD_AIR_SWITCH	0x03C0	//ѹ������
#define ADDR_WORD_AUTO_JUDGE	0x0550	//�ܻ�ȫ�Զ��ź�
#define ADDR_WORD_DOWN_INSIDE	0x03D0	//ģ���¿�											
#define ADDR_WORD_OUT_POSTURE	0x03E0	//�������
#define ADDR_WORD_IN_POSTUR		0x03F0	//��������
#define ADDR_WORD_GET_FAILD		0x0400	//ȡ��ʧ��
#define ADDR_WORD_DOWN_POSTURE	0x0410	//�½�����
#define ADDR_WORD_CROSS_KIND	0x0420	//����������
#define ADDR_WORD_M_DOWN		0x0430	//���½���
#define ADDR_WORD_M_FORWARD		0x0440	//��ǰ����
#define ADDR_WORD_M_BACKWARD	0x0450	//��������
#define ADDR_WORD_V_DOWN		0x0460	//���½���
#define ADDR_WORD_V_FORWARD		0x0470	//��ǰ����
#define ADDR_WORD_V_BACKWARD	0x0480	//��������
#define ADDR_WORD_SPEED_DOWN	0x0490	//��Ƶ����
#define ADDR_WORD_BACK_PULSE	0x04A0	//��������
#define ADDR_WORD_TEACH_ACTION1	0x04B0	//�̵�����1
#define ADDR_WORD_TEACH_ACTION2	0x04C0	//�̵�����2
#define ADDR_WORD_TEACH_ACTION3	0x04D0	//�̵�����3
#define ADDR_WORD_TEACH_ACTION4	0x04E0	//�̵�����4
#define ADDR_WORD_TEACH_ACTION5	0x04F0	//�̵�����5
#define ADDR_WORD_TEACH_ACTION	0x0500	//�̵�����0
#define ADDR_WORD_ERROR_INFO1	0x0510	//������Ϣ1
#define ADDR_WORD_ERROR_INFO2	0x0520	//������Ϣ2
#define ADDR_WORD_ERROR_INFO3	0x0530	//������Ϣ3
#define ADDR_WORD_ERROR_INFO4	0x0540	//������Ϣ4
#define ADDR_WORD_PUT_RESET		0x0550	//�ѵ�����
#define ADDR_WORD_PUT_DIRECTION	0x0558	//�ѵ�����
#define ADDR_WORD_MA_SG_SET		0x0560	//�ܻ��ź�
#define ADDR_WORD_POST_DIR		0x0568	//�ξ߷���

#endif
