
void PDO_Send_Message(CanTxMsg*);      //PDO���ͱ��ĺ���
void ServoStart_PDO(u8,u32,u32);                //PDO������
void ServoStop_PDO(u8);
void ServoEmergencyStop_PDO(u8);
void New_Set_Point_Set(u8);
void New_Set_Point_Reset(u8);
void Query_stWord_acPosition(u8);      //��ѯstatus word��Position actual value
void Get_Status_Position(void);            //����CAN����ֵ�õ�Reachede��Actual_Position
void Change_Set(u8);
void Homing_Operation_Start_Set(u8);
void Homing_Operation_Start_Reset(u8);
void Home_Stop_Axis(u8);
void Set_Halt(u8);
void Reset_Halt(u8);
void Change_Servo_State_Machine(u8,u16);
//---
u8 Can_Send_Msg(u8* msg,u8 len);
u8 Can_Receive_Msg(u8 *buf);
extern u16 Actual_Status_1[6];
extern u16 Actual_Status[6];
extern u16 STD_ID ; 
extern u16 Homing_attained_status[6];
extern u16 Target_Rreached[6];             //---2017.8.22Ŀ��ﵽ״̬��

extern u8 Axsis_Stop_Flag[6];
extern u8 Set_Halt_Flag[6];
