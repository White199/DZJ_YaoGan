#include "stm32f10x_lib.h"
#include "Delay.h"
#include "PDO.h"
#include "SDO.h"
#include "NMT.h"
#include "Sync.h"
#include "EMCY.h"
#include "SpeedControl_CAN.h" 

#define  CANOPEN_MODE 
//#define PULSE_MODE 
//#define NMT              
//#define SDO              
//#define PDO              
//#define SYNC        
#define SERVO_NODE_ID_00 0     
#define SERVO_NODE_ID_01 1     
#define SERVO_NODE_ID_02 2
#define SERVO_NODE_ID_03 3
#define SERVO_NODE_ID_04 4
#define SERVO_NODE_ID_05 5  
#define SERVO_NODE_ID_06 6  
#define SINGLE_SET_POINT       (u8)0
#define CHANGE_SET_IMMEDIATELY (u8)1
#define PROFILE_POSITION_MODE  1
#define HOMING_MODE            6

#define  SHUTDOWN           0X0006
#define  SWITCH_ON          0X0007
#define  ENABLE_OPERATION   0X000F 
#define  DISABLE_OPERATION  0X0005 
#define  QUICK_STOP         0X0002
#define  DISABLE_VOLTAGE    0X0000

/***********************************
 *            variable             *
 ***********************************/
typedef enum { Initialisation=0, Pre_Operate=127, Operate=5, Stop=4 } State_Machine_state ;
typedef enum { NMT=0, SDO=1, PDO=2, SYNC=3 } Function ; 

typedef struct  //OD,�����ֵ�ṹ��
{
  u16 index;
  u8  subindex;
  u8  ctr;
  u16 length;
}DICT_OBJECT ;

typedef struct   //PDO message
{
   u32 PDO_COB_ID;
   u8  len;
   u8  PDO_data[8];
}PDO_Struct;

typedef struct   //SDO-8�ֽ����ݸ�ʽ
{
  u8 SDO_Scs;    //������
  u8 data[7];
}SDO_Mes;

typedef struct   //SDO message
{
  u8  Node_ID;
  u16 index;
  u8  sub_index;
  u8  len;      //�������ݵĳ��ȣ�+4��Ϊ�ܵ�can���ĳ���
  u8  Data[8];
}SDO_Struct;


extern State_Machine_state  State_Machine;    //״̬��
extern Function             Function_Code;    //������
extern SDO_Struct           SDO_Message;      //SDO��Ϣ֡
extern u8   ServoNodeID_0 ;                 //�ڵ�ID��
extern u8   ServoNodeID_1 ;                 //�ڵ�1��ID��
extern u8   ServoNodeID_2 ;                 //�ڵ�2��ID��
extern u8   ServoNodeID_3 ;                 //�ڵ�3��ID��
extern u8   ServoNodeID_4 ;                 //�ڵ�4��ID��
extern u8   ServoNodeID_5 ;                 //�ڵ�5��ID��
extern u8   ServoNodeID_6 ;                 //�ڵ�6��ID��
extern vu8  Servo_Node_ID ;                 //�ŷ����ڵ�ID�ű���
extern vu16 COB_ID ;                        //��������ID
extern vu16 Command_Specifier;              //cs:�����
extern u32  CANopen_k ;
extern vu8  CAN_Recieve_Data[8];
extern vu8  Send_Node_guard;                  //Node_guard��Ϣ���ͱ�ʾλ
//extern u8   Servo_On;                         //�ŷ���Servo on
extern vs32 PDO_Position;                     //���͸��ŷ�����λ��ֵ
extern vu32 PDO_Velocity;                     //���͸��ŷ������ٶ�ֵ
extern u8   Flag_Reached_Point;               //�Ƿ񵽴����λ�ñ�־λ
extern u8   Reached;                          //״̬��6041h�ĵ�10λReached��־λ
extern u8   Set_Point_Acknowledge;            //�켣�������Ƿ��趨�ٶ�ֵ
extern u8   Change_Velocity;
extern u8   Get_Position_zero;                //�Ƿ�ﵽλ��0
extern vu8  Flag_Get_Node_State;              //�õ��ڵ�ͨ��״̬��ʶλ
extern vu8  Flag_Query_Status_Position;       //�Ƿ�������״̬��λ�ò�ѯ��ʶλ
extern u8   Flag_Chang_Operation_Mode  ;
extern u8   Change_Set_Immediately;           //�ж��ڵõ���λ��ֵ��ʱ���Ƿ��ٶȽ�Ϊ0�ٸı�
extern vu16 Controlword;                      //״̬��


extern vu32 Target_Servo_Position ;           //�ŷ�����˶���Ŀ��λ��ֵ
extern vu32 Target_Servo_Velocity ;           //�ŷ�����˶���Ŀ���ٶ�ֵ
extern vu32 Target_Servo_Acceleration ;      //�ŷ�����˶��ļ��ٶ�ֵ
extern vu32 Target_Servo_Deceleration ;      //�ŷ�����˶��ļ��ٶ�ֵ
/***********************************
 *          Machine State          *
 ***********************************/
void Pre_Operational(void);
void Operational(void);
void Stopped(void);
void RPDO_Mapping(u8);
void TPDO_Mapping(u8);
void PDO_Mapping_Deleted(u8);
void PDO_Mapping_test(u8);
void Not_Valid_PDO(u8);
void CANopen_Init(void);
void Chang_Operation_Mode(u8,u8)	;
//---
void PP_Mode_Init(void);   
void Homing_Mode_Init(void);
/***********************************
 *          User  Fuction          *
 ***********************************/
State_Machine_state  Get_state(void);
Function      Get_Fuction_Code(vu16); //COB_ID

