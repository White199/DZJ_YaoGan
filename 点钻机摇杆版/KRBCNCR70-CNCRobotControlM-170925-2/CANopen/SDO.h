/***********************************
 *           SDO Protocol          *
 ***********************************/
#define INITIATE_SDO_DOWNLOAD   0x20             //SDO�������أ�д��Servo����[4:7]:data-reponse:0x60 
#define INITIATE_SDO_UPLOAD     0x40             //SDO�����ϴ�����ȡServo����-reponse:0x40 [4:7]:data
//#define DOWNLOAD_SDO_SEGMENT                   //SDO������
//#define UPLOAD_SDO_SEGMENT                     //SDO���ϴ�
#define ABORT_SDO_TRANSFER      0x80             //��ֹSDOͨ��   [4:7]:abort codes

void Initiate_SDO_Download(void);
void Initiate_SDO_Upload(void);      
void Abort_SDO_Transfer(void);
void OD_Write(u8*,u16,u8);           //data,index,subindex
void SDO_Protocol(u8);               //SDOͨ��Э��
void SDO_Process(u8);              //SDO��Ϣ������
void Set_Servo_PP_Mode(u8);          //�ŷ���Profile Positionλ��ģʽ����	  
void Set_Servo_Homing_Mode(u8 );	 //�ŷ���Homing Mode��ԭ��ģʽ����
void Set_Guard_Time(u8);
void Set_PUU(u8,u32,u32);
void Servo_Off(u8);
void Fault_Reset(u8);
void Servo_Off(u8);
void Servo_On(u8);
void Query_Servo_parameter(u8,u16,u8);
void Change_Polarity_Positive(u8);
void Change_Polarity_Negtive(u8);
//---
void SDO_Process_PositionMode(u8);
void SDO_Process_HomingMode(u8);

