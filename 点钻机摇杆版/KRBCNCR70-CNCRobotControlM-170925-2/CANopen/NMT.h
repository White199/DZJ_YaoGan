#define START_REMOTE_NODE           0x01
#define STOP_REMOTE_NODE            0x02
#define ENTER_PRE_OPERATIONAL_STATE 0x80
#define RESET_NODE                  0x81
#define RESET_COMMUNICATION         0x82

/***********NMT����״̬֮���ת�ƣ���ע��Ϊ������***************/ 
void Send_NMT_Protocol(u8 , u8);        //����NMT�������״̬�л�
void Send_Node_Guard(u8);               //COB_ID=700+Node_ID��
void Boot_Up(u8);                       //    : Initianlising->Pre-Operational                      
void Reset_Node(u8);                    //0x81: Set the NMT Slave to "reset apllication"sub-state
void Reset_Communication(u8);           //0x82��Set the NMT Slave to "reset communication"sub-state
void Enter_Pre_Operational_State(u8);   //0x80: Set the NMT Slave to Pre-Operational
void Start_Remote_Node(u8);             //0x01��Set the NMT Slave to Operational
void Stop_Remote_Node(u8);              //0x02: Set the NMT Slave to Stopped                                                             
void Get_Node_State(void);              //state=data[0]&0x7f













