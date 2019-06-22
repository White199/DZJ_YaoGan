/******************** COPYRIGHT 2013 KingRobot *********************************
* File Name          : NMT.c
* Author             : WuXiang
* Version            : V1.0.0
* Date               : 11/14/2013
* Description        : This file complete NMT protocol
********************************************************************************/
#include "CANopen.h"

CanTxMsg NMT_TxMessage;        //NMT���͵�CAN����

u8  NMT_TransmitMailbox;       //���صķ�������״̬
vu8 Flag_Get_Node_State=0;     //�õ��ڵ�ͨ��״̬��ʶλ

/***********************************************
 *              Send_NMT_Protocol              *
 ***********************************************/
/***************************************************************************
**  ��������  Send_NMT_Protocol()
**	�����������
**	�����������
**	�������ܣ�����NMTЭ����Ϣ
**	��ע��	  ��
**  ���ߣ�    ����    
**  �������ڣ�2013/11/14
***************************************************************************/
/***********state tansition*********/
void Send_NMT_Protocol(u8 NMT_Node_ID , u8 NMT_cs)
{
  /* transmit */
  NMT_TxMessage.StdId   = 0x00;                  // COB_ID=0
  NMT_TxMessage.RTR     = CAN_RTR_DATA;          // �趨��������Ϣ֡���ͣ�_DATA-����֡��_REMOTE-Զ��֡
  NMT_TxMessage.IDE     = CAN_ID_STD;            // �趨��Ϣ��ʶ�����ͣ�_STD-��׼��ʶ����_EXT-��׼+��չ��ʶ��
  NMT_TxMessage.DLC     = 2;                     // ȷ��������Ϣ��֡���ȣ�0~0x8 
  NMT_TxMessage.Data[0] = NMT_cs;                // CMD specifier��������
  NMT_TxMessage.Data[1] = NMT_Node_ID;           // �ŷ���ID
 
  NMT_TransmitMailbox=CAN_Transmit(&NMT_TxMessage);
  CANopen_k= 0;
  while((CAN_TransmitStatus(NMT_TransmitMailbox) != CANTXOK) && (CANopen_k!= 0xFF)) 
  {                                               //ͨ�����CANTXOKλ��ȷ�Ϸ����Ƿ�ɹ�
      CANopen_k++;
  }
}

/***********get node state*********/
void Get_Node_State()
{
     u8 state=0;
   //  state = CAN_Recieve_Data[0]&0x7f ;         //�õ�״ֵ̬[6:0]   
     switch(state)
     {
        case 0  :  State_Machine = Initialisation; break;
        case 4  :  State_Machine = Stop;           break;
        case 5  :  State_Machine = Operate;        break;
        case 127:  State_Machine = Pre_Operate;    break;
        default :  break;       
     }
}

/*******Error_Control protocol*********/
/*    Boot-up    and   Node Gurding   */
void Send_Node_Guard( u8 Node_ID )
{
  NMT_TxMessage.StdId   = 0x700 + Node_ID ;      // COB_ID=0x700 + Node_ID
  NMT_TxMessage.RTR     = CAN_RTR_REMOTE;          // �趨��������Ϣ֡���ͣ�_DATA-����֡��_REMOTE-Զ��֡
  NMT_TxMessage.IDE     = CAN_ID_STD;            // �趨��Ϣ��ʶ�����ͣ�_STD-��׼��ʶ����_EXT-��׼+��չ��ʶ��
 
  NMT_TransmitMailbox=CAN_Transmit(&NMT_TxMessage);
  CANopen_k = 0;
  while((CAN_TransmitStatus(NMT_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF)) 
  {                                              //ͨ�����CANTXOKλ��ȷ�Ϸ����Ƿ�ɹ�
      CANopen_k++;
  }
} 

void Boot_Up( u8 Node_ID )
{
  NMT_TxMessage.StdId   = 0x700 + Node_ID ;      // COB_ID=0x700 + Node_ID
  NMT_TxMessage.RTR     = CAN_RTR_DATA;          // �趨��������Ϣ֡���ͣ�_DATA-����֡��_REMOTE-Զ��֡
  NMT_TxMessage.IDE     = CAN_ID_STD;            // �趨��Ϣ��ʶ�����ͣ�_STD-��׼��ʶ����_EXT-��׼+��չ��ʶ��
  NMT_TxMessage.DLC     = 1;                     // ȷ��������Ϣ��֡���ȣ�0~0x8 
  NMT_TxMessage.Data[0] = 0x00;                  // CMD specifier��������
 
  NMT_TransmitMailbox=CAN_Transmit(&NMT_TxMessage);
  CANopen_k = 0;
  while((CAN_TransmitStatus(NMT_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF)) 
  {                                              //ͨ�����CANTXOKλ��ȷ�Ϸ����Ƿ�ɹ�
      CANopen_k++;
  }
}

/*******����Զ�̽ڵ�*********/
void Start_Remote_Node(u8 Node_ID)             //0x01
{
  Send_NMT_Protocol(Node_ID , START_REMOTE_NODE) ;
  State_Machine = Operate;
}

/*******ֹͣԶ�̽ڵ�*********/
void Stop_Remote_Node(u8 Node_ID)              //0x02
{
  Send_NMT_Protocol(Node_ID , STOP_REMOTE_NODE);
  State_Machine = Stop;
}

/*******����Ԥ����״̬*********/
void Enter_Pre_Operational_State(u8 Node_ID)   //0x80 
{
   Send_NMT_Protocol(Node_ID , ENTER_PRE_OPERATIONAL_STATE);
   State_Machine = Pre_Operate;
}

/*******��λ�ڵ�*********/
void Reset_Node(u8 Node_ID)                    //0x81
{
   Send_NMT_Protocol(Node_ID , RESET_NODE);
   State_Machine = Initialisation;  
}

/*******��λͨ��*********/
void Reset_Communication(u8 Node_ID)           //0x82
{                                                         
   Send_NMT_Protocol(Node_ID , RESET_COMMUNICATION);
   State_Machine = Initialisation;
}
