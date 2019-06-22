#include "CANopen.h"

CanTxMsg EMCY_TxMessage;
u8 EMCY_TransmitMailbox;
    
/***********************************************
 *              Send_NMT_Protocol              *
 ***********************************************/
void Process_EMCY()
{
  EMCY_TxMessage.StdId   = 0x80+0x01;             // COB_ID=0x80
  EMCY_TxMessage.RTR     = CAN_RTR_DATA;          // �趨��������Ϣ֡���ͣ�_DATA-����֡��_REMOTE-Զ��֡
  EMCY_TxMessage.IDE     = CAN_ID_STD;            // �趨��Ϣ��ʶ�����ͣ�_STD-��׼��ʶ����_EXT-��׼+��չ��ʶ��
  EMCY_TxMessage.DLC     = 8;                     // ȷ��������Ϣ��֡���ȣ�0~0x8 
  EMCY_TxMessage.Data[0] = 0x00;               
  EMCY_TxMessage.Data[1] = 0x00;               
  EMCY_TxMessage.Data[2] = 0x00;
  EMCY_TxMessage.Data[3] = 0x00;
  EMCY_TxMessage.Data[4] = 0x00;
  EMCY_TxMessage.Data[5] = 0x00;
  EMCY_TxMessage.Data[6] = 0x00;
  EMCY_TxMessage.Data[7] = 0x00;
  EMCY_TransmitMailbox=CAN_Transmit(&EMCY_TxMessage);
  CANopen_k = 0;
  while((CAN_TransmitStatus(EMCY_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF)) 
  {                                            
     CANopen_k++;
  }
}
