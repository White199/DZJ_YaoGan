/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __usart_h_
#define __usart_h_

#define USART_SEND_START  0xf5         //���ݷ��͵Ľ����ź�
#define USART_SEND_END    0xfa	       //���ݷ��͵���ʼ�ź�

#define DIR485_H  GPIO_SetBits(GPIOF,GPIO_Pin_7)   //GPIOF->BSRR=1<<7
#define DIR485_L  GPIO_ResetBits(GPIOF,GPIO_Pin_7) //GPIOF->BRR=1<<7

void UART1_SendByte(u16 dat);
void UART1Write(vu8* data,u16 len);
void USART1_SendData(u8,u8,u8*);
void UsartAcknowledge( u8 );
void UsartDataDecode(void);

extern u8  StartReceiveDataFlag;       //��ʼ�������ݱ�־
extern u8  USART1ErrorFlag;	           //ͨ�ų����־λ
extern u8  ReceiveDataBufferOverflow;  //������ɱ�־λ
extern u8  SendDataBuffer[60];         //�������ݻ���
extern u8  ReceiveDataBuffer[60];      //�������ݻ���
extern u8  UsartReceiveData[60];	   //���ڽ�������
extern u8  ReceiveDataCounter;		   //�������ݼ���
extern u8  ReceiveDataLen;			   //��¼�������ݳ���
extern u8  NewOrder;		           //���յ�������
extern u8  UsartDataDecodeFlag;		   //�������ݱ�־λ
extern u8  StartReceiveDataTimeFlag ;  //���ڽ������ݼ�ʱ��־λ
extern u8  StartReceiveDataTime ;

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/
