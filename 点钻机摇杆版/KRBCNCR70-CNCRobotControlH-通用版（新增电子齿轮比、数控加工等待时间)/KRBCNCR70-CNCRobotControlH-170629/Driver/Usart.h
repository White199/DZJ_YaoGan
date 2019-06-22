/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __usart_h_
#define __usart_h_


#define  USART2_RS485
//#define  USART1_RS232

void USART3_SendData(unsigned char*,u8);
void USART1_SendDataArray(u8 , u8 , u8 *);
void USART1_SendData(u8,u8,u8*);
void USART1Acknowledge(void);
void Usart1DataDecode(void);

void USART2_SendDataArray(u8 , u8 , u8 *);
void USART2_SendData(u8,u8,u8*);
void Usart2Acknowledge( u8 );
void Usart2DataDecode(void);

/*-------------Usart1 parameter----------------*/
#define USART1_SEND_START  0xF5         //���ݷ��͵���ʼ�ź�
#define USART1_SEND_END    0xFA	       //���ݷ��͵Ľ����ź�
extern u8  USART1StartReceiveDataFlag;       //��ʼ�������ݱ�־
extern u8  USART1RecieveFinishedFlag;
extern u8  USART1RecieveCount;
extern u8  USART1ErrorFlag;	           //ͨ�ų����־λ
extern u8  USART1ReceiveDataBufferOverflow;  //������ɱ�־λ
extern u8  USART1SendBuff[60];         //�������ݻ���
extern u8  USART1ReceiveDataBuffer[60];      //�������ݻ���
extern u8  Usart1ReceiveData[60];	   //���ڽ�������
extern u8  USART1ReceiveDataCounter;		   //�������ݼ���
extern u8  USART1ReceiveDataLen;			   //��¼�������ݳ���
extern u8  USART1NewOrder;		           //���յ�������
extern u8  USART1DataDecodeFlag;		   //�������ݱ�־λ
extern u8  USART1StartReceiveDataTimeFlag ;  //���ڽ������ݼ�ʱ��־λ
extern u8  USART1StartReceiveDataTime ;
extern u8  USART1_Current_Order;
extern u8  USART1_Current_Key;
extern u8  USART1_Current_Num;


				   
#define DIR485_H  GPIO_SetBits(GPIOA,GPIO_Pin_1)  //GPIOA->BSRR=1<<1
#define DIR485_L  GPIO_ResetBits(GPIOA,GPIO_Pin_1)  //GPIOA->BRR=1<<1 


/*-------------Usart3_LCD parameter----------------*/
#define LCD_PROTOCOL1  0xA5
#define LCD_PROTOCOL2  0x5A

extern u8  LCDStartReceiveDataFlag;       //��ʼ�������ݱ�־
extern u8  LCDErrorFlag;	           //ͨ�ų����־λ
extern u8  LCDReceiveDataBufferOverflow;  //������ɱ�־λ
extern u8  LCDReceiveDataBuffer[20];      //�������ݻ���
extern u8  LCDReceiveData[20];	   //���ڽ�������
extern u8  LCDReceiveDataCounter;		   //�������ݼ���
extern u8  LCDReceiveDataLen;			   //��¼�������ݳ���
extern u8  LCDStartReceiveDataTimeFlag ;  //���ڽ������ݼ�ʱ��־λ
extern u8  LCDStartReceiveDataTime ;



#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/








