/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ****************
* File Name          : Usart.c
* Author             : 
* Version            : 
* Date               : 
* Description        : This file provides USART communicated functions.
******************************************************************************/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "Usart1_Robot.h"
//#include "Usart.h"
#include "DGUS.h"
#include "Delay.h"

/*-------------Usart1 parameter----------------*/
u8  USART1StartReceiveDataFlag=FALSE;        //��ʼ�������ݱ�־
u8  USART1RecieveFinishedFlag = FALSE;		//ͨ����ɱ�־λ������Э�鷴��
u8  USART1RecieveCount = 0;				   //ͨ�Ž��ռ�ʱ��־λ�����1s��δ���յ��������
u8  USART1ErrorFlag=FALSE;	           //ͨ�ų����־λ
u8  USART1ReceiveDataBufferOverflow=FALSE;   //������ɱ�־λ
u8	USART1NewOrder=FALSE;			           //���յ��µ�����
u8  USART1DataDecodeFlag = FALSE;		   //
u8  USART1StartReceiveDataTimeFlag = FALSE;  //���ڽ������ݼ�ʱ��־λ
u8  USART1StartReceiveDataTime = 0;          //���ڽ������ݼ�ʱ 

u8  USART1SendBuff[60]={0};            //�������ݻ���
u8  USART1ReceiveDataBuffer[60]={0};         //�������ݻ���

u8  USART1ReceiveData[20]={0};		   //���ڽ��յ�������
u8  USART1ReceiveDataCounter=0;		       //�������ݼ���
u8  USART1ReceiveDataLen=0;				   //��¼�������ݳ���
u8  USART1temp[4] = {0x01,0x02,0x03,0x04};
u8  USART1_Current_Order = 0;			   //����1��ǰ��ѯ����
u8  USART1_Current_Key = 0;				   //����1��ǰ��ѯkey
u8  USART1_Current_Num = 0;				   //����1��ǰ��ѯNum

/*-------------Usart3_LCD parameter----------------*/
u8  LCDStartReceiveDataFlag=FALSE;        //��ʼ�������ݱ�־
u8  LCDErrorFlag=FALSE;	                  //ͨ�ų����־λ
u8  LCDReceiveDataBufferOverflow=FALSE;   //������ɱ�־λ
u8  LCDStartReceiveDataTimeFlag = FALSE;  //���ڽ������ݼ�ʱ��־λ
u8  LCDStartReceiveDataTime = 0;          //���ڽ������ݼ�ʱ 

u8  LCDReceiveDataBuffer[20]={0};         //�������ݻ���
u8  LCDReceiveData[20]={0};		          //���ڽ��յ�������
u8  LCDReceiveDataCounter=0;		      //�������ݼ���
u8  LCDReceiveDataLen=0;				  //��¼�������ݳ���
u8  LCDtemp[4] = {0x01,0x02,0x03,0x04};



void UART1_SendByte(u16 Data)
{ 
   while (!(USART1->SR & USART_FLAG_TXE));    
   USART_SendData(USART1, Data); 
}

void UART1Write(vu8* data,u16 len)
{
	u16 i;
	for (i=0; i<len; i++)
	{
		UART1_SendByte(data[i]);
	}
}


void USART1_SendDataArray(u8 DataLen, u8 Order, u8 *Data)
{
	u8 i=3;
//	u8 j=0;
	USART1_Current_Order = 0;
//	while(StartReceiveDataFlag==TRUE);                    //�ȴ���һ�ε����ݽ������
	while(DMA_GetCurrDataCounter(DMA1_Channel4)!= 0);     //�ȴ���һ��DMA��ɴ�������
	//while(DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET);   //�ȴ���һ��DMA��ɴ�������
	DMA_ClearFlag(DMA1_FLAG_TC4);
	DMA_Cmd(DMA1_Channel4, DISABLE);
	USART1SendBuff[0]=USART1_SEND_START;  //�������ݿ�ʼ��־	
	USART1SendBuff[1]=DataLen+3;		 //���ݳ���   
	USART1SendBuff[2]=Order; 		     //��������
	USART1_Current_Order = Order;
	USART1_Current_Key = Data[0];
	if(DataLen>=3)
	{
		USART1_Current_Num = Data[2];
	}
	if(DataLen>0)
	{
		for(i=3;i<DataLen+3;i++)
		{
			USART1SendBuff[i]=Data[i-3];//��������
		}
	}
	USART1SendBuff[i]=USART1_SEND_END;	 //�������ݽ�����־
	i++;
	DMA1_Channel4->CNDTR = i;
	DMA_Cmd(DMA1_Channel4, ENABLE);	
//	USART_ClearFlag(USART1,USART_FLAG_TC);
//	for(j=0;j<i;j++)
//	{			
//		USART1->DR = USART1SendBuff[j];
//		while((USART1->SR&0X40)==0);	
//	}		
}


void USART2_SendDataArray(u8 DataLen, u8 Order, u8 *Data)
{
	u8 i=3;
	u8 j=0;
	USART1_Current_Order = 0;
	USART1SendBuff[0]=USART1_SEND_START;  //�������ݿ�ʼ��־	
	USART1SendBuff[1]=DataLen+3;		 //���ݳ���   
	USART1SendBuff[2]=Order; 		     //��������
	USART1_Current_Order = Order;
	USART1_Current_Key = Data[0];
	if(DataLen>=3)
	{
		USART1_Current_Num = Data[2];
	}
	if(DataLen>0)
	{
		for(i=3;i<DataLen+3;i++)
		{
			USART1SendBuff[i]=Data[i-3];//��������
		}
	}
	USART1SendBuff[i]=USART1_SEND_END;	 //�������ݽ�����־
	i++;
	USART_ClearFlag(USART2,USART_FLAG_TC);
	DIR485_H;  //д����
	for(j=0;j<i;j++)
	{			
		USART2->DR = USART1SendBuff[j];//---�����������ݴ������ݼĴ���DR
		while((USART2->SR&0X40)==0);	
	}
	DIR485_L;  //������	
}


/*******************************USART1 Control_Board*********************************/
void USART1_SendData(u8 DataLen, u8 Order, u8 *Data)        //����δ֪���ȵ�����
{	
	u8 static Sendcount1=0;
	USART1ErrorFlag = FALSE;
	while(Sendcount1<100)
	{
		USART1ErrorFlag=TRUE;
		Sendcount1++;
		#ifdef USART1_RS232
			USART1_SendDataArray(DataLen,Order,Data);
		#endif 
		#ifdef USART2_RS485
			USART2_SendDataArray(DataLen,Order,Data);
		#endif 
		USART1Acknowledge();   //����1����Ӧ������ 
		if(USART1RecieveFinishedFlag == TRUE)
		{
			USART1ErrorFlag=FALSE;
			break;
		}
	}
	Sendcount1=0;
}


/*.....................LCD....................*/

void USART3_SendData(unsigned char *send_buff,u8 length)
{
    u8 i = 0;
	USART_ClearFlag(USART3,USART_FLAG_TC);
	for(i = 0;i < length;i ++)
	{			
		USART3->DR = send_buff[i];
		while((USART3->SR&0X40)==0);	
	}	
}

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/
