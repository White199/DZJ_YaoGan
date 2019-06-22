/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Usart.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "Usart.h" 
#include "Error.h"
#include "SignalWatch.h"
#include "StatusControl.h"
#include "Delay.h"

u8  StartReceiveDataFlag=FALSE;        //��ʼ�������ݱ�־
u8  USART1ErrorFlag=FALSE;	           //ͨ�ų����־λ
u8  ReceiveDataBufferOverflow=FALSE;   //������ɱ�־λ
u8	NewOrder=FALSE;			           //���յ��µ�����
u8  UsartDataDecodeFlag=FALSE;		   //
u8  StartReceiveDataTimeFlag = FALSE;  //���ڽ������ݼ�ʱ��־λ
u8  StartReceiveDataTime = 0;          //���ڽ������ݼ�ʱ 

u8  SendDataBuffer[60]={0};            //�������ݻ���
u8  ReceiveDataBuffer[60]={0};         //�������ݻ���
u8  UsartReceiveData[60]={0};		   //���ڽ��յ�������
u8  ReceiveDataCounter=0;		       //�������ݼ���
u8  ReceiveDataLen=0;				   //��¼�������ݳ���
u8  temp[6] = {0x01,0x02,0x03,0x04,0x05,0x06};
u8  Origin_data[3]={0x00,0x00,0x00};

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


/*******************************��������ʱ�ķ���Э�� *********************************/
void USART1_SendData(u8 DataLen, u8 Order, u8 *Data)        //����δ֪���ȵ�����
{	
	u8 i=3;
	u8 j=0;
////	while(StartReceiveDataFlag==TRUE);                    //�ȴ���һ�ε����ݽ������
//	while(DMA_GetCurrDataCounter(DMA1_Channel4)!= 0);     //�ȴ���һ��DMA��ɴ�������
//	//while(DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET);   //�ȴ���һ��DMA��ɴ�������
//	DMA_ClearFlag(DMA1_FLAG_TC4);
//	DMA_Cmd(DMA1_Channel4, DISABLE);
	SendDataBuffer[0]=USART_SEND_START;  //�������ݿ�ʼ��־	
	SendDataBuffer[1]=DataLen+3;		 //���ݳ���   
	SendDataBuffer[2]=Order; 		     //��������
	if(DataLen>0)
	{
		for(i=3;i<DataLen+3;i++)
		{
			SendDataBuffer[i]=Data[i-3];//��������
		}
	}
	SendDataBuffer[i]=USART_SEND_END;	 //�������ݽ�����־
	i++;
	USART_ClearFlag(USART1,USART_FLAG_TC);
	DIR485_H;  //д����
	for(j=0;j<i;j++)
	{			
		USART1->DR = SendDataBuffer[j];
		while((USART1->SR&0X40)==0);	
	}
	DIR485_L;  //������	

//	DIR485_H;
//	DMA1_Channel4->CNDTR = i;
//	DMA_Cmd(DMA1_Channel4, ENABLE);	
//	DIR485_L;
//	//DelayNms(10);
}


/**************************************************************************************************
**  ��������  ManualMode()
**	�����������
**	�����������
**	�������ܣ��ֶ�ģʽ�µĸ��ֶ���
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void UsartAcknowledge(u8 Order)
{
	 switch(Order)
	 {
		case P_ROBOT_ENABLE_A_ORIGIN:	
		     USART1_SendData(1,0xA0,&temp[0]);
		     break; 
		case P_WORK_MODE:	
		     USART1_SendData(1,0xB0,&temp[0]);
			 break;            
	    case P_AUTO_RUN:
	  		 USART1_SendData(1,0xC0,&temp[0]);
		     break;
		case P_FREE_PROGRAM_SEND:	
		     USART1_SendData(1,0xD0,&temp[1]);
			 break; 

		case P_WATCH_COMMAND:		     
			 break; 		   

		case P_READ_IIC:		     
			 break; 

		case P_IO_DEBUG_OUTPUT1:	
		     USART1_SendData(1,0xA1,&temp[2]);
			 break; 		   		   		   		   		   		   		    		   		   		   	
		case P_IO_DEBUG_OUTPUT2:	
		     USART1_SendData(1,0xB1,&temp[3]);
			 break; 
		case P_IO_DEBUG_OUTPUT3:	
		     USART1_SendData(1,0xC1,&temp[4]);
			 break; 
	    case P_MANUL_DEBUG:
		     USART1_SendData(1,0xD1,&temp[5]);
	         break;
	    case P_PARAMETER_ORDER:
		     USART1_SendData(1,0xE1,&temp[5]);
	         break;
		default: 
		     break;		  			  			  	 
	 }

}


/**************************************************************************************************
**  ��������  UsartDataDecode()
**	�����������
**	�����������
**	�������ܣ����봮�ڽ������ݣ�����ջ���
**	��ע��	  UsartReceiveData[]���е����ݣ�������ʼ��������־λ�������ݳ���λ
**			  UsartReceiveData[0]=ReceiveDataBuffer[1]
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void UsartDataDecode()
{
    u8 i=0;
	for(i=0;i<ReceiveDataLen-1;i++)
	{
		UsartReceiveData[i]=ReceiveDataBuffer[i+1];
	}
	for(i=0;i<60;i++)
	{
	    ReceiveDataBuffer[i]=0;
	}
	ReceiveDataLen = 0;
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
