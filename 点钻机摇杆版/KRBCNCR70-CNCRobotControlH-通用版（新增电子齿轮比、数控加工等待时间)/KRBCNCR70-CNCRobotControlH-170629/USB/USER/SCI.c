
#include "stm32f10x.h"
//#include "SCI.H"
//#include "misc.h"
//#include "glovar.h"
//������DSPͨ�ű���
u8  DSP_Data_Num=0;      //���յ����ݸ���
u8  DSP_Data[40]={0};    //���յ�������
u8  DSP_CheckCodeTemp=0; //DSP��������У�����ݴ棨�ֲ���
u8 char_count=0;   //��ǽ���DSP����λ��

//������DGUSͨ�ű���
u8  LCD_Data_Num=0;      //���յ����ݸ���
u8  Lcd_Data[30]={0};    //���յ�������


void LED_Configuration(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable the GPIO_LED Clock */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE , ENABLE); 
						 
			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_SetBits(GPIOE , GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
}

void USART_Configuration(void)
{ 
  
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; 

 /********************���ذ��DGUS֮���ͨ��ʹ�ô���1**************************/
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO,ENABLE);
  /*
  *  USART1_TX -> PA9 , USART1_RX ->PA10
  */				
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);		   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ����
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//1λֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;//��У��
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //���ͺͽ���ʹ��
  USART_Init(USART1, &USART_InitStructure); 
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART1, ENABLE); 
  USART_ClearITPendingBit(USART1, USART_IT_TC);//����ж�TCλ



 /********************����ΪUSART2����**************************/
 /*****************DSP�����ذ�֮��ͨ��ʹ�ô���2**************************/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
  /*
  *  USART2_TX -> Pd5 , USART2_RX ->	PD6
  */				
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);		   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ����
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//1λֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;//��У��
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //���ͺͽ���ʹ��
  USART_Init(USART2, &USART_InitStructure); 
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART2, ENABLE); 
  USART_ClearITPendingBit(USART2, USART_IT_TC);//����ж�TCλ


 /********************����ΪUSART3����**************************/
 /*****************���ڽ�FLASH�е�PLT�ļ����͸�DSPоƬ**************************/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
  /*
  *  USART3_TX -> PB10 , USART3_RX ->	PB11
  */				
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);		   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ����
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//1λֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;//��У��
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //���ͺͽ���ʹ��
  USART_Init(USART3, &USART_InitStructure); 
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART3, ENABLE); 
  USART_ClearITPendingBit(USART3, USART_IT_TC);//����ж�TCλ


}

/***********************************************************************
�������ƣ�USART2_IRQHandler  ����2  �ж�
��    �ܣ����SCI�����ݵĽ��գ�������ʶ
���������
���������
��дʱ�䣺2014.9.11
�� д �ˣ��Ҳ���
ע    ��  ���յ���DSP���ݷ��͸����ذ��õ���USART2.
***********************************************************************/
//void USART2_IRQHandler(void)  
//{
//	u8 usart1_ch;
//	static u8 len = 0;								  
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET && Receive_DSP_Flag == 0)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
//	{	
//		USART_ClearITPendingBit(USART2, USART_IT_TC);
//		GPIO_SetBits( GPIOE,  GPIO_Pin_2); 
//		usart1_ch =USART2->DR;				//��ȡ���յ�������
//		ReceiveBuff[char_count]=usart1_ch;
//		if(ReceiveBuff[char_count]==0xED && StartRecive == 4)
//		{				  
//			  char_count=0;
//			  Receive_DSP_Flag=1;
//			  StartRecive=0;			
//		}
//		else if(StartRecive == 4)
//		{
//			  char_count=0;
//			  Receive_DSP_Flag=0;
//			  StartRecive=0;
//		}
//		else
//		{	
//			if(StartRecive == 3 && len == 0)
//			{
//				 StartRecive++;
//			}
//			else if(StartRecive == 3)
//			{
//				len--;
//			}
//			if(StartRecive == 2)
//			{
//				StartRecive++;
//				len = ReceiveBuff[char_count];
//			}
//			if(StartRecive==1)
//				StartRecive++;
//			if(StartRecive==0 && ReceiveBuff[char_count]==0x68)
//			{
//			      StartRecive=1;		
//			} 
//			if(StartRecive > 0)
//				char_count++;
//		}	
//	}
//	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//	u8 usart1_ch;								  
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
//	{	
//		
//		GPIO_SetBits( GPIOE,  GPIO_Pin_2); 
//		usart1_ch =USART2->DR;				//��ȡ���յ�������
//		ReceiveBuff[char_count]=usart1_ch;
//		if(StartRecive==0 && ReceiveBuff[char_count]==0x68)
//		{
//		      StartRecive=1;		
//		} 
//		char_count++;
//		if(ReceiveBuff[char_count-1]==0xED)
//		{				  
//			  char_count=0;
//			  Receive_DSP_Flag=1;
//			  StartRecive=0;			
//		}	
//		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//	}
//}
/***********************************************************************
�������ƣ�void USART1_IRQHandler(void) 
��    �ܣ����SCI�����ݵĽ��գ�������ʶ
���������
���������
��дʱ�䣺2014.9.11
�� д �ˣ�
ע    �⣺����DGUS�������������õ���USART1
***********************************************************************/
//void USART1_IRQHandler(void)  
//{
//	u8 i=0;
//	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET && Receive_Lcd_Flag == 0)
//	{ 	
//		USART_ClearITPendingBit(USART1, USART_IT_TC);			
//		Lcd_Data[LCD_Data_Num]=USART1->DR;
//		LCD_Data_Num++;		 //һ���ֽ�һ���ֽڵķ��͵�����ÿ����һ���ֽھͽ�һ���ж�
//		GPIO_ResetBits(GPIOE , GPIO_Pin_3);	//����2ͨ��ָʾ��
//		if(Lcd_Data[2]==0x08) //����8������
//		{
//			if(LCD_Data_Num>10)
//			{		
//				if(Lcd_Data[0]==0x05&&Lcd_Data[1]==0x50&&Lcd_Data[3]==0x83)
//				{
//					{
//						Receive_Lcd_Flag=1;
//						g_AdrID=(Lcd_Data[4]<<8)&0xFF00;
//						g_AdrID+=Lcd_Data[5]&0x00FF;
//						g_KeyID=(Lcd_Data[7]<<24)&0xFF000000;
//						g_KeyID+=(Lcd_Data[8]<<16)&0x00FF0000;
//						g_KeyID+=(Lcd_Data[9]<<8)&0x0000FF00;
//						g_KeyID+=(Lcd_Data[10])&0x000000FF;
//					} 	
//				} 
//	 		}
//		}
//		else if(Lcd_Data[2]==0x06)//����6������
//		{
//			if(LCD_Data_Num>8)
//			{
//				if(Lcd_Data[0]==0x05&&Lcd_Data[1]==0x50&&Lcd_Data[3]==0x83)
//				{
//					{
//						Receive_Lcd_Flag=1;
//						g_AdrID=(Lcd_Data[4]<<8)&0xFF00;
//						g_AdrID+=Lcd_Data[5]&0x00FF;
//						g_KeyID=(Lcd_Data[7]<<8)&0x0000FF00;
//						g_KeyID+=(Lcd_Data[8])&0x000000FF;						
//					} 
//				}
//			} 	
//		}
//		if(Receive_Lcd_Flag==1)		//�洢����������
//		{
//			for(i=0;i<LCD_Data_Num+1;i++)
//			{
//				 Lcd_Data[i]=0;
//			}
//			LCD_Data_Num=0;	
//		}
//	}
//	USART_ClearITPendingBit(USART1, USART_IT_RXNE);	
//}

/***********************************************************************
�������ƣ�USART3_Send_Data(unsigned char *send_buff,unsigned int length)
��    �ܣ�USART3_Send_Data����PLT�ļ���DSP
���������send_buff:�����͵�����ָ�룻length�����͵����ݳ��ȣ��ַ�������
���������
��дʱ�䣺2014.12.9
�� д �ˣ�
ע    �⣺
***********************************************************************/
void USART3_Send_Data(unsigned char *send_buff,unsigned int length)
{
 	unsigned int i = 0;
	USART_ClearFlag(USART3,USART_FLAG_TC);
	for(i = 0;i < length;i ++)
	{			
		USART3->DR = send_buff[i];
		while((USART3->SR&0X40)==0);	
	}	
}

/***********************************************************************
�������ƣ�USART2_Send_Data(unsigned char *send_buff,unsigned int length)
��    �ܣ�USART2_Send_Data�����ַ�����DGUS���Ĵ���
���������send_buff:�����͵�����ָ�룻length�����͵����ݳ��ȣ��ַ�������
���������
��дʱ�䣺2014.9.11
�� д �ˣ�
ע    �⣺
***********************************************************************/
void USART2_Send_Data(unsigned char *send_buff,unsigned int length)
{
 	unsigned int i = 0;
	USART_ClearFlag(USART2,USART_FLAG_TC);
	for(i = 0;i < length;i ++)
	{			
		USART2->DR = send_buff[i];
		while((USART2->SR&0X40)==0);	
	}	
}

/***********************************************************************
�������ƣ�USART1_Send_Data(unsigned char *send_buff,unsigned int length)
��    �ܣ�USART1�������ݸ�DSP�Ĵ���
������������ݴ洢�������ݳ���
���������
��дʱ�䣺2014.9.11
�� д �ˣ��Ҳ���
ע    �⣺
***********************************************************************/
void USART1_Send_Data(unsigned char *send_buff,unsigned int length)
{
 unsigned int i = 0;
	USART_ClearFlag(USART1,USART_FLAG_TC);
	for(i = 0;i < length;i ++)
	{			
		USART1->DR = send_buff[i];
		while((USART1->SR&0X40)==0);	
	}	
}

/***********************************************************************
�������ƣ�NVIC_Configuration
��    �ܣ��ж����ȼ�����
���������
���������
��дʱ�䣺2014.9.11
�� д �ˣ��Ҳ���
ע    �⣺
***********************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Set the Vector Table base location at 0x08000000 */
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
  
//  /* Enable the Ethernet global Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = ETH_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);   
//  
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
 void USART3_IRQHandler(void)  
{

}
