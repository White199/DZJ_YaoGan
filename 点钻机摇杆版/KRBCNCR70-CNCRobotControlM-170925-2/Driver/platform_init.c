/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : main.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "platform_init.h"
#include "Delay.h"
#include "Usart.h"

#define USART1_DR_Base  (USART1_BASE + 0x04)
#define USART3_DR_Base  (USART3_BASE + 0x04)


/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{  
    ErrorStatus HSEStartUpStatus;
    RCC_DeInit();                                /* RCC system reset(for debug purpose) */
    RCC_HSEConfig(RCC_HSE_ON);                   /* Enable HSE */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();  /* Wait till HSE is ready */
    if(HSEStartUpStatus == SUCCESS)
    {  
      FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);  /* Enable Prefetch Buffer */   
      FLASH_SetLatency(FLASH_Latency_2);                     /* Flash 2 wait state */     
      RCC_HCLKConfig(RCC_SYSCLK_Div1);                       /* HCLK = SYSCLK */     
      RCC_PCLK2Config(RCC_HCLK_Div1);                        /* PCLK2 = HCLK   72M*/   
      RCC_PCLK1Config(RCC_HCLK_Div2);                        /* PCLK1 = HCLK/2 36M*/   
      RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);   /* PLLCLK = 8MHz * 9 = 72 MHz */   
      RCC_PLLCmd(ENABLE);                                    /* Enable PLL */     
      while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)     /* Wait till PLL is ready */
      { 
      }    
      RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);             /* Select PLL as system clock source */    
      while(RCC_GetSYSCLKSource() != 0x08)                   /* Wait till PLL is used as system clock source:72M */
      {
      }
    }  
	/* Enable GPIOA, GPIOC,USART1 and AFIO clock */
  	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOG | RCC_APB2Periph_GPIOF | 
							RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOD | 
							RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | 
							RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 |
							RCC_APB2Periph_AFIO	 | RCC_APB2Periph_TIM1		 //72M
							, ENABLE );

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
                            RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 |		//36M
                            RCC_APB1Periph_TIM6 
                            , ENABLE ); 
                                
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE); //Timer2������ӳ��  TIM2_CH3->PB10                                
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);//��Դ������ʱ�ӿ���              
//---0801
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE );      /* CAN Periph clock enable */
}


/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructureOut;
    GPIO_InitTypeDef GPIO_InitStructureIn;
   	GPIO_InitTypeDef GPIO_InitStructure;


	/***************Usart�˿�����*****************/
	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
 	/* Configure USART1 Rx (PA.10) as input floating */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

 	//PF7-485DIR								
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOF,&GPIO_InitStructure);

//----CAN�˿�����
/* Configure CAN pin:PA11-RX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
  
    /* Configure CAN pin:PA12-TX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 

	/***************�ź�����˿�����*****************/
  	GPIO_InitStructureIn.GPIO_Mode = GPIO_Mode_IN_FLOATING;				 		//����Ϊ����
	//PA��	 PA4
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_4;
  	GPIO_Init(GPIOA,&GPIO_InitStructureIn);	  
	
	//PB��	 PB0.PB1.PB12.PB13
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_12|GPIO_Pin_13;
  	GPIO_Init(GPIOB,&GPIO_InitStructureIn);
	
	//PC��	 PC0.PC1.PC2.PC3.PC4.PC5.PC13
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_13;
  	GPIO_Init(GPIOC,&GPIO_InitStructureIn);

	//PE��	 PE2.PE3.PE4.PE5.PE6.PE7.PE8.PE9.PE10.PE11.PE12
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9	
	                               |GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
  	GPIO_Init(GPIOE,&GPIO_InitStructureIn);

	//PF��	 PIN_ALL
	GPIO_InitStructureIn.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_6|GPIO_Pin_8|GPIO_Pin_9	
	                               |GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  	GPIO_Init(GPIOF,&GPIO_InitStructureIn);

	//PG��   PG0.PG1
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	
  	GPIO_Init(GPIOG,&GPIO_InitStructureIn);

	/***************�ź�����˿�����*****************/
	GPIO_InitStructureOut.GPIO_Mode=GPIO_Mode_Out_PP;
  	GPIO_InitStructureOut.GPIO_Speed=GPIO_Speed_50MHz;
	//A��	PA8
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_8;
	GPIO_Init(GPIOA,&GPIO_InitStructureOut);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);

	//B��	PB5								
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_5;
	GPIO_Init(GPIOB,&GPIO_InitStructureOut);
	GPIO_SetBits(GPIOB,GPIO_Pin_5);

	//C��	PC8.PC9.PC10.PC11.PC12
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_Init(GPIOC,&GPIO_InitStructureOut);
	GPIO_SetBits(GPIOC,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12);
	
	//D��	PD0.PD1.PD2.PD3.PD4.PD5.PD6.PD7.PD9.PD10.PD11.PD12.PD13.PD14.PD15
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOD,&GPIO_InitStructureOut);
 	GPIO_SetBits(GPIOD,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);

	//E��	PE0.PE1
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Init(GPIOE,&GPIO_InitStructureOut);  
	GPIO_SetBits(GPIOE,GPIO_Pin_0|GPIO_Pin_1);

	//G��	PG2.PG3.PG4.PG5.PG6.PG7.PG8.PG9.PG10.PG11.PG12.PG13.PG14.PG15
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOG,&GPIO_InitStructureOut); 
	GPIO_SetBits(GPIOG,GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);   


	/*----GPIOA_5 ���ʹ�� ����˿�����----*/
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_5;
	GPIO_Init(GPIOA,&GPIO_InitStructureOut); 
//	
//   /*---- �����ź� ����˿�����----*/
//	GPIO_InitStructureIn.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_15 ;  //PE15:AXIS-tickle  ALARM
//  GPIO_Init(GPIOE, &GPIO_InitStructureIn);

//	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_8 ;   //PD8 :��ͣ����
//  GPIO_Init(GPIOD, &GPIO_InitStructureIn);


//   /*---- ��λ�����ź� ����˿�����----*/
//   /*- X36:PE9 X37:PE10 X38:PE11 X39:PE12 -*/
//   #ifdef HARDLIMITJUDGE_EXTI
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;  
//	  GPIO_Init(GPIOE, &GPIO_InitStructure);
//   #endif      

}

void Power_EXTI_Init()
{
	EXTI_InitTypeDef  EXTI_InitStructure; 
    EXTI_DeInit();
    EXTI_StructInit(&EXTI_InitStructure);  
    EXTI_InitStructure.EXTI_Line = EXTI_Line14; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);

 }


void Limit_EXTI_Init()
{
	EXTI_InitTypeDef  EXTI_InitStructure; 
    EXTI_DeInit();
    EXTI_StructInit(&EXTI_InitStructure);  
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource7);
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource8);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource9);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource10);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource11);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource12);
  /* 
   //PE7 X����
    EXTI_InitStructure.EXTI_Line = EXTI_Line7; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
	
   //PE8 X�Ҽ���
    EXTI_InitStructure.EXTI_Line = EXTI_Line8; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
*/	
  //PE9  O
    EXTI_InitStructure.EXTI_Line = EXTI_Line9; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
  	
  //PE10 X
    EXTI_InitStructure.EXTI_Line = EXTI_Line10; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
	
  //PE11 L
    EXTI_InitStructure.EXTI_Line = EXTI_Line11; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
	
  //PE12 Z
    EXTI_InitStructure.EXTI_Line = EXTI_Line12; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
  
}


//void PVD_Init(void)
//{
//	EXTI_InitTypeDef  EXTI_InitStructure; 
//    EXTI_DeInit();
//    EXTI_StructInit(&EXTI_InitStructure);  
//    EXTI_InitStructure.EXTI_Line = EXTI_Line16; 			// PVD���ӵ��ж���16��  
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//ʹ���ж�ģʽ  
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//��ѹ���ڷ�ֵʱ�����ж�  
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//ʹ���ж���  
//    EXTI_Init(&EXTI_InitStructure); 						//��ʼ 
//    PWR_PVDLevelConfig(PWR_PVDLevel_2V8); 					//�趨��ط�ֵ  
//    PWR_PVDCmd(ENABLE); 									//ʹ��PVD 
// }


void TimeInit(void)
{ 
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;	                       
  
 	/**************************************************************************
 	//     ��ʱ��6���ã� 720��Ƶ�����ϼ���������ֵΪ1000->10ms	  
 	***************************************************************************/

 	TIM_TimeBaseStructure.TIM_Period = 1000;
  	TIM_TimeBaseStructure.TIM_Prescaler = 719;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6,ENABLE);		  
}



/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures the NVIC and Vector Table base address.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    #ifdef  VECT_TAB_RAM  
	    /* Set the Vector Table base location at 0x20000000 */ 
      	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
    #else  /* VECT_TAB_FLASH  */
	    /* Set the Vector Table base location at 0x08000000 */ 
	    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
    #endif

    /* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

	/* Enable the TIM1_IRQ Interrupt */	     //L
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
		
	  /* Enable the TIM2_IRQ Interrupt */    //Z
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	   
	  
	  /* Enable the TIM3_IRQ Interrupt */    //X
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	  /* Enable the TIM4_IRQ Interrupt */	 //O
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	  /* Enable the TIM8_IRQ Interrupt */	 //V
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	  /* Enable the TIM5_IRQ Interrupt */	 //W
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	  /* Enable the TIM6_IRQ Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);				
	  
	  /* Enable the USART1_IRQ Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//	  /* Enable the PVD Interrupt */ //����PVD�ж�
//
//	NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQChannel;	
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
//	NVIC_Init(&NVIC_InitStructure);

//	  /* Enable the DMA1_Channel4_IRQ Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQChannel;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* ���ö˿�PB14Ϊ�ⲿ�жϣ����ڵ��籣�� */ 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//---CAN�ж�0802
/* Enable CAN RX0 interrupt IRQ channel */
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;        //��ռʽ   ע���ж����ȼ�������
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;               //��Ӧʽ
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);



}



	
void USART1_Configuration(void)
{
/* USART1 configuration -----------------------------------------------

-------*/
  /* USART1 configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
        - USART Clock disabled
        - USART CPOL: Clock is active low
        - USART CPHA: Data is captured on the middle 
        - USART LastBit: The clock pulse of the last data bit is not 

output to 
                         the SCLK pin			   */
 
  USART_ClockInitTypeDef  USART_ClockInitStructure;
	USART_InitTypeDef USART_InitStructure;
//	DMA_InitTypeDef   DMA_InitStructure;


//	DMA_DeInit(DMA1_Channel4);    
//    DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;  
//    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SendDataBuffer;  
//    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  
//    DMA_InitStructure.DMA_BufferSize = 0;//sizeof(g_SendBuffer);  
//    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
//    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
//    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
//    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
//    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
//    DMA_Init(DMA1_Channel4, &DMA_InitStructure); 

	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART1, &USART_ClockInitStructure);

	USART_InitStructure.USART_BaudRate            = 115200;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	/* Enable USART1 DMA Txrequest */  
//    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  
//    /* Enable DMA1 Channel4 */  
//    DMA_Cmd(DMA1_Channel4, ENABLE); 
}
//----2017.7.31
/****************CANģ���ʼ��**********************/

void CAN_Initialize()
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure; 
//   CAN_DeInit(); /* CAN register init */
  CAN_StructInit(&CAN_InitStructure); 

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM=DISABLE;           // ʱ�䴥��ͨѶģʽ
  CAN_InitStructure.CAN_ABOM=DISABLE;           // �Զ����߹���
  CAN_InitStructure.CAN_AWUM=DISABLE;           // �Զ�����ģʽ
  CAN_InitStructure.CAN_NART=DISABLE;           // ���Զ��ش���ģʽ��0-����ʧ��ʱ��һֱ�Զ��ش�֪���ɹ���1-����ֻ����һ��
  CAN_InitStructure.CAN_RFLM=DISABLE;           // ����FIFO����ģʽ��δ����-��һ�����ĻḲ��ԭ�б��ģ�����-��һ�����ı�����
  CAN_InitStructure.CAN_TXFP=DISABLE;           // ����FIFO���ȼ���  0-���ȼ��ɱ�ʾ��������1-�ɷ�������˳����������
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;        // ����ͬ����Ծ���1��ʱ�䵥λ
  CAN_InitStructure.CAN_BS1=CAN_BS1_3tq;        // ʱ���1Ϊ3��ʱ�䵥λ��1~16
  CAN_InitStructure.CAN_BS2=CAN_BS2_5tq;        // ʱ���2Ϊ 5��ʱ�䵥λ��1~8
  CAN_InitStructure.CAN_Prescaler=4;            // �趨һ��ʱ�䵥λ���ȣ����㲨����  1/(1+CAN_BS1_3tq+CAN_BS1_3tq)*tq
  //�ο�STM32CAN���������� CAN  ����ʱ�� 36M �����ʱ� 1M                                        //1Mb/s -- BRT = 1/(1+CAN_BS1_3tq+CAN_BS1_5tq)*tq =1Mb/s; tq=CAN_Prescaler*PCLK
   CAN_Init(&CAN_InitStructure);                // ����CAN_InitStruct��ָ���Ĳ�����ʼ������CAN�ļĴ���
  
  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber=0;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  CAN_ClearITPendingBit(CAN_IT_FMP0);
  CAN_ITConfig(CAN_IT_FMP0, ENABLE);/* CAN FIFO0 message pending interrupt enable */ 

}




/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/

