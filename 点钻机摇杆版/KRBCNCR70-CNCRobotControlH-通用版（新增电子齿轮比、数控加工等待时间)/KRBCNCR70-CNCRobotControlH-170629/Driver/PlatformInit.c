/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Usart.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "PlatformInit.h"
#include "usart.h"
#include "Parameter.h"
#include "key.h"
#include "DGUS.h"
 #include "stdlib.h"
 #include "delay.h"
//#define SRC_USART3_DR    (USART3_BASE + 0x04)        //���ڽ��ռĴ�����ΪԴͷ
#define SRC_USART1_DR    (USART1_BASE + 0x04)        //���ڽ��ռĴ�����ΪԴͷ
extern u8 Product_Num;
u16 Rod_Count=0;
u8 Rod_Speed_Rank=0;;//�жϵ�ǰ�ٶȵȼ�
u8 Axis_DIR=0;//��ǰ����
u8 Link_Step_Flag=0;//�綯�����л���־λ
u16 Rod_X_Positive=0, Rod_X_Negative=0,Rod_Y_Positive=0,Rod_Y_Negative=0,Rod_Centry_X=0,Rod_Centry_Y=0;
u16 Rod_Step_X[8]={0},Rod_Step_Y[8]={0};
//N_flag:1 ����ڷ�ת P_flag: 1�������ת 
/*---------------����༭����-------------------*/
extern u8 g_Background_Color_ProgramEdit ;			      //�༭����ɫ
extern u8 gs_Background_Color_ProgramEdit;
extern u8 gs_Current_Operate_List_ProgramEdit;				//��ѡ�������ɫ�������ڵĵ���

extern  u8 gs_LinkMove_Speed[6];	         //X�������ٶ�
extern u8 gs_XStepMove_Distance;        //X��綯����(��ʱ)
extern u8 gs_ZStepMove_Distance;        //Z��綯����
extern u8 gs_LStepMove_Distance;        //L��綯����
extern u8 gs_OStepMove_Distance;        //O��綯����
extern u8 gs_AStepMove_Distance;        //A��綯����
extern u8 gs_BStepMove_Distance;        //B��綯����

extern u8 Step_Period_Count;//�綯����

extern DZJSpeed DZJSpeed1;
u8 Speed_Rank[5]={0};
/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{ 
//  ErrorStatus HSEStartUpStatus;
//  RCC_DeInit();
//  RCC_HSEConfig(RCC_HSE_ON);
//  HSEStartUpStatus = RCC_WaitForHSEStartUp();
//  if(HSEStartUpStatus == SUCCESS)
//  {
//    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
//    FLASH_SetLatency(FLASH_Latency_2);
//    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
//    RCC_PCLK2Config(RCC_HCLK_Div1); 
//    RCC_PCLK1Config(RCC_HCLK_Div2);
//    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);	
//    RCC_PLLCmd(ENABLE);
//    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
//    {
//    }
//    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
//    while(RCC_GetSYSCLKSource() != 0x08)
//    {
//    }
//  }
   /*-----��ȡʱ����Ϣ-----*/
//   RCC_ClocksTypeDef RCC_ClockFreq;
//   RCC_GetClocksFreq(&RCC_ClockFreq);

//  //USBʱ������wx
//  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);


  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                          RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                          RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF |
						              RCC_APB2Periph_AFIO , ENABLE );

  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 , ENABLE );  
						   
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}


/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM  
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

   /* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		
 
  /* Enable the USART1_IRQ/USART3_IRQ Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	 /* Enable the TIM3_IRQ Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	 /* Enable the TIM4_IRQ Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* ���ö˿�PB0Ϊ�ⲿ�жϣ�ң�˰��� */ 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures IO
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

#if defined USE_USART2	&& defined USE_STM3210B_EVAL
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
#endif
  /*-------------------------UART1----PA9-PA10----------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//uart1���� TX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//uart1����	RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*-------------------------UART2----PA1-PA2-PA3-------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//485-DIR
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//485-TX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//485-RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

 /*-------------------------UART3------PB10-PB11--------------------*/
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;//uart3����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;//uart3����
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);



/*-------------------------GPIO---------------------------*/

  /**----�а���Ĥ���ų�ʼ��--����--**/
  //PB12~PB15	 ���
  //PC5 ~PC11    �ڲ�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15  ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 |GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  

  /**----ң�˰���(PB0)���ų�ʼ��--����--**/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  /**----�ײ�����(PB1)��Կ��(PA0 PC13)���ų�ʼ��--����--**/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /**----�������������(PC1 PC2 PC3)--����--**/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 |GPIO_Pin_3  ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7  ;	  //Ӳ�����Ŵ���PA6-PC3 PA7-PC2��Ҫ�޸�20161103
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

}
void Key_EXTI_Init(void)
{
		EXTI_InitTypeDef  EXTI_InitStructure; 
    EXTI_DeInit();
    EXTI_StructInit(&EXTI_InitStructure);  
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

   //PB0 ң�˰���
    EXTI_InitStructure.EXTI_Line = EXTI_Line0; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
}

/*************************************************************************
**  ��������  Adc_Init()
**	���������
**	�����������
**	�������ܣ�ADCC��ʼ��
**	��ע��	  ��
**  ���ߣ�    White_L   
**  �������ڣ�2018/7/20
**************************************************************************/
void Adc_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��

	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	/*----ң����·ADC�ɼ���PA4 PA5 ����������ߵ�һ��ң�� PC2 PC3���������ڲ���һ��ң�ˣ�---*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;//ADC12_IN4 ADC12_IN5 ͨ��4.5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;//ADC12_IN12  ADC12_IN13 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	ADC_DeInit(ADC1);  //��λADC1 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 4;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   
	
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_239Cycles5 );	
	//ʹ��ADC1��DMA  
  ADC_DMACmd(ADC1, ENABLE);  
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����
	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
 
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
}
/*************************************************************************
**  ��������  MYDMA_Config()
**	���������
**	�����������
**	�������ܣ�DMA��ʼ��
**	��ע��	  ��
**  ���ߣ�    White_L   
**  �������ڣ�2018/7/20
**************************************************************************/
vu16  AD_Value[10][4];   //�������ADCת�������Ҳ��DMA��Ŀ���ַ
vu16  After_filter[4];    //���������ƽ��ֵ֮��Ľ�� 
void MYDMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����
	
  DMA_DeInit(DMA1_Channel1);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ

	DMA_InitStructure.DMA_PeripheralBaseAddr =  (u32)&ADC1->DR;  //DMA�������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = 40;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //����������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
	  	
}
/*************************************************************************
**  ��������  Rod_Select_Order
**	���������
**	�����������
**	�������ܣ�ͨ��ң��ʵ�ַ�ҳѡָ��
**	��ע��	  ��
**  ���ߣ�    White_L   
**  �������ڣ�2018/11/02
**************************************************************************/

void Rod_Select_Order(u8 Page_Type)
{
	int  sum = 0,i=0,j=0,k=0,n=0;
	u8 count=0;
	 for(i=0;i<2;i++)         
	 {            
			for ( count=0;count<10;count++)            
			{   
				sum += AD_Value[count][i];             
			}            
			After_filter[i]=sum/10;            
			sum=0;       
	}
	 /**************************��Ʒ�༭ҳ��****************************/
	if(After_filter[0] > 2150)//ǰ������
	{
		if(Page_Type==2)
		{
		 if(gs_Current_Operate_List==g_Background_Color)	//�����ǰ����
		 {
			 if(gs_Current_Operate_List>1&&Rod_Count==0)//��������
			 {
					gs_Current_Operate_List--;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List>1&&Rod_Count>500)//��ⳤʱ��ң��ǰ��һֱ��������
			 {
					Rod_Count=1;
					gs_Current_Operate_List--;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;	
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
		 //����ǰ����
		 else
		 {
			 if(gs_Current_Operate_List>1&&Rod_Count==0)//��������
			 {
					gs_Current_Operate_List--;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List>1&&Rod_Count>500)//��ⳤʱ��ң��ǰ��һֱ��������
			 {
					Rod_Count=1;
					gs_Current_Operate_List--;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;		
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
	  }
		Rod_Count++;				
	}
	else if(After_filter[0] < 1950)//��������
	{
		if(Page_Type==2)//��Ʒ�༭ҳ��
		{
		 if(gs_Current_Operate_List==g_Background_Color)	//�����ǰ����
		 {
			 if(gs_Current_Operate_List<Product_Num&&Rod_Count==0)//��������
			 {
					gs_Current_Operate_List++;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List<Product_Num&&Rod_Count>500)//��ⳤʱ��ң��ǰ��һֱ��������
			 {
					Rod_Count=1;
					gs_Current_Operate_List++;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;	
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
		 //����ǰ����
		 else
		 {
			 if(gs_Current_Operate_List<Product_Num&&Rod_Count==0)//��������
			 {
					gs_Current_Operate_List++;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List<Product_Num&&Rod_Count>500)//��ⳤʱ��ң��ǰ��һֱ��������
			 {
					Rod_Count=1;
					gs_Current_Operate_List++;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;		
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
	  }
			Rod_Count++;				
	}
	else
	{
		Rod_Count=0;
	}
	 /********************************����༭ҳ��*****************************************/	
	if(After_filter[0] > 2150)//ǰ������
	{
		if(Page_Type==8)
		{
		 if(gs_Current_Operate_List_ProgramEdit==g_Background_Color_ProgramEdit)	//�����ǰ����
		 {
			 if(gs_Current_Operate_List_ProgramEdit>1&&Rod_Count==0)//��������
			 {
					gs_Current_Operate_List_ProgramEdit--;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List_ProgramEdit>1&&Rod_Count>500)//��ⳤʱ��ң��ǰ��һֱ��������
			 {
					Rod_Count=1;
					gs_Current_Operate_List_ProgramEdit--;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;	
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
		 //����ǰ����
		 else
		 {
			 if(gs_Current_Operate_List_ProgramEdit>1&&Rod_Count==0)//��������
			 {
					gs_Current_Operate_List_ProgramEdit--;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;
				 //��ȡǰ���LARGESTPROGRAMNUM������
				 if(gs_Current_Operate_List_ProgramEdit%LARGESTPROGRAMNUM==0&&Free_Program_Operate.Program[0].List>1)	
				 {
					  //����������
					  LCD_Current_Page = 66;
					  LCD_Pre_Page = LCD_Current_Page;
					  PageChange(LCD_Pre_Page);
						j=Free_Program_Operate.Program[0].List-1;
						for(i=j;i>j-LARGESTPROGRAMNUM;i--)
						{
							AT24CXX_Read(CURRENT_PROGRAM_ADR2+(i-1)*ORDER_LENTH,&IIC_Temp[3],ORDER_LENTH);
							/*List Value1 2 3 4  5 LC_ID Key IO_Group*/
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Flag  = 1;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].LC_ID	= IIC_Temp[25];	
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Key   = IIC_Temp[26];
							 if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 �㽺 2���� 3ȡ�� 4 ��ʱ 5����
							 {
								Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
							 }
							else if(4==IIC_Temp[26])//��ʱָ��
							{
								Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
							}							
						}
						gs_Current_Operate_List_ProgramEdit=j;
						LCD_Current_Page=8;
				 }
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List_ProgramEdit>1&&Rod_Count>500)//��ⳤʱ��ң��ǰ��һֱ��������
			 {
					Rod_Count=1;
					gs_Current_Operate_List_ProgramEdit--;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;		
				 //��ȡǰ���LARGESTPROGRAMNUM������
				 if(gs_Current_Operate_List_ProgramEdit%LARGESTPROGRAMNUM==0&&Free_Program_Operate.Program[0].List>1)	
				 {
						j=Free_Program_Operate.Program[0].List-1;
						for(i=j;i>j-LARGESTPROGRAMNUM;i--)
						{
							AT24CXX_Read(CURRENT_PROGRAM_ADR2+(i-1)*ORDER_LENTH,&IIC_Temp[3],ORDER_LENTH);
							/*List Value1 2 3 4  5 LC_ID Key IO_Group*/
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Flag  = 1;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].LC_ID	= IIC_Temp[25];	
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Key   = IIC_Temp[26];
							 if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 �㽺 2���� 3ȡ�� 4 ��ʱ 5����
							 {
								Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
							 }
							else if(4==IIC_Temp[26])//��ʱָ��
							{
								Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
							}							
						}
						LCD_Current_Page=8;						
				 }
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
	  }
		Rod_Count++;				
	}
	else if(After_filter[0] < 1950)//��������
	{
		if(Page_Type==8)//��Ʒ�༭ҳ��
		{
		 if(gs_Current_Operate_List_ProgramEdit==g_Background_Color_ProgramEdit)	//�����ǰ����
		 {
			 if(gs_Current_Operate_List_ProgramEdit<Free_Program_Operate.Num&&Rod_Count==0)//��������
			 {
					gs_Current_Operate_List_ProgramEdit++;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;

					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List_ProgramEdit<Free_Program_Operate.Num&&Rod_Count>500)//��ⳤʱ��ң��ǰ��һֱ��������
			 {
					Rod_Count=1;
					gs_Current_Operate_List_ProgramEdit++;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;	
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
		 //����ǰ����
		 else
		 {
			 if(gs_Current_Operate_List_ProgramEdit<Free_Program_Operate.Num&&Rod_Count==0)//��������
			 {
					gs_Current_Operate_List_ProgramEdit++;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;
				 if((gs_Current_Operate_List_ProgramEdit-1)%LARGESTPROGRAMNUM==0&&Free_Program_Operate.Num>=gs_Current_Operate_List_ProgramEdit)
				 {
					j=Free_Program_Operate.Program[LARGESTPROGRAMNUM-1].List;
					if(Free_Program_Operate.Num-j>LARGESTPROGRAMNUM)
						n=j+LARGESTPROGRAMNUM;
					else
						n=Free_Program_Operate.Num;
					for(i=j;i<n;i++)
					{
						AT24CXX_Read(CURRENT_PROGRAM_ADR2+i*ORDER_LENTH,&IIC_Temp[3],ORDER_LENTH);
					/*List Value1 2 3 4  5 LC_ID Key IO_Group*/
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Flag  = 1;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].LC_ID	= IIC_Temp[25];	
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Key   = IIC_Temp[26];
						 if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 �㽺 2���� 3ȡ�� 4 ��ʱ 5����
						 {
							Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
						 }
						else if(4==IIC_Temp[26])//��ʱָ��
						{
							Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
						}										
					}
					if(i%LARGESTPROGRAMNUM!=0)
					{
						for(k=i%LARGESTPROGRAMNUM;k<LARGESTPROGRAMNUM;k++)
						{
						/*List key  value1 2 3 4 5*/
							Free_Program_Operate.Program[k].Flag  = 0;
							Free_Program_Operate.Program[k].List  = 0;
							Free_Program_Operate.Program[k].Key   = 0;
							Free_Program_Operate.Program[k].Value1= 0;
							Free_Program_Operate.Program[k].Value2= 0;
							Free_Program_Operate.Program[k].Value3= 0;
							Free_Program_Operate.Program[k].Value4= 0;
							Free_Program_Operate.Program[k].Value5= 0;										
						}
					}
						LCD_Current_Page=8;
				}
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List_ProgramEdit<Free_Program_Operate.Num&&Rod_Count>500)//��ⳤʱ��ң��ǰ��һֱ��������
			 {
					Rod_Count=1;
					gs_Current_Operate_List_ProgramEdit++;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;		
				 if((gs_Current_Operate_List_ProgramEdit-1)%LARGESTPROGRAMNUM==0&&Free_Program_Operate.Num>=gs_Current_Operate_List_ProgramEdit)
				 {
					j=Free_Program_Operate.Program[LARGESTPROGRAMNUM-1].List;
					if(Free_Program_Operate.Num-j>LARGESTPROGRAMNUM)
						n=j+LARGESTPROGRAMNUM;
					else
						n=Free_Program_Operate.Num;
					for(i=j;i<n;i++)
					{
						AT24CXX_Read(CURRENT_PROGRAM_ADR2+i*ORDER_LENTH,&IIC_Temp[3],ORDER_LENTH);
						/*List Value1 2 3 4  5 LC_ID Key IO_Group*/
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Flag  = 1;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].LC_ID	= IIC_Temp[25];	
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Key   = IIC_Temp[26];
						 if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 �㽺 2���� 3ȡ�� 4 ��ʱ 5����
						 {
							Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
						 }
						else if(4==IIC_Temp[26])//��ʱָ��
						{
							Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
						}									
					}
					if(i%LARGESTPROGRAMNUM!=0)
					{
						for(k=i%LARGESTPROGRAMNUM;k<LARGESTPROGRAMNUM;k++)
						{
						/*List key  value1 2 3 4 5*/
							Free_Program_Operate.Program[k].Flag  = 0;
							Free_Program_Operate.Program[k].List  = 0;
							Free_Program_Operate.Program[k].Key   = 0;
							Free_Program_Operate.Program[k].Value1= 0;
							Free_Program_Operate.Program[k].Value2= 0;
							Free_Program_Operate.Program[k].Value3= 0;
							Free_Program_Operate.Program[k].Value4= 0;
							Free_Program_Operate.Program[k].Value5= 0;										
						}
					}
						LCD_Current_Page=8;
				}
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
	  }
			Rod_Count++;				
	}
	else
	{
		Rod_Count=0;
	}	
}
/*************************************************************************
**  ��������  Rod_Axis_Contrl
**	���������
**	�����������
**	�������ܣ�ң���˶�����
**	��ע��	  ��
**  ���ߣ�    White_L   
**  �������ڣ�2018/12/02
**************************************************************************/
void Rod_Axis_Contrl(u8 Rod_Speed_Rank,u8 Axis_DIR,u8 Axis)
{
	if(Axis_DIR)
	{
		  if(Link_Step_Flag==0)//����ģʽ
		  {
			if(g_Axis_Is_Run==0||(g_Axis_Is_Run!=0&&Axsis_Move_Direction[Axis]==1&&gs_LinkMove_Speed[Axis]!=Rod_Speed_Rank))
			{
				 Axis_Position_Scan=TRUE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				gs_LinkMove_Speed[Axis] = Rod_Speed_Rank;
				Order_Temp[0]=P_ACTION_SPEEDADJ;            //ң����Ǳ���ٶȱ��
				Order_Temp[1]=gs_LinkMove_Speed[Axis];            //
				Order_Temp[2]=Axis;            //		
				Order_Temp[3]=1;            			//	����								
				USART2_SendDataArray(4,MANUL_OPERATE,&Order_Temp[0]);//���ڷ��ͻ�е���Զ������е����������ذ�
			}
			else if(g_Axis_Is_Run!=0&&Axsis_Move_Direction[Axis]==0)
			{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				switch(Axis)
				{
					case X_Axsis:
						Order_Temp[0] =XAXIS_MOVE_LEFT;        //X��綯����
					break;
					case L_Axsis:
						Order_Temp[0] =LAXIS_MOVE_LEFT;        //X��綯����
					break;	
					case Z_Axsis:
						Order_Temp[0] =ZAXIS_MOVE_LEFT;        //X��綯����
					break;
					case O_Axsis:
						Order_Temp[0] =OAXIS_MOVE_LEFT;        //X��綯����
					break;
					case A_Axsis:
						Order_Temp[0] =AAXIS_MOVE_LEFT;        //X��綯����
					break;
					case B_Axsis:
						Order_Temp[0] =BAXIS_MOVE_LEFT;        //X��綯����
					break;
				}
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X��ֹͣ								
			}
		}
		else//�����綯ģʽ
		{
//			if(Step_Period_Count>=5)//ÿ��50ms�綯һ��
//			{
//				Step_Period_Count=0;
//				switch(Axis)
//				{
//					case X_Axsis:
//						Order_Temp[0] =XAXIS_MOVE_RIGHT;        //X��綯����
//					break;
//					case L_Axsis:
//						Order_Temp[0] =LAXIS_MOVE_RIGHT;        //X��綯����
//					break;	
//					case Z_Axsis:
//						Order_Temp[0] =ZAXIS_MOVE_RIGHT;        //X��綯����
//					break;
//					case O_Axsis:
//						Order_Temp[0] =OAXIS_MOVE_RIGHT;        //X��綯����
//					break;
//					case A_Axsis:
//						Order_Temp[0] =AAXIS_MOVE_RIGHT;        //X��綯����
//					break;
//					case B_Axsis:
//						Order_Temp[0] =BAXIS_MOVE_RIGHT;        //X��綯����
//					break;
//				}
//			 	 Order_Temp[1] = JOG_MOVE_MODE;	            //�綯ģʽ
//				 Order_Temp[2] = 1;	
//				 USART1_SendData(3,MANUL_OPERATE,&Order_Temp[0]);//ͨ�ţ���λ��ǰ-��λ�ں�
//				 Axis_Position_Scan=TRUE;                   //�Զ�����ʱ����λ�ò�ѯ
//			}				
		}			
	}
	else
	{
		  if(Link_Step_Flag==0)//����ģʽ
		  {
			if(g_Axis_Is_Run==0||(g_Axis_Is_Run!=0&&Axsis_Move_Direction[Axis]==0&&gs_LinkMove_Speed[Axis]!=Rod_Speed_Rank))
			{
				 Axis_Position_Scan=TRUE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				gs_LinkMove_Speed[Axis] = Rod_Speed_Rank;
				Order_Temp[0]=P_ACTION_SPEEDADJ;            //ң����Ǳ���ٶȱ��
				Order_Temp[1]=gs_LinkMove_Speed[Axis];            //
				Order_Temp[2]=Axis;            //		
				Order_Temp[3]=0;            //	����								
				USART2_SendDataArray(4,MANUL_OPERATE,&Order_Temp[0]);//���ڷ��ͻ�е���Զ������е����������ذ�
			}		
			else if(g_Axis_Is_Run!=0&&Axsis_Move_Direction[Axis]==1)
			{
			 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
			switch(Axis)
			{
				case X_Axsis:
					Order_Temp[0] =XAXIS_MOVE_RIGHT;        //X��綯����
				break;
				case L_Axsis:
					Order_Temp[0] =LAXIS_MOVE_RIGHT;        //X��綯����
				break;	
				case Z_Axsis:
					Order_Temp[0] =ZAXIS_MOVE_RIGHT;        //X��綯����
				break;
				case O_Axsis:
					Order_Temp[0] =OAXIS_MOVE_RIGHT;        //X��綯����
				break;
				case A_Axsis:
					Order_Temp[0] =AAXIS_MOVE_RIGHT;        //X��綯����
				break;
				case B_Axsis:
					Order_Temp[0] =BAXIS_MOVE_RIGHT;        //X��綯����
				break;
			}
			 Order_Temp[1] = LINK_MOVE_MODE;	                   
			 Order_Temp[2] = 0;
			 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X��ֹͣ								
			}
		 }	
		else//�����綯ģʽ
		{
//			if(Step_Period_Count>=5)//ÿ��50ms�綯һ��
//			{
//				Step_Period_Count=0;
//				switch(Axis)
//				{
//					case X_Axsis:
//						Order_Temp[0] =XAXIS_MOVE_LEFT;        //X��綯����
//					break;
//					case L_Axsis:
//						Order_Temp[0] =LAXIS_MOVE_LEFT;        //X��綯����
//					break;	
//					case Z_Axsis:
//						Order_Temp[0] =ZAXIS_MOVE_LEFT;        //X��綯����
//					break;
//					case O_Axsis:
//						Order_Temp[0] =OAXIS_MOVE_LEFT;        //X��綯����
//					break;
//					case A_Axsis:
//						Order_Temp[0] =AAXIS_MOVE_LEFT;        //X��綯����
//					break;
//					case B_Axsis:
//						Order_Temp[0] =BAXIS_MOVE_LEFT;        //X��綯����
//					break;
//				}
//			 	 Order_Temp[1] = JOG_MOVE_MODE;	            //�綯ģʽ
//				 Order_Temp[2] = 1;	
//				 USART1_SendData(3,MANUL_OPERATE,&Order_Temp[0]);//ͨ�ţ���λ��ǰ-��λ�ں�
//				 Axis_Position_Scan=TRUE;                   //�Զ�����ʱ����λ�ò�ѯ
//			}			
		}				
	}
}
/*************************************************************************
**  ��������  Rob_Move
**	���������
**	�����������
**	�������ܣ���ADC��ֵ�����Ͷ�Ӧ���˶�ָ�ָֹͣ�����Ϸ��ͣ��˶�ָ����MANULOPERATE����
**	��ע��	  ��
**  ���ߣ�    DH   
**  �������ڣ�2018/8/02
**************************************************************************/
//ң�˶�Ӧ���˶���ʽ��800  2100-2900:�綯�ƶ���  
//												 			�綯--->�綯  �ȵ��ͣ���ٽ��д綯
//												 			����--->�綯	�ȵ��ͣ���ٽ��д綯
//										600  2900-3500:�����ٶ�V1 �ﵽ�ٶ�V1�Ժ������˶��׶Σ���ʼ����ң���ٶȸı�	
//															����--->����	���ٺ�ʼ����
//															�綯--->����	���ֹͣ���������
//										500  3500-4000:�����ٶ�V2 �ﵽ�ٶ�V2�Ժ������˶��׶Σ���ʼ����ң���ٶȸı�
//										100  4000-4096:�����ٶ�V3 �ﵽ�ٶ�V3�Ժ������˶��׶Σ���ʼ����ң���ٶȸı�

void Rob_Move(void)
{    
	int  sum = 0,i=0;
	 u8  count;       
	 for(i=0;i<2;i++)         
	 {            
			for ( count=0;count<10;count++)            
			{   
				sum += AD_Value[count][i];             
			}            
			After_filter[i]=sum/10;            
			sum=0;       
	 }
	 if(((After_filter[0]) >= Rod_Step_X[4]) || ((After_filter[0]) <= Rod_Step_X[3]) || ((After_filter[1]) >= Rod_Step_Y[4]) || ((After_filter[1]) <= Rod_Step_Y[3]))
	 {
/**************************ң�˵�һ���������XZA�ᶯ��************************************/
		if((abs(After_filter[0]-Rod_Centry_X))>= (abs(After_filter[1]-Rod_Centry_Y)))
		{	
			if((After_filter[0] >= Rod_Step_X[4]&&After_filter[0]<Rod_Step_X[5]))
			{
				Rod_Speed_Rank=5;
				Axis_DIR=1;
			}
			if(After_filter[0] <= Rod_Step_X[3]&&After_filter[0]>Rod_Step_X[2])
			{
				Rod_Speed_Rank=5;
				Axis_DIR=0;
			}
		  if(After_filter[0] >= Rod_Step_X[5]&&After_filter[0] < Rod_Step_X[6])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]>>2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]>>2;				
				Axis_DIR=1;			
			}
			if(After_filter[0] <= Rod_Step_X[2]&&After_filter[0]>Rod_Step_X[1])
			{
				Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]>>2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]>>2;
				Axis_DIR=0;					
			}
			if(After_filter[0] >= Rod_Step_X[6]&&After_filter[0] < Rod_Step_X[7])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=1;			
			}
			if(After_filter[0] <= Rod_Step_X[1]&&After_filter[0]>Rod_Step_X[0])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=0;					
			}
			if(After_filter[0] >= Rod_Step_X[7])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]<<1;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<1;
				Axis_DIR=1;					
			}
			if(After_filter[0] <= Rod_Step_X[0])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]<<1;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<1;
				Axis_DIR=0;					
			}
/**********************************----ң���ٶȿ���V1 2 3 4-----****************************************/
		switch(LCD_Current_Page)
			{
				case 148://X
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,X_Axsis);
					break;
				case 149://Z
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,Z_Axsis);
					break;
				case 150://A
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,A_Axsis);
					break;
				case 0x04://X
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,X_Axsis);
					break;
				case 0x09://Z
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,Z_Axsis);
					break;
				case 0x0A://A
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,A_Axsis);
					break;
				default:
					break;
			}
		}
/**************************ң�˵�һ���������YOB�ᶯ��************************************/
		else
		{
			if((After_filter[1] >= Rod_Step_Y[4]&&After_filter[1]<Rod_Step_Y[5]))
			{
				Rod_Speed_Rank=5;
				Axis_DIR=1;
			}
			if(After_filter[1] <= Rod_Step_Y[3]&&After_filter[1]>Rod_Step_Y[2])
			{
				Rod_Speed_Rank=5;
				Axis_DIR=0;
			}
		  if(After_filter[1] >= Rod_Step_Y[5]&&After_filter[1] < Rod_Step_Y[6])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]>>2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]>>2;
				Axis_DIR=1;			
			}
			if(After_filter[1] <= Rod_Step_Y[2]&&After_filter[1]>Rod_Step_Y[1])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]>>2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]>>2;
				Axis_DIR=0;					
			}
			if(After_filter[1] >= Rod_Step_Y[6]&&After_filter[1] < Rod_Step_Y[7])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=1;			
			}
			if(After_filter[1] <= Rod_Step_Y[1]&&After_filter[1]>Rod_Step_Y[0])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=0;					
			}
			if(After_filter[1] >= Rod_Step_Y[7])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]<<1;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<1;
				Axis_DIR=1;					
			}
			if(After_filter[1] <= Rod_Step_Y[0])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]<<1;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<1;
				Axis_DIR=0;					
			}
/**********************************----ң���ٶȿ���V1 2 3 4-----****************************************/
		switch(LCD_Current_Page)
			{
				case 148://Y
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,L_Axsis);
					break;
				case 149://O
//						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,O_Axsis);
					break;
				case 150://B
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,B_Axsis);
					break;
				case 0x04://Y
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,L_Axsis);
					break;
				case 0x09://O
//						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,O_Axsis);
					break;
				case 0x0A://B
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,B_Axsis);
					break;
				default:
					break;
			}			
		}
		LCD_RealTime_Watch=TRUE;
		Axis_Position_Scan=TRUE;                   //�Զ�����ʱ����λ�ò�ѯ
	}
	else
	{
		switch(LCD_Current_Page)
		{
			case 148://X
				if(Axsis_Move_Direction[X_Axsis]==1&&(g_Axis_Is_Run&0x01)==0x01)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = XAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X��ֹͣ					
				}
				if(Axsis_Move_Direction[X_Axsis]==0&&(g_Axis_Is_Run&0x01)==0x01)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = XAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X��ֹͣ					
				}
				DelayNms(20);
				if(Axsis_Move_Direction[L_Axsis]==1&&(g_Axis_Is_Run&0x04)==0x04)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = LAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Y��ֹͣ	
				}
				if(Axsis_Move_Direction[L_Axsis]==0&&(g_Axis_Is_Run&0x04)==0x04)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = LAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Y��ֹͣ					
				}
				break;
			case 149://O
				if(Axsis_Move_Direction[O_Axsis]==1&&(g_Axis_Is_Run&0x08)==0x08)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = OAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //0��ֹͣ					
				}
				if(Axsis_Move_Direction[O_Axsis]==0&&(g_Axis_Is_Run&0x08)==0x08)
				{			
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = OAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //0��ֹͣ						
				}		
				DelayNms(20);
				if(Axsis_Move_Direction[Z_Axsis]==1&&(g_Axis_Is_Run&0x02)==0x02)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = ZAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Z��ֹͣ						
				}
				if(Axsis_Move_Direction[Z_Axsis]==0&&(g_Axis_Is_Run&0x02)==0x02)
				{	
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = ZAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Z��ֹͣ						
				}
				break;
			case 150:
				if(Axsis_Move_Direction[A_Axsis]==1&&(g_Axis_Is_Run&0x10)==0x10)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = AAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //V��ֹͣ		
				}
				if(Axsis_Move_Direction[A_Axsis]==0&&(g_Axis_Is_Run&0x10)==0x10)
				{	
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = AAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //V��ֹͣ						
				}
				DelayNms(20);
				if(Axsis_Move_Direction[B_Axsis]==1&&(g_Axis_Is_Run&0x20)==0x20)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = BAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //W��ֹͣ	
				}	
				if(Axsis_Move_Direction[B_Axsis]==0&&(g_Axis_Is_Run&0x20)==0x20)
				{	
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = BAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //W��ֹͣ
				}							
				break;
			case 0x04://X
				if(Axsis_Move_Direction[X_Axsis]==1&&(g_Axis_Is_Run&0x01)==0x01)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = XAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X��ֹͣ					
				}
				if(Axsis_Move_Direction[X_Axsis]==0&&(g_Axis_Is_Run&0x01)==0x01)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = XAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X��ֹͣ					
				}
				DelayNms(20);
				if(Axsis_Move_Direction[L_Axsis]==1&&(g_Axis_Is_Run&0x04)==0x04)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = LAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Y��ֹͣ	
				}
				if(Axsis_Move_Direction[L_Axsis]==0&&(g_Axis_Is_Run&0x04)==0x04)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = LAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Y��ֹͣ					
				}
				break;
			case 0x09://O
				if(Axsis_Move_Direction[O_Axsis]==1&&(g_Axis_Is_Run&0x08)==0x08)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = OAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //0��ֹͣ					
				}
				if(Axsis_Move_Direction[O_Axsis]==0&&(g_Axis_Is_Run&0x08)==0x08)
				{			
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = OAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //0��ֹͣ						
				}		
				DelayNms(20);
				if(Axsis_Move_Direction[Z_Axsis]==1&&(g_Axis_Is_Run&0x02)==0x02)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = ZAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Z��ֹͣ						
				}
				if(Axsis_Move_Direction[Z_Axsis]==0&&(g_Axis_Is_Run&0x02)==0x02)
				{	
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = ZAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Z��ֹͣ						
				}
				break;
			case 0x0A:
				if(Axsis_Move_Direction[A_Axsis]==1&&(g_Axis_Is_Run&0x10)==0x10)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = AAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //V��ֹͣ		
				}
				if(Axsis_Move_Direction[A_Axsis]==0&&(g_Axis_Is_Run&0x10)==0x10)
				{	
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = AAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //V��ֹͣ						
				}
				DelayNms(20);
				if(Axsis_Move_Direction[B_Axsis]==1&&(g_Axis_Is_Run&0x20)==0x20)
				{
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = BAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //W��ֹͣ	
				}	
				if(Axsis_Move_Direction[B_Axsis]==0&&(g_Axis_Is_Run&0x20)==0x20)
				{	
				 Axis_Position_Scan=FALSE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = BAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //W��ֹͣ
				}							
				break;
		}		
	}

}
/*********************************************************************************************************
** �������� ��Rod_Adjust()  
** �������� ��ң��У׼
** ��ڲ��� ��
** ���ڲ��� ����
** ����			��White_L
********************************************************************************************************/
int Rod_Display_X=0,Rod_Display_Y=0;
void Rod_Adjust(void)
{
	int  sum = 0,i=0;
	 u8  count;       
	 for(i=0;i<2;i++)         
	 {            
			for ( count=0;count<10;count++)            
			{   
				sum += AD_Value[count][i];             
			}            
			After_filter[i]=sum/10;            
			sum=0;       
	 }	
	 if(((After_filter[0]) >= Rod_Step_X[4]) || ((After_filter[0]) <= Rod_Step_X[3]) || ((After_filter[1]) >= Rod_Step_Y[4]) || ((After_filter[1]) <= Rod_Step_Y[3]))
	 {
		if((abs(After_filter[0]-Rod_Centry_X))>= (abs(After_filter[1]-Rod_Centry_Y)))
		{
			if(After_filter[0] <= Rod_Step_X[7])//������
			{
				Rod_Display_X = ((After_filter[0]-Rod_Centry_X)/20);
			}
			if(After_filter[0] >= Rod_Step_X[7])
			{
				Rod_Display_X=100;
			}
			if(After_filter[0] <= Rod_Step_X[3]&&After_filter[0] >= Rod_Step_X[0])//������
			{
				Rod_Display_X = -(int)((Rod_Centry_X-After_filter[0])/20);
			}
			if(After_filter[0] <= Rod_Step_X[0])
			{
				Rod_Display_X=-100;
			}			
		}
		else
		{
			if(After_filter[1] <= Rod_Step_Y[7])//������
			{
				Rod_Display_Y = ((After_filter[1]-Rod_Centry_X)/20);
			}
			if(After_filter[1] >= Rod_Step_Y[7])
			{
				Rod_Display_Y=100;
			}
			if(After_filter[1] <= Rod_Step_Y[3]&&After_filter[1] >= Rod_Step_Y[0])//������
			{
				Rod_Display_Y = -(int)((Rod_Centry_X-After_filter[1])/20);
			}	
			if(After_filter[1] <= Rod_Step_Y[0])
			{
				Rod_Display_Y=-100;
			}			
		}
	 }
	 else//��ʾ0
	 {
		 Rod_Display_X=0;
		 Rod_Display_Y=0;
	 }
}
/*************************************************************************
**  ��������  Rob_Link_Move
**	���������
**	�����������
**	�������ܣ���ADC��ֵ�����Ͷ�Ӧ���˶�ָ�ָֹͣ�����Ϸ��ͣ��˶�ָ����MANULOPERATE����
**	��ע��	  ��
**  ���ߣ�    DH   
**  �������ڣ�2018/8/02
**************************************************************************/
extern u8 Order_Temp[];
extern u8 AXIS_Choose;
void Rob_Link_Move(void)
{ 
	int  sum = 0,i=0;
	 u8  count;       
	 for(i=0;i<2;i++)         
	 {            
			for ( count=0;count<10;count++)            
			{   
				sum += AD_Value[count][i];             
			}            
			After_filter[i]=sum/10;            
			sum=0;       
	 }
	 if(((After_filter[0]) >= Rod_Step_X[4]) || ((After_filter[0]) <= Rod_Step_X[3]) || ((After_filter[1]) >= Rod_Step_Y[4]) || ((After_filter[1]) <= Rod_Step_Y[3]))
	 {
/**************************ң�˵�һ���������XZA�ᶯ��************************************/
		if((abs(After_filter[0]-Rod_Centry_X))>= (abs(After_filter[1]-Rod_Centry_Y)))
		{	
			if((After_filter[0] >= Rod_Step_X[4]&&After_filter[0]<Rod_Step_X[5]))
			{
				Rod_Speed_Rank=1;
				Axis_DIR=1;
			}
			if(After_filter[0] <= Rod_Step_X[3]&&After_filter[0]>Rod_Step_X[2])
			{
				Rod_Speed_Rank=1;
				Axis_DIR=0;
			}
		  if(After_filter[0] >= Rod_Step_X[5]&&After_filter[0] < Rod_Step_X[6])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];				
				Axis_DIR=1;			
			}
			if(After_filter[0] <= Rod_Step_X[2]&&After_filter[0]>Rod_Step_X[1])
			{
				Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=0;					
			}
			if(After_filter[0] >= Rod_Step_X[6]&&After_filter[0] < Rod_Step_X[7])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]<<1;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<1;
				Axis_DIR=1;			
			}
			if(After_filter[0] <= Rod_Step_X[1]&&After_filter[0]>Rod_Step_X[0])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]<<1;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<1;
				Axis_DIR=0;					
			}
			if(After_filter[0] >= Rod_Step_X[7])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]<<2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<2;
				Axis_DIR=1;					
			}
			if(After_filter[0] <= Rod_Step_X[0])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]<<2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<2;
				Axis_DIR=0;					
			}
/**********************************----ң���ٶȿ���V1 2 3 4-----****************************************/
		switch(LCD_Current_Page)
			{
				case 4://X
						if(Axis_DIR==0&&g_XAxis_Position==0)
						{}
						else
						{
							Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,X_Axsis);
						}
					break;
				case 9://Z
						if(Axis_DIR==0&&g_ZAxis_Position==0)
						{}
						else
						{
							Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,Z_Axsis);
						}
					break;
				case 10://V
						if(Axis_DIR==0&&g_AAxis_Position==0)
						{}
						else
						{
							Rod_Axis_Contrl(1,Axis_DIR,A_Axsis);
						}
					break;
				default:
					break;
			}
		}
/**************************ң�˵�һ���������YOB�ᶯ��************************************/
		else
		{
			if((After_filter[1] >= Rod_Step_Y[4]&&After_filter[1]<Rod_Step_Y[5]))
			{
				Rod_Speed_Rank=1;
				Axis_DIR=1;
			}
			if(After_filter[1] <= Rod_Step_Y[3]&&After_filter[1]>Rod_Step_Y[2])
			{
				Rod_Speed_Rank=1;
				Axis_DIR=0;
			}
		  if(After_filter[1] >= Rod_Step_Y[5]&&After_filter[1] < Rod_Step_Y[6])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]/4;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]/4;
				Axis_DIR=1;			
			}
			if(After_filter[1] <= Rod_Step_Y[2]&&After_filter[1]>Rod_Step_Y[1])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]/4;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]/4;
				Axis_DIR=0;					
			}
			if(After_filter[1] >= Rod_Step_Y[6]&&After_filter[1] < Rod_Step_Y[7])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]/2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]/2;
				Axis_DIR=1;			
			}
			if(After_filter[1] <= Rod_Step_Y[1]&&After_filter[1]>Rod_Step_Y[0])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]/2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]/2;
				Axis_DIR=0;					
			}
			if(After_filter[1] >= Rod_Step_Y[7])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=1;					
			}
			if(After_filter[1] <= Rod_Step_Y[0])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=0;					
			}
/**********************************----ң���ٶȿ���V1 2 3 4-----****************************************/
		switch(LCD_Current_Page)
			{
				case 4://Y
						if(Axis_DIR==0&&g_LAxis_Position==0)
						{}
						else
						{
							Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,L_Axsis);
						}
					break;
				case 9://O
						if(Axis_DIR==0&&g_OAxis_Position==0)
						{}
						else
						{
							Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,O_Axsis);
						}
					break;
				case 10://W
						if(Axis_DIR==0&&g_BAxis_Position==0)
						{}
						else
						{
							Rod_Axis_Contrl(1,Axis_DIR,B_Axsis);
						}
					break;
			}			
		}
		LCD_RealTime_Watch=TRUE;
		Axis_Position_Scan=TRUE;                   //�Զ�����ʱ����λ�ò�ѯ
	}
	else
	{
		switch(LCD_Current_Page)
		{
			case 4://X
				if(Axsis_Move_Direction[X_Axsis]==1&&(g_Axis_Is_Run&0x01)==0x01)
				{
				 Order_Temp[0] = XAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X��ֹͣ					
				}
				if(Axsis_Move_Direction[X_Axsis]==0&&(g_Axis_Is_Run&0x01)==0x01)
				{
				 Order_Temp[0] = XAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X��ֹͣ					
				}
				DelayNms(100);
				if(Axsis_Move_Direction[L_Axsis]==1&&(g_Axis_Is_Run&0x04)==0x04)
				{
				 Order_Temp[0] = LAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Y��ֹͣ	
				}
				if(Axsis_Move_Direction[L_Axsis]==0&&(g_Axis_Is_Run&0x04)==0x04)
				{
				 Order_Temp[0] = LAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Y��ֹͣ					
				}
				break;
			case 9://O
				if(Axsis_Move_Direction[O_Axsis]==1&&(g_Axis_Is_Run&0x08)==0x08)
				{
				 Order_Temp[0] = OAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //0��ֹͣ					
				}
				if(Axsis_Move_Direction[O_Axsis]==0&&(g_Axis_Is_Run&0x08)==0x08)
				{			
				 Order_Temp[0] = OAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //0��ֹͣ						
				}		
				if(Axsis_Move_Direction[Z_Axsis]==1&&(g_Axis_Is_Run&0x02)==0x02)
				{
				 Order_Temp[0] = ZAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Z��ֹͣ						
				}
				if(Axsis_Move_Direction[Z_Axsis]==0&&(g_Axis_Is_Run&0x02)==0x02)
				{	
				 Order_Temp[0] = ZAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Z��ֹͣ						
				}
				break;
			case 10:
				if(Axsis_Move_Direction[A_Axsis]==1&&(g_Axis_Is_Run&0x10)==0x10)
				{
				 Order_Temp[0] = AAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //V��ֹͣ		
				}
				if(Axsis_Move_Direction[A_Axsis]==0&&(g_Axis_Is_Run&0x10)==0x10)
				{	
				 Order_Temp[0] = AAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //V��ֹͣ						
				}
				if(Axsis_Move_Direction[B_Axsis]==1&&(g_Axis_Is_Run&0x20)==0x20)
				{
				 Order_Temp[0] = BAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //W��ֹͣ	
				}	
				if(Axsis_Move_Direction[B_Axsis]==0&&(g_Axis_Is_Run&0x20)==0x20)
				{	
				 Order_Temp[0] = BAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //W��ֹͣ
				}							
				break;
		}		
	}

}
/*******************************************************************************
* Function Name  : USART1_DMA_Configuration
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/	
void USART1_DMA_Configuration(void)
{
    USART_ClockInitTypeDef  USART_ClockInitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef   DMA_InitStructure;


	DMA_DeInit(DMA1_Channel4);    
    DMA_InitStructure.DMA_PeripheralBaseAddr = SRC_USART1_DR;  
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART1SendBuff;  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  
    DMA_InitStructure.DMA_BufferSize = 0;//sizeof(g_SendBuffer);  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
    DMA_Init(DMA1_Channel4, &DMA_InitStructure); 

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
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  
    /* Enable DMA1 Channel4 */  
    DMA_Cmd(DMA1_Channel4, ENABLE); 
}

/*******************************************************************************
* Function Name  : USART1_Robot_Configuration
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_Robot_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART1, &USART_ClockInitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	USART_ClearITPendingBit(USART1, USART_IT_TC);

}

/*******************************************************************************
* Function Name  : USART2_485_Configuration
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_485_Configuration()
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART2, &USART_ClockInitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
	USART_ClearITPendingBit(USART2, USART_IT_TC);
}
/*******************************************************************************
* Function Name  : USART3_LCD_Configuration
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART3_LCD_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART3, &USART_ClockInitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
	USART_ClearITPendingBit(USART3, USART_IT_TC);

}



void TimeInit(void)
{ 
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;
	TIM_ICInitTypeDef TIM_ICInitStructure;                       
 	/**************************************************************************
 	//     ��ʱ��4���ã� 720��Ƶ�����ϼ���������ֵΪ1000->10ms	  
 	***************************************************************************/
 	TIM_TimeBaseStructure.TIM_Period = 1000;
  	TIM_TimeBaseStructure.TIM_Prescaler = 719;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

 	/**************************************************************************
 	//     ��ʱ��3���ã� ��������������	  	
 	***************************************************************************/
 	TIM_TimeBaseStructure.TIM_Period = 0xffff;
  	TIM_TimeBaseStructure.TIM_Prescaler = 0;
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);   //�����ز�׽
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0x06;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1 | TIM_Channel_2;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	TIM_ClearFlag(TIM3, TIM_FLAG_Update); 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
//	TIM3->CNT =0;	      //�����ʱ��
//	TIM_Cmd(TIM3, ENABLE);//ʹ�ܶ�ʱ��  
//	TIM_GetCounter(TIM3); //��ȡ������
}

void RTC_Configuration()
{
	  /* Enable PWR and BKP clocks */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	  
	  /* Allow access to BKP Domain */
	  PWR_BackupAccessCmd(ENABLE);
	
	  /* Reset Backup Domain */
	  BKP_DeInit();
	  /* Enable LSE */
	  RCC_LSEConfig(RCC_LSE_ON);
	  /* Wait till LSE is ready */
	  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	  {
	  }
	
	  /* Select LSE as RTC Clock Source */
	  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);  
	  /* Enable RTC Clock */
	  RCC_RTCCLKCmd(ENABLE);
	
	
	#ifdef RTCClockOutput_Enable  
	  /* Disable the Tamper Pin */
	  BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper
	                               functionality must be disabled */
	                               
	  /* Enable RTC Clock Output on Tamper Pin */
	  BKP_RTCCalibrationClockOutputCmd(ENABLE);
	#endif 
	
	  /* Wait for RTC registers synchronization */
	  RTC_WaitForSynchro();
	
	  /* Wait until last write operation on RTC registers has finished */
	  RTC_WaitForLastTask();
	  
	  /* Enable the RTC Second */  
	 // RTC_ITConfig(RTC_IT_SEC, ENABLE);
	
	  /* Wait until last write operation on RTC registers has finished */
	  RTC_WaitForLastTask();
	  RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
	  
	  /* Wait until last write operation on RTC registers has finished */
	  RTC_WaitForLastTask();
}

/*******************************************************************************
* Function Name  : Time_Adjust
* Description    : Adjusts time.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Time_Adjust(void)
{
	  /* Wait until last write operation on RTC registers has finished */
	  RTC_WaitForLastTask(); 
	  /* Change the current time */
	  RTC_SetCounter(0x00000000);
	  /* Wait until last write operation on RTC registers has finished */
	  RTC_WaitForLastTask();   
}


void RTC_Init()
{
	u32 temp=0;
	temp=BKP_ReadBackupRegister(BKP_DR1);
	  if( temp!= 0xA5A5)
	  {
	  	Check_battery=1;			  //��ر��ε����ı�־λ(ÿ�ν������ж�Ϊû�е��)
	    RTC_Configuration();
	    /* Adjust time by values entred by the user on the hyperterminal */
	    Time_Adjust();
	    BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);    
	  }
	  else
	  {
	    
	    /* Wait for RTC registers synchronization */
	    RTC_WaitForSynchro();
	    /* Wait until last write operation on RTC registers has finished */
	    RTC_WaitForLastTask();
	  }
}

