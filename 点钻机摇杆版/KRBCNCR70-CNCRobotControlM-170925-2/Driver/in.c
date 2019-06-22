/*************** (C) COPYRIGHT 2012 Kingrobot manipulator Team ****************
* File Name          : in.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This file provides all the GPIO input functions.
******************************************************************************/

#include "stm32f10x_lib.h"
#include "in.h"
#include "out.h"
#include "Delay.h"
#include "Auto.h"
#include "StatusControl.h"

/*------------�������ݼ��-------------*/
u8 Local_IO_Status[2]={0,0};
u8 Input_Detect_Status[6]={0,0,0,0,0,0};
u8 Input_Status_Change = FALSE;


/**************************************************************************************************
**  ��������  ReadEmergencyStop()
**	�����������
**	���������u8 (0/1)
**	�������ܣ���ȡ����˿��ź�	   Port:GPD12
**	��ע��	  
**  ���ߣ�    �ܺ���    
**  �������ڣ�2012/11/09
***************************************************************************************************/
u8 ReadEmergencyStop(void)
{
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8) == 0)		   //��ͣ����:PD8
	{
		Delay(10);
		if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8) == 0)
		{
			Delay(10);
			if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8) == 0)
			{		   
				return FALSE;	
			}
			else
			{
				return ReadEmergencyStop();
			}	
		}
		else
		{
			return ReadEmergencyStop();	
		}
	}
	else
	{		  
		return TRUE;	
	}		
}

/**************************************************************************************************
**  ��������  ReadAxsisError()
**	���������u8 
**	���������u8 (0/1)
**	�������ܣ���ȡ������ź�	   Port:GPE15
**	��ע��	  
**  ���ߣ�        
**  �������ڣ�2017/04/11
***************************************************************************************************/
u8 ReadAxsisError(u8 Error)
{
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15) == Error)		   //�ᱨ��:PE15  
	{
		Delay(10);
		if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15) == Error)
		{		   
			return TRUE;	
		}
		else
		{
			return ReadAxsisError(Error);
		}	
	}
	else
	{	  
		return FALSE;
	}		
}


/**************************************************************************************************
**  ��������  ReadXAxsisError()
**	�����������
**	���������u8 (0/1)
**	�������ܣ���ȡX������ź�	   Port:GPA5
**	��ע��	  
**  ���ߣ�        
**  �������ڣ�2012/11/09
***************************************************************************************************/
u8 ReadXAxsisError(void)
{
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15) == 1)		   //X�ᱨ��:PE15  �ߵ�ƽ��Ч
	{
		Delay(10);
		if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15) == 1)
		{		   
			return TRUE;	
		}
		else
		{
			return ReadXAxsisError();
		}	
	}
	else
	{	  
		return FALSE;
	}		
}




/**************************************************************************************************
**  ��������  ReadZAxsisError()
**	�����������
**	���������u8 (0/1)
**	�������ܣ���ȡZ������ź�	   Port:GPA2
**	��ע��	  
**  ���ߣ�    �ܺ���    
**  �������ڣ�2012/11/09
***************************************************************************************************/
u8 ReadZAxsisError(void)
{
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15) == 0)		   //X�ᱨ��:PE15
	{
		Delay(10);
		if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15) == 0)
		{		   
			return TRUE;	
		}
		else
		{
			return ReadZAxsisError();
		}	
	}
	else
	{		  
		return FALSE;
	}		
}


/**************************************************************************************************
**  ��������  ReadInput()
**	���������u8 IO_Num:��Ҫ��ȡ�Ķ˿�
**	���������u8 (0/1)
**	�������ܣ���ȡ����˿��ź�
**
**	��ע��	  �͵�ƽ��Ч������Ϊ�͵�ƽʱ����ӦλΪ1	 result=0
**
**  ���ߣ�    ����    
**  �������ڣ�2013/12/17 
***************************************************************************************************/
u8 ReadInput(u8 IO_Num)
{
	u8 result = FALSE;
	switch (IO_Num)
	{
		case 1://X6-PC3 								   //
			result = ReadIOPort(GPIOC,GPIO_Pin_3,(0x01&Input_Detect_Status[0]));
			if(result)
			{
				Input_Detect_Status[0] &= 0xfe;
			}
			else
			{
				Input_Detect_Status[0] |= 0x01;		     
			}
			break;

		case 2://X7-PA4 								   //
			result = ReadIOPort(GPIOA,GPIO_Pin_4,(0x02&Input_Detect_Status[0]));
			if(result)
			{
				Input_Detect_Status[0] &= 0xfd;
			}
			else
			{
				Input_Detect_Status[0] |= 0x02;			 
			}
			break;


		case 3://X8-PC4									  //
			result = ReadIOPort(GPIOC,GPIO_Pin_4,(0x04&Input_Detect_Status[1]));
			if(result)
			{
				Input_Detect_Status[0] &= 0xfb;
			}
			else
			{
				Input_Detect_Status[0] |= 0x04;		 
			}
			break;

		case 4://X9-PC5 								  //
			result = ReadIOPort(GPIOC,GPIO_Pin_5,(0x08&Input_Detect_Status[1]));
			if(result)
			{
				Input_Detect_Status[0] &= 0xf7;
			}
			else
			{
				Input_Detect_Status[0] |= 0x08;			 
			}
			break; 

		case 5://X10-PB0 								  //
			result = ReadIOPort(GPIOB,GPIO_Pin_0,(0x10&Input_Detect_Status[1]));
			if(result)
			{
				Input_Detect_Status[0] &= 0xef;
			}
			else
			{
				Input_Detect_Status[0] |= 0x10;			 
			}
			break; 

		case 6://X11-PB1 								  //
			result = ReadIOPort(GPIOB,GPIO_Pin_1,(0x20&Input_Detect_Status[1]));
			if(result)
			{
				Input_Detect_Status[0] &= 0xdf;
			}
			else
			{
				Input_Detect_Status[0] |= 0x20;			
			}
			break;

		case 7://X12-PF11 								//
			result = ReadIOPort(GPIOF,GPIO_Pin_11,(0x40&Input_Detect_Status[1]));
			if(result)
			{
				Input_Detect_Status[0] &= 0xbf;
			}
			else
			{
				Input_Detect_Status[0] |= 0x40;
			}
			break;

		case 8://X13-PF12 								//
			result = ReadIOPort(GPIOF,GPIO_Pin_12,(0x80&Input_Detect_Status[1]));
			if(result)
			{
				Input_Detect_Status[0] &= 0x7f;				
			}
			else
			{
				Input_Detect_Status[0] |= 0x80;
			}
			break;

		case 9://X14-PF13 							//
			result = ReadIOPort(GPIOF,GPIO_Pin_13,(0x01&Input_Detect_Status[1]));
			if(result)
			{
				Input_Detect_Status[1] &= 0xfe;
				Input_Count15=0;
			}
			else
			{
				Input_Detect_Status[1] |= 0x01;
			}
			break;

		case 10://X15-PF14							   //
			result = ReadIOPort(GPIOF,GPIO_Pin_14,(0x02&Input_Detect_Status[2]));
			if(result)
			{
				Input_Detect_Status[1] &= 0xfd;
				Input_Count16=0;
			}
			else
			{
				Input_Detect_Status[1] |= 0x02;
			}
			break;

		case 11://X16-PF15 							 //
			result = ReadIOPort(GPIOF,GPIO_Pin_15,(0x04&Input_Detect_Status[2]));
			if(result)
			{
				Input_Detect_Status[1] &= 0xfb;
			}
			else
			{
				Input_Detect_Status[1] |= 0x04;
			}
			break; 

		case 12://X17-PG0 							 //
			result = ReadIOPort(GPIOG,GPIO_Pin_0,(0x08&Input_Detect_Status[2]));
			if(result)
			{
				Input_Detect_Status[1] &= 0xf7;
			}
			else
			{
				Input_Detect_Status[1] |= 0x08;
			}
			break; 

		case 13://X18-PG1 							 //
			result = ReadIOPort(GPIOG,GPIO_Pin_1,(0x10&Input_Detect_Status[2]));
			if(result)
			{
				Input_Detect_Status[1] &= 0xef;
			}
			else
			{
				Input_Detect_Status[1] |= 0x10;
			}
			break;

		case 14://X19-PE7 							 //
			result = ReadIOPort(GPIOE,GPIO_Pin_7,(0x20&Input_Detect_Status[2]));
			if(result)
			{
				Input_Detect_Status[1] &= 0xdf;
			}
			else
			{
				Input_Detect_Status[1] |= 0x20;
			}
			break;

			case 15://X20-PE8							 //
			result = ReadIOPort(GPIOE,GPIO_Pin_8,(0x40&Input_Detect_Status[2]));
			if(result)
			{
				Input_Detect_Status[1] &= 0xbf;
			}
			else
			{
				Input_Detect_Status[1] |= 0x40;
			}
			break;	
			
			case 16://X21-PE9						  //B��ԭ��
			result = ReadIOPort(GPIOE,GPIO_Pin_9,(0x80&Input_Detect_Status[2]));
			if(result)
			{
				Input_Detect_Status[1] &= 0x7f;
			}
			else
			{
				Input_Detect_Status[1] |= 0x80;
			}
			break;

		case 17://X22-PE10 						   //A��ԭ��
			result = ReadIOPort(GPIOE,GPIO_Pin_10,(0x01&Input_Detect_Status[2]));
			if(result)
			{
				Input_Detect_Status[2] &= 0xfe;
			}
			else
			{
				Input_Detect_Status[2] |= 0x01;
			}
			break;

		case 18://X23-PE11			 		   //O��ԭ��
			result = ReadIOPort(GPIOE,GPIO_Pin_11,(0x02&Input_Detect_Status[3]));
			if(result)
			{
				Input_Detect_Status[2] &= 0xfd;
			}
			else
			{
				Input_Detect_Status[2] |= 0x02;
			}
			break;

		case 19://X24-PE12 				L��ԭ��
			result = ReadIOPort(GPIOE,GPIO_Pin_12,(0x04&Input_Detect_Status[3]));
			if(result)
			{
				Input_Detect_Status[2] &= 0xfb;
			}
			else
			{
				Input_Detect_Status[2] |= 0x04;
			}
			break;

		case 20://X25-PF3 	   X��ԭ��
			result = ReadIOPort(GPIOF,GPIO_Pin_3,(0x08&Input_Detect_Status[3]));
			if(result)
			{
				Input_Detect_Status[2] &= 0xf7;
			}
			else
			{
				Input_Detect_Status[2] |= 0x08;
			}
			break;

		case 21://X26-PF4		 Z��ԭ��
			result = ReadIOPort(GPIOF,GPIO_Pin_4,(0x10&Input_Detect_Status[3]));
			if(result)
			{
				Input_Detect_Status[2] &= 0xef;
			}
			else
			{
				Input_Detect_Status[2] |= 0x10;
			}
			break;
			
		default:
			break;			
	}
	return (result);
}

/**************************************************************************************************
**  ��������  ReadIOPort()
**	���������GPIO_TypeDef* GPIOx, u16 PortNum:�����Ϊ�˿ںű�־
			  u8 PortPreviousStatus���ö˿�֮ǰ��״̬	
**	���������u8 (0/1)
**	�������ܣ���ȡĳ��������ź�
**	��ע��	  
**  ���ߣ�    �ܺ���    
**  �������ڣ�2012/10/8
***************************************************************************************************/
u8 ReadIOPort(GPIO_TypeDef* GPIOx, u16 PortNum, u8 PortPreviousStatus)
{
	if(PortPreviousStatus == 0x00)
	{
		if(GPIO_ReadInputDataBit(GPIOx,PortNum) == 0)
		{
			DelayNus(200);
			if(GPIO_ReadInputDataBit(GPIOx,PortNum) == 0)
			{
				return 0;
			}//loop2
		}//loop1
		return 1;	
	}
	else
	{
		if(GPIO_ReadInputDataBit(GPIOx,PortNum) != 0)
		{
			DelayNus(200);
			if(GPIO_ReadInputDataBit(GPIOx,PortNum) != 0)
			{	
				return 1;
			}//loop2
		}//loop1
		return 0;	    
	}
}






/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/
