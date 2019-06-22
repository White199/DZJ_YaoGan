//#include <string.h>	
#include "ff.h"

#include "usbh_usr.h"
#include "usbh_msc_core.h"
#include "usbh_msc_scsi.h"
#include "usbh_msc_bot.h"


#include "DGUS.h"
#include "Delay.h"

uint8_t USBH_USR_ApplicationState = USH_USR_FS_INIT;
uint8_t temp[]                        = "          \n";
u8 Robot_Startup=1;
TCHAR  File_Buffer[11]={0};  //���ڴ�Ŷ�ȡ��U���е������ļ�������
UINT   File_Num=0;			     //�ļ������ļ���
UINT   FileName_Len=0;		   //�ļ������ļ�������
u8  USB_Button_Ok=FALSE;	   //ȷ����ť
u8  USB_Button_Cancle=FALSE; //ȡ����ť
u8  Test_USB_Flag=FALSE;	   //USB���ܲ��Ա�ʶ

u8 Firmware_Name_Table[9][12]={0};

UINT br;
FATFS fs;         /* Work area (file system object) for logical drive */
FIL fsrc;         /* file objects */
FRESULT res;
extern uint8_t Line;



/*  Points to the DEVICE_PROP structure of current device */
/*  The purpose of this register is to speed up the execution */

USBH_Usr_cb_TypeDef USR_Callbacks =
{
  USBH_USR_Init,
  USBH_USR_DeviceAttached,
  USBH_USR_ResetDevice,
  USBH_USR_DeviceDisconnected,
  USBH_USR_OverCurrentDetected,
  USBH_USR_DeviceSpeedDetected,
  USBH_USR_Device_DescAvailable,
  USBH_USR_DeviceAddressAssigned,
  USBH_USR_Configuration_DescAvailable,
  USBH_USR_Manufacturer_String,
  USBH_USR_Product_String,
  USBH_USR_SerialNum_String,
  USBH_USR_EnumerationDone,
  USBH_USR_UserInput,
  USBH_USR_DeviceNotSupported,
  USBH_USR_UnrecoveredError
};


/**
U�̶�д˵��
*/
void USBH_USR_Init(void)
{
}

/**
��⵽U�̲���
*/
void USBH_USR_DeviceAttached(void)
{
//	UCBDeviceConectStatus=1;	   										  //USB�豸���ӱ�־λ
		USBH_Copy_Status = USBHSTATUS_UDISKATTACHED;
		USB_Update_Status = USBSTATUS_UDISKATTACHED;
		DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);
		DisplayChineseString(0x5930,USB_UpdateStatus_String[USB_Update_Status],24);
}


/**
U�̲��ɻָ�����
*/
void USBH_USR_UnrecoveredError (void)
{
		Test_USB_Flag=TRUE;   //wx
		USBH_Copy_Status = USBHSTATUS_UDISKERROR;
	  USB_Update_Status = USBSTATUS_UDISKERROR;
		DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);
		DisplayChineseString(0x5930,USB_UpdateStatus_String[USB_Update_Status],24);
		USB_Button_Cancle=TRUE;	
}


/**
��⵽U�̰ε�
*/
void USBH_USR_DeviceDisconnected (void)
{ 

		USBH_Copy_Status = USBHSTATUS_UDISKPULLOUT;
	  USB_Update_Status = USBSTATUS_UDISKPULLOUT;
		DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);
		DisplayChineseString(0x5930,USB_UpdateStatus_String[USB_Update_Status],24);
		if(Robot_Startup)
		{
			USBH_Copy_Status = USBHSTATUS_NOUDISK;
			USB_Update_Status = USBSTATUS_NOUDISK;
			DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);
			DisplayChineseString(0x5930,USB_UpdateStatus_String[USB_Update_Status],24);
			Robot_Startup=0;	
		}
		USB_Button_Cancle=TRUE;

}
/**
��λU���豸
*/
void USBH_USR_ResetDevice(void)
{
  /* callback for USB-Reset */
}
/**
���U���豸�ٶ�
*/
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
{

}

/**
* @brief  USBH_USR_Device_DescAvailable 
*         Displays the message on LCD for device descriptor
* @param  device descriptor
* @retval None
*/
void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{
	
}

/**
* @brief  USBH_USR_DeviceAddressAssigned 
*         USB device is successfully assigned the Address 
* @param  None
* @retval None
*/
void USBH_USR_DeviceAddressAssigned(void)
{
  
}
/**
U���豸˵��
*/
void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc)
{
}

/**
U�̳���˵��
*/
void USBH_USR_Manufacturer_String(void *ManufacturerString)
{ 
	
}

/**
��Ʒ��Ϣ
*/
void USBH_USR_Product_String(void *ProductString)
{
	
}

/**
U�̴��ں���Ϣ
*/
void USBH_USR_SerialNum_String(void *SerialNumString)
{
	
} 

/**
ö�����
*/
void USBH_USR_EnumerationDone(void)
{
	
} 
/**
		Ӳ����֧��
*/
void USBH_USR_DeviceNotSupported(void)
{
  
  Test_USB_Flag=TRUE;   //wx 
  if(MSC_Machine.maxLun > 0)
  {

//	printf("> Contains more than one LUN! \r\n ");  

    /*Display the LUN in the connected device*/
    USBH_USR_HexToASCII((uint32_t)(MSC_Machine.maxLun)); 
  }
  
    /* 2 seconds delay */
		USB_OTG_BSP_mDelay(2000);

		USBH_Copy_Status = USBHSTATUS_UDISKERROR;
	  USB_Update_Status = USBSTATUS_UDISKERROR;
		DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);
		DisplayChineseString(0x5930,USB_UpdateStatus_String[USB_Update_Status],24);
		USB_Button_Cancle=TRUE;
}  


/**
* @brief  USBH_USR_UserInput
*         User Action for application state entry
* @param  None
* @retval USBH_USR_Status : User response for key button
*/
USBH_USR_Status USBH_USR_UserInput(void)
{
//  USBH_USR_Status usbh_usr_status;
		return USBH_USR_RESP_OK;
}  

/**
���������
*/
void USBH_USR_OverCurrentDetected (void)
{
		Test_USB_Flag=TRUE;   //wx
		/* 2 seconds delay */
		USB_OTG_BSP_mDelay(2000);

		USBH_Copy_Status = USBHSTATUS_UDISKERROR;
	  USB_Update_Status = USBSTATUS_UDISKERROR;
		DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);
		DisplayChineseString(0x5930,USB_UpdateStatus_String[USB_Update_Status],24);
		USB_Button_Cancle=TRUE;
}

/**
  * @brief  USBH_USR_MSC_Application 
  *         Demo application for mass storage
  * @param  None
  * @retval Staus
  */
int USBH_USR_MSC_Application(void)
{
  int i=0,j=0;
  u8 USB_temp[2]={0};
  FILINFO fno;
  DIR dir;
	u8 RES;
  //u8 Read_Write_Buff[1507]={0};
  u8 Read_Write_Buff[16]={0};
 /**********״̬��******************/
  switch(USBH_USR_ApplicationState)
  {
  	case USH_USR_FS_INIT:  //�ļ���ʼ������ȡu�̴�С
    
			//��⵽U��
			if((USBH_Copy_Status != USBHSTATUS_UDISKATTACHED)||(USB_Update_Status != USBSTATUS_UDISKATTACHED))
			{			 
				USBH_Copy_Status = USBHSTATUS_UDISKATTACHED;
				USB_Update_Status = USBSTATUS_UDISKATTACHED;
				DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);
				DisplayChineseString(0x5930,USB_UpdateStatus_String[USB_Update_Status],24);
			}

	    /* Initialises the file system */
	    RES=f_mount(0,&fs);			  
		  if( RES == FR_OK)
			{
				/*Display the size of the disk*/
				USBH_USR_HexToASCII((((USBH_MSC_Param.MSCapacity) * \
													 (USBH_MSC_Param.MSPageLength))/(1048576)));	 //���U�̿ռ��С

				//д�ļ�
				if(USB_Update_Flag == 0)
				{
					if(USBH_Direction==CONTROLERTOUSB)   
					{
						if(USBH_MSC_Param.MSWriteProtect==DISK_WRITE_PROTECTED) //���U��д����
						{
						//����������U�̱�����ʾ��DGUS��
						 USBH_Copy_Status = USBHSTATUS_UDISKWIRTEPRO;
						 DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);	
						 USBH_USR_ApplicationState = USH_USR_FS_NULL; 						
						}
						else
						{
						 USBH_USR_ApplicationState = USH_USR_FS_WRITEFILE;
						}
					}				
					//���ļ�
					else if(USBH_Direction==USBTOCONTROLER)   
					{
						USBH_USR_ApplicationState = USH_USR_FS_READLIST;//USH_USR_FS_READFILE;
					}
				} 
				//����
				else
				{
					USBH_USR_ApplicationState = USB_USR_FS_READLIST;
				}
				USB_Button_Ok=FALSE;
				USB_Button_Cancle=FALSE;	
	    }
      else
      {
				
			}				
	    break;
    
  	case USH_USR_FS_READLIST:	//��ȡu�̵ĸ�Ŀ¼�ļ�����
    
			/*Reads the dir list*/
			res = f_opendir(&dir, "0:");
			if( res != FR_OK )
			{
				return(-2);
			}    
	    
	    /**************��u�̸�Ŀ¼����ʾ��DGUS���϶�Ӧ���ı���ʾ��***************/
			while((HCD_IsDeviceConnected(&USB_OTG_FS_dev)) && ( f_readdir(&dir, &fno) == FR_OK ) && fno.fname[0] != 0 )
			{       
				//��ѯ��Program.txt�ļ�   fno.fname[0]=0x00 ??
				if((fno.fname[1]=='R')&&(fno.fname[2]=='O')&&(fno.fname[3]=='G'))
				{
					for(i=0;i<11;i++)
					{
						File_Buffer[i]=fno.fname[i];		 //��¼�ļ���
					}
					File_Buffer[0]='P';
					File_Num=1;
					break;
				}
			}
			if(File_Num)  //�����ļ��������ļ���ȡ����
			{
				USBH_USR_ApplicationState = USH_USR_FS_READFILE;
			}
			else
			{
			 //δ�ҵ������ļ�
				USBH_Copy_Status = USBHSTATUS_UDISKNOFILES;
				DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);			
				USBH_USR_ApplicationState = USH_USR_FS_NULL; 	 
			}
			break;    

  case USH_USR_FS_WRITEFILE:		//��U�̴����ļ���д������   
	    /* Writes a text file, U������.TXT in the disk*/	    
	    if(USBH_MSC_Param.MSWriteProtect == DISK_WRITE_PROTECTED)//���u��д����
	    {	    
				//д����
				USBH_Copy_Status = USBHSTATUS_UDISKWIRTEPRO;
				DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);
				USBH_USR_ApplicationState = USH_USR_FS_NULL;
	    }	
			else
			{    
			  //��ʽ��U��
//			  USBH_Copy_Status = USBHSTATUS_UDISKFAMMAT;  //U�̸�ʽ��...
//		    DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);
//			  f_mkfs(0,0,0);
				
			  //�����ļ�-�����ļ�
			  USBH_Copy_Status = USBHSTATUS_ISCOPING;	  //���򿽱���
		    DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);
			  res = f_open( &fsrc , "0:/PROGRAM.TXT" , FA_CREATE_ALWAYS | FA_WRITE);	//������Program.TXT���ļ�	U�����ļ���PROGRAM.TXT	

		    /* д���� Free_Program_Save[SAVEPROGRAMNUM]*/
			  /* SAVEPROGRAMNUM=10;�ϼƱ���10������      */
			  /* ÿ��������Դ�100��ָ��,�ϼ�1507�ֽ�     */
			  for(i=3;i<SAVEPROGRAMNUM+3;i++)
			  {
					if(USART1_ReadProgram(i)==0) //���������ȡ�ɹ� key>2
					{
						Read_Write_Buff[0]=Free_Program_Operate.Flag;
						Read_Write_Buff[1]=Free_Program_Operate.Code;
						Read_Write_Buff[2]=Free_Program_Operate.Name>>24;
						Read_Write_Buff[3]=Free_Program_Operate.Name>>16;
						Read_Write_Buff[4]=Free_Program_Operate.Name>>8;
						Read_Write_Buff[5]=Free_Program_Operate.Name;
						Read_Write_Buff[6]=Free_Program_Operate.Num;	
						res = f_write(&fsrc, Read_Write_Buff, 7, &br); 	
						for(j=0;j<LARGESTPROGRAMNUM;j++)
						{
							Read_Write_Buff[0] =Free_Program_Operate.Program[j].Flag;
							Read_Write_Buff[1] =Free_Program_Operate.Program[j].List;
							Read_Write_Buff[2] =Free_Program_Operate.Program[j].Order;
							Read_Write_Buff[3] =Free_Program_Operate.Program[j].Key;
							Read_Write_Buff[4] =Free_Program_Operate.Program[j].Value1>>24;
							Read_Write_Buff[5] =Free_Program_Operate.Program[j].Value1>>16;
							Read_Write_Buff[6] =Free_Program_Operate.Program[j].Value1>>8;
							Read_Write_Buff[7] =Free_Program_Operate.Program[j].Value1;
							Read_Write_Buff[8] =Free_Program_Operate.Program[j].Value2>>24;
							Read_Write_Buff[9] =Free_Program_Operate.Program[j].Value2>>16;
							Read_Write_Buff[10]=Free_Program_Operate.Program[j].Value2>>8;
							Read_Write_Buff[11]=Free_Program_Operate.Program[j].Value2;
							Read_Write_Buff[12]=Free_Program_Operate.Program[j].Value3>>24;
							Read_Write_Buff[13]=Free_Program_Operate.Program[j].Value3>>16;
							Read_Write_Buff[14]=Free_Program_Operate.Program[j].Value3>>8;
							Read_Write_Buff[15]=Free_Program_Operate.Program[j].Value3;	
							res = f_write(&fsrc, Read_Write_Buff, 16, &br); 				
						}
					}
					else
					{					    
						break;
					}	  			  
			  }    
				/*close file */
				f_close(&fsrc);   
			  //�����ļ����
			  if(i==SAVEPROGRAMNUM+3)
			  {
					if(USART1_ReadProgram(0x02)==0)  //�������֮��ָ���������
				  {
					  USBH_Copy_Status = USBHSTATUS_FINISHEDCOPY;	  //���򿽱����
					  i=0;
				  }
				  else
				  {
						USBH_Copy_Status = USBHSTATUS_FAILEDCOPY;	  //���򿽱�ʧ��
					  i=0;
				  }
				}
			  else
			  {
					USBH_Copy_Status = USBHSTATUS_FAILEDCOPY;	  //���򿽱�ʧ��
				  i=0;
			  }
			  DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);			  
			  //���д�ļ� ���������״̬
			  USBH_USR_ApplicationState = USH_USR_FS_NULL;   
			}
	    break;

  	case USH_USR_FS_READFILE:	  //��ȡ�ļ�
				res = f_open(&fsrc, "0:/PROGRAM.TXT", FA_OPEN_EXISTING | FA_READ);
				if( res == FR_OK )
				{
					USBH_Copy_Status = USBHSTATUS_ISCOPING;	  //���򿽱���
					DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);

					USB_Send_Program=TRUE;
					//USB��������ʼ
					USB_temp[0] =PROGRAM_FROM_USB_START;
					USART1_SendData(1,FREE_PROGRAM_SEND,USB_temp);     

					for(i=0;i<SAVEPROGRAMNUM;i++)
					{		 	 					
						res = f_read( &fsrc, Read_Write_Buff,7, &br );
						Free_Program_Operate.Flag = Read_Write_Buff[0];
						Free_Program_Operate.Code = Read_Write_Buff[1];
						Free_Program_Operate.Name = (u32)( (((u32)Read_Write_Buff[2])<<24)|(((u32)Read_Write_Buff[3])<<16)|(((u32)Read_Write_Buff[4])<<8)|(((u32)Read_Write_Buff[5])) );
						Free_Program_Operate.Num  = Read_Write_Buff[6];
						for(j=0;j<LARGESTPROGRAMNUM;j++)
						{
							res = f_read( &fsrc, Read_Write_Buff,16, &br );
							Free_Program_Operate.Program[j].Flag  = Read_Write_Buff[0];
							Free_Program_Operate.Program[j].List  = Read_Write_Buff[1];
							Free_Program_Operate.Program[j].Order = Read_Write_Buff[2];
							Free_Program_Operate.Program[j].Key   = Read_Write_Buff[3];;
							Free_Program_Operate.Program[j].Value1= (u32)( (((u32)Read_Write_Buff[4])<<24)|(((u32)Read_Write_Buff[5])<<16)|(((u32)Read_Write_Buff[6])<<8)|(((u32)Read_Write_Buff[7])) );
							Free_Program_Operate.Program[j].Value2= (u32)( (((u32)Read_Write_Buff[8])<<24)|(((u32)Read_Write_Buff[9])<<16)|(((u32)Read_Write_Buff[10])<<8)|(((u32)Read_Write_Buff[11])) );
							Free_Program_Operate.Program[j].Value3= (u32)( (((u32)Read_Write_Buff[12])<<24)|(((u32)Read_Write_Buff[13])<<16)|(((u32)Read_Write_Buff[14])<<8)|(((u32)Read_Write_Buff[15])) );										
						}
						USART1_SendProgram(Free_Program_Operate.Num,&Free_Program_Operate);
						DelayNms(10);
						Free_Program_Save[Free_Program_Operate.Code-1].Flag=Free_Program_Operate.Flag;
						Free_Program_Save[Free_Program_Operate.Code-1].Code=Free_Program_Operate.Code;
						Free_Program_Save[Free_Program_Operate.Code-1].Name=Free_Program_Operate.Name;
						Free_Program_Save[Free_Program_Operate.Code-1].Num =Free_Program_Operate.Num;					
					}              
					//USB�����������
					USB_temp[0] =PROGRAM_FROM_USB_END;
					USART1_SendData(1,FREE_PROGRAM_SEND,USB_temp);     

					g_Run_Program_Num = 0;
					USB_Send_Program=FALSE;
					f_close(&fsrc);  /*close file */
					//�����ļ����
					USBH_Copy_Status = USBHSTATUS_FINISHEDCOPY;	  //���򿽱����
					DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);	
				}
				USBH_USR_ApplicationState = USH_USR_FS_NULL;
				break;
		
		case USB_USR_FS_READLIST:
			/*Reads the dir list*/
			USB_Button_Ok = TRUE;
			File_Num = 0;
			if(LCD_Current_Page==145)//���ȷ����
			{
				res = f_opendir(&dir, "0:");
				if( res != FR_OK )
				{
					return(-2);
				}
			/**************��u�̸�Ŀ¼����ʾ��DGUS���϶�Ӧ���ı���ʾ��***************/
				//while((HCD_IsDeviceConnected(&USB_OTG_FS_dev)) && ( f_readdir(&dir, &fno) == FR_OK ) && fno.fname[0] != 0 )
				while(1)
				{
					res = f_readdir(&dir, &fno);//��ȡĿ¼�µ�һ���ļ�
					if(res != FR_OK || fno.fname[0] == 0)
					{
						USBH_USR_ApplicationState=USH_USR_FS_NULL;
						break;  //������/��ĩβ��,�˳�
					}
					//��ѯ��Program.txt�ļ�   fno.fname[0]=0x00
					else if((fno.fname[1]=='_')&&(fno.fname[2]=='V')&&(fno.fname[4]=='_')&&(fno.fname[6]=='.')&&(fno.fname[7]=='B')&&(fno.fname[8]=='I')&&(fno.fname[9]=='N'))
					{
						if(USBH_Direction == Handset)//�ֳ����̼�����ʾ
						{
							if(fno.fname[0]=='H')
							{
								for(i=0;fno.fname[i] != 0;i++)
								{
									File_Buffer[i]=fno.fname[i];		 //��¼�ļ���
									Firmware_Name_Table[File_Num][i] = fno.fname[i];
								}
								File_Num++;
								DisplayChineseString(0x5960+12*(File_Num-1),(u8 *)&File_Buffer[0],12);
								File_Buffer[10]=0;
								//break;
							}
						}
						else if(USBH_Direction == Controller)
						{
							if(fno.fname[0]=='M')
							{
								for(i=0;fno.fname[i] != 0;i++)
								{
									File_Buffer[i]=fno.fname[i];		 //��¼�ļ���
									Firmware_Name_Table[File_Num][i] = fno.fname[i];
								}
								File_Num++;
								DisplayChineseString(0x5960+12*(File_Num-1),(u8 *)&File_Buffer[0],12);
								File_Buffer[10]=0;
							}
						}
					}
				}
				if(File_Num)  //�����ļ��������ļ���ȡ����
				{
				}
				else
				{
				 //δ�ҵ������ļ�
					USB_Update_Status = USBSTATUS_UDISKNOFILES;
					DisplayChineseString(0x5930,USB_UpdateStatus_String[USB_Update_Status],24);				
					USBH_USR_ApplicationState = USH_USR_FS_NULL; 	 
				}
			}
			break;
		 
  	case USH_USR_FS_NULL:	  //�����״̬
	     USBH_USR_ApplicationState = USH_USR_FS_NULL;
			 if(USB_Button_Cancle==FALSE)
			 {
				 USB_Button_Ok=TRUE;
				 USB_Button_Cancle=TRUE;
			 }
	     break;
		 
  	 default:
	     break;
  }
  return(0);
}

/**
  * @brief  USBH_USR_HexToASCII 
  *         Converts uint16_t type input variable to ASCII digit's array
  * @param  Value to be converted to ASCII
  * @retval None
  */
void USBH_USR_HexToASCII(uint32_t receivedValue)
{
  uint8_t count;
  uint8_t localTemp[10] = "         "; 
  uint8_t *ptr;
  uint8_t counter = 0;
  
  for(count = 0; count < 8; count++ )
    temp[count] = 0x20;
  
  temp[8] = 0;
  
  
  for(count = 0; count < 8; count++ )
  {
    localTemp[count] = receivedValue % 10;
    
    
    if(localTemp[count] < 0x0A)
      localTemp[count] += 0x30;
    else
      localTemp[count] += 0x37;      
    
    if(receivedValue < 10)
    {     
      break;
    }
    
    receivedValue = receivedValue / 10;
    
  }
  
  count = 0;
  
  /*check for end of data*/
  ptr = localTemp;
  while(*ptr != 0x20)
  {
    count++;
    ptr++;
  }
  
  counter = 0;
  ptr--;
  
  while(count)
  {
    temp[counter] = *ptr;
    ptr--;
    count--;
    counter++;
  }  
  
}

/**
* @brief  USBH_USR_DeInit
*         Deint User state and associated variables
* @param  None
* @retval None
*/
void USBH_USR_DeInit(void)
{
  USBH_USR_ApplicationState = USH_USR_FS_INIT;
}

