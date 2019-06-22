/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ****************
* File Name          : 
* Author             : 
* Version            : 
* Date               : 
* Description        : 
******************************************************************************/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "DGUS.h"
#include "manuloperate.h"
#include "Parameter.h"
#include "freeprogramme.h"
#include "delay.h"


u16 Read_Point_Num=0;

static u32 gs_Manul_XAxis_Position = 0;      //X��λ��
static u32 gs_Manul_ZAxis_Position = 0;      //Z��λ��
static u32 gs_Manul_LAxis_Position = 0;      //L��λ��
static u32 gs_Manul_OAxis_Position = 0;      //O��λ��
static u32 gs_Manul_AAxis_Position = 0;      //O��λ��
static u32 gs_Manul_BAxis_Position = 0;      //O��λ��

static u8  gs_Current_Manul_Axis = NO_AXIS;	 //�ֶ�����-΢��-��ѡ��
static u8  gs_Axis_Choose[4][4]={{"X�� "},{"Z�� "},{"Y�� "},{"O�� "}};

//X��-�������ü���
static PageCut PageCut_XParameter_Setting={0x4120,98,4,366,793,474,4,366};
//Z��-�������ü���
static PageCut PageCut_ZParameter_Setting={0x4130,99,4,366,793,474,4,366};
//L��-�������ü���
static PageCut PageCut_LParameter_Setting={0x4140,113,4,366,793,474,4,366};
//O��-�������ü���
static PageCut PageCut_OParameter_Setting={0x4150,121,4,366,793,474,4,366};

/**************2018.9.12 DH �������ʩ�ü�����**********************/
static PageCut LC1_Y = {0x4B30,7,100,15,171,69,99,18};	 
static PageCut LC1_N = {0x4B30,6,100,15,171,69,99,18};	

static PageCut LC2_Y = {0x4B40,7,188,15,265,69,188,15};	 
static PageCut LC2_N = {0x4B40,6,188,15,265,69,188,15};	

static PageCut LC3_Y = {0x4B50,7,276,15,356,69,276,15};	 
static PageCut LC3_N = {0x4B50,6,276,15,356,69,276,15};

static PageCut LC4_Y = {0x4B60,7,377,15,454,69,377,15};	 
static PageCut LC4_N = {0x4B60,6,377,15,454,69,377,15};	

static PageCut LC5_Y = {0x4B70,7,465,15,543,69,465,15};	 
static PageCut LC5_N = {0x4B70,6,465,15,543,69,465,15};

static PageCut LC6_Y = {0x4B80,7,101,92,171,150,101,92};	 
static PageCut LC6_N = {0x4B80,6,101,92,171,150,101,92};

static PageCut LC7_Y = {0x4B90,7,191,92,261,150,191,92};	 
static PageCut LC7_N = {0x4B90,6,191,92,261,150,191,92};	 

static PageCut LC8_Y = {0x4BA0,7,281,92,351,150,281,91};
static PageCut LC8_N = {0x4BA0,6,281,92,351,150,281,91};

static PageCut LC9_Y = {0x4BB0,7,371,92,450,150,376,92};
static PageCut LC9_N = {0x4BB0,6,371,92,450,150,372,92};

static PageCut LC10_Y = {0x4BC0,7,461,92,539,150,466,92};
static PageCut LC10_N = {0x4BC0,6,461,92,539,150,462,92};

static PageCut ChuJiao1_Y = {0x4BD0,7,98,181,180,245,98,181};
static PageCut ChuJiao1_N = {0x4BD0,6,98,181,180,245,98,181};

static PageCut ChuJiao2_Y = {0x4BE0,7,188,181,270,245,188,181};
static PageCut ChuJiao2_N = {0x4BE0,6,188,181,270,245,188,181};

static PageCut ChuJiao3_Y = {0x4BF0,7,278,181,360,245,278,181};
static PageCut ChuJiao3_N = {0x4BF0,6,278,181,360,245,278,181};

static PageCut JiaoTouXiaYi1_Y = {0x4210,7,368,184,455,245,368,181};
static PageCut JiaoTouXiaYi1_N = {0x4210,6,368,184,455,245,368,181};

static PageCut JiaoTouXiaYi2_Y = {0x4220,7,458,184,545,245,458,181};
static PageCut JiaoTouXiaYi2_N = {0x4220,6,458,184,545,245,458,181};

static PageCut JiaoTouXiaYi3_Y = {0x4230,7,548,184,620,245,548,181};
static PageCut JiaoTouXiaYi3_N = {0x4230,6,548,184,620,245,548,181};


static PageCut XiZuan1_Y = {0x4C30,7,630,181,705,245,630,181};
static PageCut XiZuan1_N = {0x4C30,6,630,181,705,245,630,181};

static PageCut XiZuan2_Y = {0x4C40,7,705,181,780,245,705,181};
static PageCut XiZuan2_N = {0x4C40,6,705,181,780,245,705,181};

static PageCut XiZuan3_Y = {0x4C50,7,101,266,181,327,101,266};
static PageCut XiZuan3_N = {0x4C50,6,101,266,181,327,101,266};

static PageCut XiZuan4_Y = {0x4C60,7,181,266,281,327,181,266};
static PageCut XiZuan4_N = {0x4C60,6,181,266,281,327,181,266};

static PageCut XiZuan5_Y = {0x4C70,7,281,266,371,327,279,262};
static PageCut XiZuan5_N = {0x4C70,6,281,266,371,327,279,262};

static PageCut XiZuan6_Y = {0x4C80,7,371,266,458,327,364,266};
static PageCut XiZuan6_N = {0x4C80,6,371,266,458,327,364,266};

static PageCut XiZuan7_Y = {0x4C90,7,456,266,533,324,453,266};
static PageCut XiZuan7_N = {0x4C90,6,456,266,533,324,453,266};

static PageCut XiZuan8_Y = {0x4CA0,7,541,266,621,327,545,270};
static PageCut XiZuan8_N = {0x4CA0,6,541,266,621,327,545,270};

static PageCut ZuanTouXiaYi1_Y = {0x4CB0,7,621,266,701,327,628,271};
static PageCut ZuanTouXiaYi1_N = {0x4CB0,6,621,266,701,327,628,271};

static PageCut ZuanTouXiaYi2_Y = {0x4CC0,7,701,268,780,327,701,268};
static PageCut ZuanTouXiaYi2_N = {0x4CC0,6,701,268,780,327,701,268};

static PageCut ZuanTouXiaYi3_Y = {0x4CD0,7,101,348,181,410,101,348};
static PageCut ZuanTouXiaYi3_N = {0x4CD0,6,101,348,181,410,101,348};

static PageCut ZuanTouXiaYi4_Y = {0x4CE0,7,181,348,281,410,181,348};
static PageCut ZuanTouXiaYi4_N = {0x4CE0,6,181,348,281,410,181,348};

static PageCut ZuanTouXiaYi5_Y = {0x4CF0,7,281,348,371,410,281,348};
static PageCut ZuanTouXiaYi5_N = {0x4CF0,6,281,348,371,410,281,348};

static PageCut ZuanTouXiaYi6_Y = {0x4D00,7,371,348,440,455,374,350};
static PageCut ZuanTouXiaYi6_N = {0x4D00,6,371,348,440,455,374,350};

static PageCut ZuanTouXiaYi7_Y = {0x4D10,7,455,348,540,545,460,350};
static PageCut ZuanTouXiaYi7_N = {0x4D10,6,455,348,540,545,460,350};

static PageCut ZuanTouXiaYi8_Y = {0x4D20,7,545,348,625,410,545,348};
static PageCut ZuanTouXiaYi8_N = {0x4D20,6,545,348,625,410,545,348};
static u8 gs_Save_Point_Num = 2;	           //��ǰ�����ı��,��ʵ�ʱ��С1,���������±�
u8 gs_LinkMove_Speed[6]={10,10,10,10,10,10};
 u8 gs_XStepMove_Distance = 10;        //X��綯����(��ʱ)
 u8 gs_ZStepMove_Distance = 10;        //Z��綯����
 u8 gs_LStepMove_Distance = 10;        //L��綯����
 u8 gs_OStepMove_Distance = 10;        //O��綯����
 u8 gs_AStepMove_Distance = 10;        //A��綯����
 u8 gs_BStepMove_Distance = 10;        //B��綯����

static u8 gs_Refresh_Speed_Para = TRUE;	     //�ٶ�-����ˢ��
static u32 gs_XSavePoint_Code = 0;	         //����-X����
static u32 gs_ZSavePoint_Code = 0;	         //����-Z����
static u32 gs_LSavePoint_Code = 0;	         //����-L����
static u32 gs_OSavePoint_Code = 0;	         //����-O����
static u8  gs_Save_Point_Pre_Page=0;	       //�������������ҳ

u8 g_Manul_Move_count=0;	                   //��������������
u8 g_Manul_Move_Flag = FALSE;                //�ֶ��ƶ���־λ�������Ƿ�Э������
u8 g_Manul_Link_Flag = FALSE;                //������������Ҫ��־������
u8 Temp_Display_Data[4]={0x20,0x20,0x20,0x20};

static PageCut PageCut_SavePointFullX={0x4160,14,8,45,792,85,8,45};   //X
static PageCut PageCut_SavePointFullZ={0x4160,91,8,45,792,85,8,45};   //Z
static PageCut PageCut_SavePointFullL={0x4160,67,8,45,792,85,8,45};   //L
static PageCut PageCut_SavePointFullO={0x4160,69,8,45,792,85,8,45};   //O

u8  Run_Mode[2][10]={"����ģʽ",
										 "�綯ģʽ"};
u8 Run_Mode_Flag=0,gs_Mode_Refresh_Flag=TRUE;
extern u8 gs_Program_Refresh_Flag;
u16 Point_Num=0;//��ǰ�������Ŀ
u16 Order_Num=0;//��ǰ����ָ����Ŀ
extern u8 gs_Current_Choose_Program;
extern u8 program_change;
extern u8 LC_Count_X[],LC_Count_Y[];
extern u8 g_Run_Program_Num;
u8 LC_Point_Flag=0;
u8 Parameter_Refresh=TRUE;										//����ҳ��ˢ�±�־

u32 LC_IO_Axis_Code = 0;	         //IO���ú��ϲ����ñ�������
u8 g_Programma_Flag=FALSE;//���򱣴��־λ
u8 DJ_IO_Set_Flag=0,DZ_IO_Set_Flag=0;//�㽺����IO����
u8 LC_Set_Flag=0,IO_Set_Flag=0;												//�ϲ�ѡ������
u32 Program_Name=0;
u32 seccount=0;//����ʱ���
u16 Distance=50;//ʵ�ʾ���
u16 Distance_Temp=0;//�����洢
u8 Key_Push_Flag;//�������±�־
u8 Axis_Move_Dir=0;//1������ 2������
u32 Point_Name=0;
extern u8 Link_Step_Refresh,Link_Step_Flag;
extern u32 IO_Choose;
//extern u32 IO_Choose1;
extern u8 Product_Num,Program_Flag[SAVEPROGRAMNUM],Select_Program_Flag;
extern u16 Current_Select_Program;
extern u32 gs_Run_Program_Name,g_Run_Program_Name;
extern u8 Point_Action_Flag;
extern u8 gs_Background_Color;
extern u8 Program_Or_Point;
extern u8 Program_Save_OrNot;
extern u8 name_temp[4];
extern u16 LCD_GearRatio_Distance[6];
extern u8 LC_ID;
extern u8 Program_Transport_Flag;
extern u8 Program_Transport_Count;
extern u8 g_Background_Color_PointEdit,gs_Background_Color_PointEdit,gs_Current_Operate_List_PointEdit;
extern u8 gs_Current_Operate_List_PointEdit,g_Background_Color_PointEdit,gs_Background_Color_PointEdit;
extern u8 g_Background_Color_PointEdit,gs_Background_Color_PointEdit,gs_Current_Operate_List_PointEdit;
extern u8 g_Background_Color_ProgramEdit,gs_Background_Color_ProgramEdit,gs_Current_Operate_List_ProgramEdit;
extern u8 Order_Type;
extern u8 LCD_Main_Page_Pre;
extern u8 Now_LC_Choose_1,Now_LC_Choose_2,Now_LC_Choose_3,Now_LC_Choose_4,Now_LC_Choose_5,Now_LC_Choose_6,
Now_LC_Choose_7,Now_LC_Choose_8,Now_LC_Choose_9,Now_LC_Choose_10;
extern u8 ChuJiao1,ChuJiao2,ChuJiao3,JiaoTouXiaYi1,JiaoTouXiaYi2,JiaoTouXiaYi3,
XiZuan1,XiZuan2,XiZuan3,XiZuan4,XiZuan5,XiZuan6,XiZuan7,XiZuan8,
ZuanTouXiaYi1,ZuanTouXiaYi2,ZuanTouXiaYi3,ZuanTouXiaYi4,ZuanTouXiaYi5,ZuanTouXiaYi6,ZuanTouXiaYi7,ZuanTouXiaYi8;
extern u8 IO_Parameter_Refresh;

//u8 gs_FreeProgram_Setting_XZ=FALSE;
/*************************************************************************
**  ��������  ManulOutputSet()
**	���������
**	�����������
**	�������ܣ�ѡ��ʾ��ʱʹ�õ�IO�˿�
**	��ע��	  ��
**  ���ߣ�    White_L   
**  �������ڣ�2018/6/28 
**************************************************************************/
void ManulOutputSet(void)
{
	if(LCD_Var_Address==0x4001||LCD_Var_Address==0x4002||LCD_Var_Address==0x4003||LCD_Var_Address==0x400C||LCD_Var_Address==0x400E||LCD_Var_Address==0x400F)
		LC_IO_Axis_Code = LCD_Var_Address;
	switch(LCD_Var_Address)
	{
		case 0x4F05:
				if(Order_Type==1||Order_Type==2||Order_Type==3||Order_Type==5)//�㽺 ���� ����
				{
					if(LC_IO_Axis_Code==0x4001)
					{
						LCD_Current_Page= 4;  //X
					}
					else if(LC_IO_Axis_Code==0x400C)
					{
						LCD_Current_Page= 9;		//O
					}
					else if(LC_IO_Axis_Code==0x400E)
					{
						LCD_Current_Page= 10;		//V
					}
				}
//				else if(Order_Type==3)//ȡ��
//				{
//				  if(Order_Check(Order_Type))//�ж�ָ���Ƿ����
//					Insert_Order(Order_Type);
//				  else
//					LCD_Current_Page=8;
//				}
				break;
		case 0x4B00:
				if(LC_ID==1)
					LC_ID=0;
				else
					LC_ID=1;					
				Parameter_Refresh=TRUE;
			break;
		case 0x4B01:
				if(LC_ID==2)
					LC_ID=0;
				else
					LC_ID=2;	
				Parameter_Refresh=TRUE;
			break;
		case 0x4B02:
				if(LC_ID==3)
					LC_ID=0;
				else
					LC_ID=3;	
				Parameter_Refresh=TRUE;
			break;
		case 0x4B03:
				if(LC_ID==4)
					LC_ID=0;
				else
					LC_ID=4;	
				Parameter_Refresh=TRUE;
			break;
		case 0x4B04:
				if(LC_ID==5)
					LC_ID=0;
				else
					LC_ID=5;	
				Parameter_Refresh=TRUE;
			break;
		case 0x4B05:
				if(LC_ID==6)
					LC_ID=0;
				else
					LC_ID=6;	
				Parameter_Refresh=TRUE;
			break;
		case 0x4B06:
				if(LC_ID==7)
					LC_ID=0;
				else
					LC_ID=7;	
				Parameter_Refresh=TRUE;
			break;
		case 0x4B07:
				if(LC_ID==8)
					LC_ID=0;
				else
					LC_ID=8;	
				Parameter_Refresh=TRUE;
			break;
		case 0x4B08:
				if(LC_ID==9)
					LC_ID=0;
				else
					LC_ID=9;	
				Parameter_Refresh=TRUE;
			break;
		case 0x4B09:
				if(LC_ID==10)
					LC_ID=0;
				else
					LC_ID=10;	
				Parameter_Refresh=TRUE;
			break;
		case 0x4B0A:
				ChuJiao1=!ChuJiao1;
				if(ChuJiao1==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFFE)|0x00000001;//1
				}
				else
				{
					IO_Choose=(IO_Choose&0x00EFFFFE);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B0B:
				ChuJiao2=!ChuJiao2;
				if(ChuJiao2==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFFD)|0x00000002;//2
				}
				else
				{
					IO_Choose=(IO_Choose&0x00EFFFFD);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B0C:
				ChuJiao3=!ChuJiao3;
				if(ChuJiao3==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFFB)|0x00000004;//3
				}
				else
				{
					IO_Choose=(IO_Choose&0x00EFFFFB);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B0D:
				JiaoTouXiaYi1=!JiaoTouXiaYi1;
				if(JiaoTouXiaYi1==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFF7)|0x00000008;//4
					Order_Temp[0]=4;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFFF7);
					Order_Temp[0]=4;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B0E:
				JiaoTouXiaYi2=!JiaoTouXiaYi2;
				if(JiaoTouXiaYi2==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFEF)|0x00000010;//5
					Order_Temp[0]=5;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFFEF);//5
					Order_Temp[0]=5;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B0F:
				JiaoTouXiaYi3=!JiaoTouXiaYi3;
				if(JiaoTouXiaYi3==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFDF)|0x00000020;//6
					Order_Temp[0]=6;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFFDF);//6
					Order_Temp[0]=6;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B10:
				XiZuan1=!XiZuan1;
				if(XiZuan1==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFBF)|0x00000040;//7
					Order_Temp[0]=7;
					Order_Temp[1]=1;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFFBF);//7
					Order_Temp[0]=7;
					Order_Temp[1]=0;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B11:
				XiZuan2=!XiZuan2;
				if(XiZuan2==1)
				{
					IO_Choose=(IO_Choose&0x00EFFF7F)|0x00000080;//8
					Order_Temp[0]=8;
					Order_Temp[1]=1;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFF7F);//8
					Order_Temp[0]=8;
					Order_Temp[1]=0;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B12:
				XiZuan3=!XiZuan3;
				if(XiZuan3==1)
				{
					IO_Choose=(IO_Choose&0x00EFFEFF)|0x00000100;//9
					Order_Temp[0]=9;
					Order_Temp[1]=1;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFEFF);//9
					Order_Temp[0]=9;
					Order_Temp[1]=0;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B13:
				XiZuan4=!XiZuan4;
				if(XiZuan4==1)
				{
					IO_Choose=(IO_Choose&0x00EFFDFF)|0x00000200;//10
					Order_Temp[0]=10;
					Order_Temp[1]=1;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFDFF);//10
					Order_Temp[0]=10;
					Order_Temp[1]=0;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B14:
				XiZuan5=!XiZuan5;
				if(XiZuan5==1)
				{
					IO_Choose=(IO_Choose&0x00EFFBFF)|0x00000400;//11
					Order_Temp[0]=11;
					Order_Temp[1]=1;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFBFF);//11
					Order_Temp[0]=11;
					Order_Temp[1]=0;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B15:
				XiZuan6=!XiZuan6;
				if(XiZuan6==1)
				{
					IO_Choose=(IO_Choose&0x00EFF7FF)|0x00000800;//13
					Order_Temp[0]=12;
					Order_Temp[1]=1;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFF7FF);//13
					Order_Temp[0]=12;
					Order_Temp[1]=0;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B16:
				XiZuan7=!XiZuan7;
				if(XiZuan7==1)
				{
					IO_Choose=(IO_Choose&0x00EFEFFF)|0x00001000;//14
					Order_Temp[0]=13;
					Order_Temp[1]=1;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFEFFF);//14
					Order_Temp[0]=13;
					Order_Temp[1]=0;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B17:
				XiZuan8=!XiZuan8;
				if(XiZuan8==1)
				{
					IO_Choose=(IO_Choose&0x00EFDFFF)|0x00002000;//15
					Order_Temp[0]=14;
					Order_Temp[1]=1;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFDFFF);//15
					Order_Temp[0]=14;
					Order_Temp[1]=0;
//					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;

		case 0x4B18:
				ZuanTouXiaYi1=!ZuanTouXiaYi1;
				if(ZuanTouXiaYi1==1)
				{
					IO_Choose=(IO_Choose&0x00EFBFFF)|0x00004000;//16
					Order_Temp[0]=15;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFBFFF);//16
					Order_Temp[0]=15;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B19:
				ZuanTouXiaYi2=!ZuanTouXiaYi2;
				if(ZuanTouXiaYi2==1)
				{
					IO_Choose=(IO_Choose&0x00EF7FFF)|0x00008000;//17
					Order_Temp[0]=16;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EF7FFF);//17
					Order_Temp[0]=16;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B1A:
				ZuanTouXiaYi3=!ZuanTouXiaYi3;
				if(ZuanTouXiaYi3==1)
				{
					IO_Choose=(IO_Choose&0x00EEFFFF)|0x00010000;//18
					Order_Temp[0]=17;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EEFFFF);//18
					Order_Temp[0]=17;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B1B:
				ZuanTouXiaYi4=!ZuanTouXiaYi4;
				if(ZuanTouXiaYi4==1)
				{
					IO_Choose=(IO_Choose&0x00EDFFFF)|0x00020000;//19
					Order_Temp[0]=18;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EDFFFF);//19
					Order_Temp[0]=18;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B1C:
				ZuanTouXiaYi5=!ZuanTouXiaYi5;
				if(ZuanTouXiaYi5==1)
				{
					IO_Choose=(IO_Choose&0x00EBFFFF)|0x00040000;//20
					Order_Temp[0]=19;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EBFFFF);//20
					Order_Temp[0]=19;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B1D:
				ZuanTouXiaYi6=!ZuanTouXiaYi6;
				if(ZuanTouXiaYi6==1)
				{
					IO_Choose=(IO_Choose&0x00E7FFFF)|0x00080000;//21
					Order_Temp[0]=20;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00E7FFFF);//21
					Order_Temp[0]=20;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B1E:
				ZuanTouXiaYi7=!ZuanTouXiaYi7;
				if(ZuanTouXiaYi7==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFFF)|0x00100000;//22
					Order_Temp[0]=21;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFFFF);//22
					Order_Temp[0]=21;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		case 0x4B1F:
				ZuanTouXiaYi8=!ZuanTouXiaYi8;
				if(ZuanTouXiaYi8==1)
				{
					IO_Choose=(IO_Choose&0x00DFFFFF)|0x00200000;//23
					Order_Temp[0]=22;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00DFFFFF);//23
					Order_Temp[0]=22;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				Parameter_Refresh=TRUE;
			break;
		default:
			break;
	}
}
/*************************************************************************
**  ��������  Key_Confirm()
**	���������
**	�����������
**	�������ܣ�ʾ�̻����ֶ�����ʱ�İ���ȷ�ϱ���
**	��ע��	  ��
**  ���ߣ�    White_L   
**  �������ڣ�2018/6/28 
**************************************************************************/
void Key_Confirm(void)
{
	if(Key_Push_Flag==1&&g_Axis_Is_Run==0)
	{
		Key_Push_Flag=0;
		if(LCD_Current_Page == 148||LCD_Current_Page==149)//X�ֶ����Ա���
		{
			LCD_New_Order=TRUE;
			LCD_Var_Address=0x4400;		
		}
		else if(LCD_Current_Page == 4||LCD_Current_Page == 9||LCD_Current_Page == 10)//Xʾ��ҳ�汣��
		{
			LCD_New_Order=TRUE;
			LCD_Var_Address=0x400A;		
		}			
		else if(LCD_Current_Page == 40)//Xҳ�汣��ȷ��
		{
			LCD_New_Order=TRUE;
			LCD_Var_Address=0x4042;
		} 
		else if(LCD_Current_Page == 95)//Zҳ�汣��ȷ��
		{
			LCD_New_Order=TRUE;
			LCD_Var_Address=0x4046;
		} 		
		else if(LCD_Current_Page == 2)//��Ʒ����ȷ�Ͻ���������
		{
			if(Product_Num!=0&&OperateProgram[gs_Current_Operate_List-1].Code==g_Run_Program_Num)
			{
				 Program_Or_Point       = 2;
				 Current_Select_Program = gs_Current_Operate_List-1;
				 gs_Background_Color_ProgramEdit=0;
				 g_Refresh_Dispaly_List_Flag = TRUE;
				 Select_Program_Flag         = TRUE;
				 LCD_Current_Page       = 8;//���ɱ�̽���
			}
			else 
			{
				LCD_Current_Page       = 161;//��ʾ�������ļ�
			}
		}
		else if(LCD_Current_Page==161)//��ʾ��������� X
		{
			Program_Or_Point=1;
			LCD_Current_Page       = 2;//��Ʒ��̽���			
		}
	}
}
/*************************************************************************
**  ��������  LCDStartUp()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/5/28
**************************************************************************/
void LCDManulOperateProcess(void)
{
	int i=0;
	if( (Robot_Origined == TRUE)&&(Robot_Status == ACTION_STOP) )
	{
		if( (LCD_Var_Address == 0x4000) && (LCD_Main_Page != MANUL_DEBUG))//������ǵ�ǰҳ�棬���л�����ǰ����
		{
			LCD_Main_Page_Pre = LCD_Main_Page;
			if(Program_Or_Point==2)
			{
				LCD_Main_Page = MANUL_DEBUG;                //LCD������Ϊʾ�̽���                    
				LCD_Current_Page = 	LCD_Main_Page;          //LCD��ǰ����Ϊʾ��
				g_Watch_IO_Control = 0; 						  	    //ʵʱIO����״̬
				Order_Temp[0]=MANUL_MODE;                   //����Ϊ�ֶ�ģʽ
				USART1_SendData(1,WORK_MODE,&Order_Temp[0]);//���ڷ��͹���ģʽ�е��ֶ�ģʽ�����ذ�
				g_Robot_Auto_Reset = FALSE;
				LC_IO_Axis_Code = 0x4001;
			}
			else if(Program_Or_Point==1||Program_Or_Point==0)
			{
				Program_Or_Point=0;
				LCD_Main_Page = MANUL_DEBUG;                //LCD������Ϊ�ֶ�����                    
				LCD_Current_Page = 	148;          		//LCD��ǰ����Ϊ�ֶ�����
				g_Watch_IO_Control = 0; 						  	    //ʵʱIO����״̬
				Order_Temp[0]=MANUL_MODE;                   //����Ϊ�ֶ�ģʽ
				USART1_SendData(1,WORK_MODE,&Order_Temp[0]);//���ڷ��͹���ģʽ�е��ֶ�ģʽ�����ذ�
				g_Robot_Auto_Reset = FALSE;
				LC_IO_Axis_Code = 0x4001;				
			}
		}
		if(LCD_Var_Address==0x4001||LCD_Var_Address==0x4002||LCD_Var_Address==0x4003||LCD_Var_Address==0x400C
			||LCD_Var_Address==0x400E||LCD_Var_Address==0x400F)
			LC_IO_Axis_Code = LCD_Var_Address;
		ManulOutputSet();															 //IO����
		switch(LCD_Var_Address)
		{
		  //X��ҳ��(XY)
			case 0x4001:	  
				 if((LCD_Main_Page == MANUL_DEBUG) && (g_Axis_Is_Run==FALSE)&&Program_Or_Point!=0)  //��е��û�����˶�״̬�¿��л�
				 {
				 	LCD_Current_Page = 4;
				}
				 else if(LCD_Main_Page==MANUL_DEBUG&&g_Axis_Is_Run==FALSE&&Program_Or_Point==0)
				 {
					 LCD_Current_Page = 148;
				 }
				 break;
				 
		  //O��ҳ��(ZOҳ��)
			case 0x400C:
//			     if(JXS_Parameter.Axis ==FAN_X_Z_L_O)
//				 {	  
			    if((LCD_Main_Page == MANUL_DEBUG) && (g_Axis_Is_Run==FALSE)&&Program_Or_Point!=0)  //��е��û�����˶�״̬�¿��л�
				  {
				 	 LCD_Current_Page = 9;
			    }
				 else if(LCD_Main_Page==MANUL_DEBUG&&g_Axis_Is_Run==FALSE&&Program_Or_Point==0)
				 {
					 LCD_Current_Page = 149;
				 }

//				 }
			    break;
		  //V��ҳ�棨AB����棩
			case 0x400E:
//			     if(JXS_Parameter.Axis ==FAN_X_Z_L_O)
//				 {	  
			    if((LCD_Main_Page == MANUL_DEBUG) && (g_Axis_Is_Run==FALSE)&&Program_Or_Point!=0)  //��е��û�����˶�״̬�¿��л�
				  {
				 	 LCD_Current_Page = 10;
				  }
				 else if(LCD_Main_Page==MANUL_DEBUG&&g_Axis_Is_Run==FALSE&&Program_Or_Point==0)
				 {
					 LCD_Current_Page = 150;
				 }
//				 }
			    break;			
			//����У׼XY
			case 0x479E:
				LCD_Current_Page = 98;
				break;
			
			//����У׼OZ
			case 0x479F:
				LCD_Current_Page = 99;
				break;
			
			//����У׼AB
			case 0x47A0:
				LCD_Current_Page = 100;
				break;

			//����У׼Xȷ��
			case 0x4085:
				LCD_Current_Page= 148;  //X		
				JXS_Parameter.GearRatio.Ratio[0] = gs_Manul_XAxis_Position/LCD_GearRatio_Distance[0];
				Order_Temp[0]=PARAMETER_FUCTION_SET;  //---��Ӧ���ư�parameter.c  129�е���UsartReceiveData[1]�������Դ�����
				Order_Temp[1]=0x07;
				Order_Temp[2]=0;				
				Order_Temp[3] = JXS_Parameter.GearRatio.Ratio[0];
				Order_Temp[4] = JXS_Parameter.GearRatio.Ratio[0]>>8;			 
				USART2_SendDataArray(5,PARAMETER_ORDER,&Order_Temp[0]);//���ڷ��͹���ģʽ�е��ֶ�ģʽ�����ذ�				
				break;
			//����У׼Yȷ��
			case 0x408D:
				LCD_Current_Page= 148;  //L	
				JXS_Parameter.GearRatio.Ratio[2] = gs_Manul_LAxis_Position/LCD_GearRatio_Distance[2];
				Order_Temp[0]=PARAMETER_FUCTION_SET;  //---��Ӧ���ư�parameter.c  129�е���UsartReceiveData[1]�������Դ�����
				Order_Temp[1]=0x07;
				Order_Temp[2]=2;				
				Order_Temp[3] = JXS_Parameter.GearRatio.Ratio[2];
				Order_Temp[4] = JXS_Parameter.GearRatio.Ratio[2]>>8;			 
				USART2_SendDataArray(5,PARAMETER_ORDER,&Order_Temp[0]);//���ڷ��͹���ģʽ�е��ֶ�ģʽ�����ذ�
				break;
			//����У׼XY ȡ��
			case 0x408E:
				LCD_Current_Page = 148;				
				break;			
			//����У׼Oȷ��
			case 0x4095:
				LCD_Current_Page= 149;  //O				
				JXS_Parameter.GearRatio.Ratio[4] = gs_Manul_OAxis_Position/LCD_GearRatio_Distance[3];
				Order_Temp[0]=PARAMETER_FUCTION_SET;  //---��Ӧ���ư�parameter.c  129�е���UsartReceiveData[1]�������Դ�����
				Order_Temp[1]=0x07;
				Order_Temp[2]=3;				
				Order_Temp[3] = JXS_Parameter.GearRatio.Ratio[3];
				Order_Temp[4] = JXS_Parameter.GearRatio.Ratio[3]>>8;			 
				USART2_SendDataArray(5,PARAMETER_ORDER,&Order_Temp[0]);//���ڷ��͹���ģʽ�е��ֶ�ģʽ�����ذ�
				break;
			//����У׼Zȷ��
			case 0x409D:
				LCD_Current_Page= 149;  //Z
				JXS_Parameter.GearRatio.Ratio[1] = gs_Manul_ZAxis_Position/LCD_GearRatio_Distance[1];
				Order_Temp[0]=PARAMETER_FUCTION_SET;  //---��Ӧ���ư�parameter.c  129�е���UsartReceiveData[1]�������Դ�����
				Order_Temp[1]=0x07;
				Order_Temp[2]=1;				
				Order_Temp[3] = JXS_Parameter.GearRatio.Ratio[1];
				Order_Temp[4] = JXS_Parameter.GearRatio.Ratio[1]>>8;			 
				USART2_SendDataArray(5,PARAMETER_ORDER,&Order_Temp[0]);//���ڷ��͹���ģʽ�е��ֶ�ģʽ�����ذ�
				break;	
			//����У׼OZ ȡ��
			case 0x409E:
				LCD_Current_Page = 149;				
				break;	

			//����У׼Aȷ��
			case 0x40AF:
				LCD_Current_Page= 150;  //V�ֶ�����		
				JXS_Parameter.GearRatio.Ratio[4] = gs_Manul_AAxis_Position/LCD_GearRatio_Distance[4];
				Order_Temp[0]=PARAMETER_FUCTION_SET;  //---��Ӧ���ư�parameter.c  129�е���UsartReceiveData[1]�������Դ�����
				Order_Temp[1]=0x07;
				Order_Temp[2]=4;				
				Order_Temp[3] = JXS_Parameter.GearRatio.Ratio[4];
				Order_Temp[4] = JXS_Parameter.GearRatio.Ratio[4]>>8;			 
				USART2_SendDataArray(5,PARAMETER_ORDER,&Order_Temp[0]);//���ڷ��͹���ģʽ�е��ֶ�ģʽ�����ذ�
				break;
			//����У׼Bȷ��
			case 0x40AD:
				LCD_Current_Page= 150;  //B
				JXS_Parameter.GearRatio.Ratio[5] = gs_Manul_BAxis_Position/LCD_GearRatio_Distance[5];
				Order_Temp[0]=PARAMETER_FUCTION_SET;  //---��Ӧ���ư�parameter.c  129�е���UsartReceiveData[1]�������Դ�����
				Order_Temp[1]=0x07;
				Order_Temp[2]=5;				
				Order_Temp[3] = JXS_Parameter.GearRatio.Ratio[5];
				Order_Temp[4] = JXS_Parameter.GearRatio.Ratio[5]>>8;			 
				USART2_SendDataArray(5,PARAMETER_ORDER,&Order_Temp[0]);//���ڷ��͹���ģʽ�е��ֶ�ģʽ�����ذ�
				break;	
			//����У׼AB ȡ��
			case 0x40AE:
				LCD_Current_Page = 150;				
				break;	
			
			case 0x4080://Xȷ��
				LCD_GearRatio_Distance[0]=LCD_Key_Value;
				break;

			case 0x4086://Yȷ��
				LCD_GearRatio_Distance[2]=LCD_Key_Value;
				break;

			case 0x4096://Zȷ��
				LCD_GearRatio_Distance[1]=LCD_Key_Value;
				break;
			
			case 0x4090://Oȷ��
				LCD_GearRatio_Distance[3]=LCD_Key_Value;
				break;
			
			case 0x40A0://Aȷ��
				LCD_GearRatio_Distance[4]=LCD_Key_Value;
				break;

			case 0x40A6://Bȷ��
				LCD_GearRatio_Distance[5]=LCD_Key_Value;
				break;

			//X��  ʾ��ָ��水��
			case 0x400A:	  				 
				if(Order_Check(Order_Type))
					Insert_Order(Order_Type);//����ʾ�̵�
				else
				{
					LCD_Current_Page=8;
				 /*******ˢ�����ɱ�̽���**********/
					LCD_Main_Page = FREE_PROGRAMME;
					g_Watch_IO_Control      = 0; 						          //IO��ʵʱ��ع�
					g_Refresh_Dispaly_List_Flag=TRUE;                         //ˢ�³�����ʾ�б�
					g_Robot_Auto_Reset = FALSE;                               //��е���Զ���λ��
					g_Refresh_Dispaly_List_Flag=TRUE; 
				}
			   break;

			//X���ɱ�̲�������-����	 
			case 0x400B:	  
			 	 LCD_Current_Page = FREE_PROGRAMME;				
				 Current_Operate_Program.Program[gs_Current_Operate_List-1].Value1=g_XAxis_Position|0xf0000000 ;
				 g_Refresh_Dispaly_List_Flag=TRUE;
				 g_FreeProgram_Setting_XZ = FALSE;
				 //gs_FreeProgram_Setting_XZ=FALSE;
			   break;	

			//Z�ᱣ��
			case 0x401A:	  
				 	LCD_Current_Page = 95;
					g_Z_SavePoint_Flag = TRUE;				 				 
			   break;
				 
			//Z���ɱ�̲�������-����	 
			case 0x401B:	 
		 		 LCD_Current_Page = FREE_PROGRAMME;				
				 Current_Operate_Program.Program[gs_Current_Operate_List-1].Value1=g_ZAxis_Position|0xf0000000;
				 g_Refresh_Dispaly_List_Flag=TRUE;
				 g_FreeProgram_Setting_XZ = FALSE;
			   break;	
			
			//L�ᱣ��
			case 0x4066:	   
				 	LCD_Current_Page = 114;
					g_L_SavePoint_Flag = TRUE;			 
			   break;
			
      //L���ɱ�̲�������-����				 
			case 0x4067:	  
		 		 LCD_Current_Page = FREE_PROGRAMME;				
				 Current_Operate_Program.Program[gs_Current_Operate_List-1].Value1=g_LAxis_Position|0xf0000000;
				 g_Refresh_Dispaly_List_Flag=TRUE;
				 g_FreeProgram_Setting_XZ = FALSE;
			   break;	
			
      //O�����ɱ�̲�������-����				 
			case 0x4077:	  
		 		 LCD_Current_Page = FREE_PROGRAMME;				
				 Current_Operate_Program.Program[gs_Current_Operate_List-1].Value1=g_OAxis_Position|0xf0000000;
				 g_Refresh_Dispaly_List_Flag=TRUE;
				 g_FreeProgram_Setting_XZ = FALSE;
			   break;	
			
			//X��ȷ��-�·������	 
			case 0x4042:
				 for(i=0;i<25;i++)
				 {
				 	if(Manul_Save_PointX[i].Name==gs_XSavePoint_Code)    //�Ƿ���������ͬ
					{
						gs_Save_Point_Num=i;
						break;
					}
				 }			     				 
				 Manul_Save_PointX[gs_Save_Point_Num].Flag  =1;
				 Manul_Save_PointX[gs_Save_Point_Num].Name  = Point_Name;
				 Manul_Save_PointX[gs_Save_Point_Num].Point = gs_Manul_XAxis_Position;
				 Order_Temp[0] = XAXIS_SAVE_POINT;	                   //X��-���棬����	 0x05
				 Order_Temp[1] = gs_Save_Point_Num+1;	                 //���	 -1���������±�
				 Order_Temp[2] = Manul_Save_PointX[gs_Save_Point_Num].Flag;
				 Order_Temp[3] = Manul_Save_PointX[gs_Save_Point_Num].Name;
				 Order_Temp[4] = Manul_Save_PointX[gs_Save_Point_Num].Name>>8;
				 Order_Temp[5] = Manul_Save_PointX[gs_Save_Point_Num].Name>>16;
				 Order_Temp[6] = Manul_Save_PointX[gs_Save_Point_Num].Name>>24;
				 Order_Temp[7] = Manul_Save_PointX[gs_Save_Point_Num].Point;
				 Order_Temp[8] = Manul_Save_PointX[gs_Save_Point_Num].Point>>8;
				 Order_Temp[9] = Manul_Save_PointX[gs_Save_Point_Num].Point>>16;
				 Order_Temp[10]= Manul_Save_PointX[gs_Save_Point_Num].Point>>24;
				 USART1_SendData(11,MANUL_OPERATE,&Order_Temp[0]);     //ͨ�ţ���λ��ǰ-��λ�ں�	
				 //Y������ͬʱ���
				 for(i=0;i<25;i++)                                  //�Ƿ���������ͬ
				 {
					if(Manul_Save_PointL[i].Name==gs_LSavePoint_Code)
					{
						gs_Save_Point_Num=i;
						break;
					}
				 }			     				 
				 Manul_Save_PointL[gs_Save_Point_Num].Flag=1;
				 Manul_Save_PointL[gs_Save_Point_Num].Name  = Point_Name;
				 Manul_Save_PointL[gs_Save_Point_Num].Point = gs_Manul_LAxis_Position;
				 Order_Temp[0] = LAXIS_SAVE_POINT;	                //L��-���棬����	 0x0A
				 Order_Temp[1] = gs_Save_Point_Num+1;	              //���	 -1���������±�
				 Order_Temp[2] = Manul_Save_PointL[gs_Save_Point_Num].Flag;
				 Order_Temp[3] = Manul_Save_PointL[gs_Save_Point_Num].Name;
				 Order_Temp[4] = Manul_Save_PointL[gs_Save_Point_Num].Name>>8;
				 Order_Temp[5] = Manul_Save_PointL[gs_Save_Point_Num].Name>>16;
				 Order_Temp[6] = Manul_Save_PointL[gs_Save_Point_Num].Name>>24;
				 Order_Temp[7] = Manul_Save_PointL[gs_Save_Point_Num].Point;
				 Order_Temp[8] = Manul_Save_PointL[gs_Save_Point_Num].Point>>8;
				 Order_Temp[9] = Manul_Save_PointL[gs_Save_Point_Num].Point>>16;
				 Order_Temp[10]= Manul_Save_PointL[gs_Save_Point_Num].Point>>24;
				 USART1_SendData(11,MANUL_OPERATE,&Order_Temp[0]);	//ͨ�ţ���λ��ǰ-��λ�ں�
				 
				 //Z������ͬʱ���
				 for(i=0;i<25;i++)
				 {
					if(Manul_Save_PointZ[i].Name==gs_ZSavePoint_Code)    //�Ƿ���������ͬ
					{
						gs_Save_Point_Num=i;
						break;
					}
				 }			
				 Manul_Save_PointZ[gs_Save_Point_Num].Flag  =1;
				 Manul_Save_PointZ[gs_Save_Point_Num].Name  = Point_Name;
				 Manul_Save_PointZ[gs_Save_Point_Num].Point = gs_Manul_ZAxis_Position;
				 Order_Temp[0] = ZAXIS_SAVE_POINT;	                   //X��-���棬����	 0x05
				 Order_Temp[1] = gs_Save_Point_Num+1;	                 //���	 -1���������±�
				 Order_Temp[2] = Manul_Save_PointZ[gs_Save_Point_Num].Flag;
				 Order_Temp[3] = Manul_Save_PointZ[gs_Save_Point_Num].Name;
				 Order_Temp[4] = Manul_Save_PointZ[gs_Save_Point_Num].Name>>8;
				 Order_Temp[5] = Manul_Save_PointZ[gs_Save_Point_Num].Name>>16;
				 Order_Temp[6] = Manul_Save_PointZ[gs_Save_Point_Num].Name>>24;
				 Order_Temp[7] = Manul_Save_PointZ[gs_Save_Point_Num].Point;
				 Order_Temp[8] = Manul_Save_PointZ[gs_Save_Point_Num].Point>>8;
				 Order_Temp[9] = Manul_Save_PointZ[gs_Save_Point_Num].Point>>16;
				 Order_Temp[10]= Manul_Save_PointZ[gs_Save_Point_Num].Point>>24;
				 USART1_SendData(11,MANUL_OPERATE,&Order_Temp[0]);     //ͨ�ţ���λ��ǰ-��λ�ں�
				 
				 gs_Save_Point_Num=0;
				 LC_Point_Flag=0;
				 LCD_Current_Page=148;				 
				 break;

			//X��ȷ��-�·������	 
			case 0x404E:
				 //Z������ͬʱ���
				 for(i=0;i<25;i++)
				 {
					if(Manul_Save_PointZ[i].Name==gs_ZSavePoint_Code)    //�Ƿ���������ͬ
					{
						gs_Save_Point_Num=i;
						break;
					}
				 }			
				 Manul_Save_PointZ[gs_Save_Point_Num].Flag  =1;
				 Manul_Save_PointZ[gs_Save_Point_Num].Name  = Point_Name;
				 Manul_Save_PointZ[gs_Save_Point_Num].Point = gs_Manul_ZAxis_Position;
				 Order_Temp[0] = ZAXIS_SAVE_POINT;	                   //X��-���棬����	 0x05
				 Order_Temp[1] = gs_Save_Point_Num+1;	                 //���	 -1���������±�
				 Order_Temp[2] = Manul_Save_PointZ[gs_Save_Point_Num].Flag;
				 Order_Temp[3] = Manul_Save_PointZ[gs_Save_Point_Num].Name;
				 Order_Temp[4] = Manul_Save_PointZ[gs_Save_Point_Num].Name>>8;
				 Order_Temp[5] = Manul_Save_PointZ[gs_Save_Point_Num].Name>>16;
				 Order_Temp[6] = Manul_Save_PointZ[gs_Save_Point_Num].Name>>24;
				 Order_Temp[7] = Manul_Save_PointZ[gs_Save_Point_Num].Point;
				 Order_Temp[8] = Manul_Save_PointZ[gs_Save_Point_Num].Point>>8;
				 Order_Temp[9] = Manul_Save_PointZ[gs_Save_Point_Num].Point>>16;
				 Order_Temp[10]= Manul_Save_PointZ[gs_Save_Point_Num].Point>>24;
				 USART1_SendData(11,MANUL_OPERATE,&Order_Temp[0]);     //ͨ�ţ���λ��ǰ-��λ�ں�
				 
				 gs_Save_Point_Num=0;
				 LC_Point_Flag=0;
				 LCD_Current_Page=149;				 
				 break;
				 
			//X�ᱣ��-ȡ��
			case 0x4043:	 
				 if(g_FreeProgram_Setting_XZ)
				 {
				 	LCD_Current_Page = 98;					
				 }
			   else
				 {
					if(Program_Or_Point==2)
					{
						LCD_Main_Page = FREE_PROGRAMME;
						LCD_Current_Page        = 8;
						g_Watch_IO_Control      = 0; 						          //IO��ʵʱ��ع�
						g_Refresh_Dispaly_List_Flag=TRUE;                         //ˢ�³�����ʾ�б�
						g_Robot_Auto_Reset = FALSE;                               //��е���Զ���λ��
						g_Refresh_Dispaly_List_Flag=TRUE;
					}
					else 
						LCD_Current_Page = 148;
				 }
			   break;

			//X�ᱣ��-ȡ��
			case 0x404F:	 
				 if(g_FreeProgram_Setting_XZ)
				 {
				 	LCD_Current_Page = 98;					
				 }
			   else
				 {
					if(Program_Or_Point==2)
					{
						LCD_Main_Page = FREE_PROGRAMME;
						LCD_Current_Page        = 8;
						g_Watch_IO_Control      = 0; 						          //IO��ʵʱ��ع�
						g_Refresh_Dispaly_List_Flag=TRUE;                         //ˢ�³�����ʾ�б�
						g_Robot_Auto_Reset = FALSE;                               //��е���Զ���λ��
						g_Refresh_Dispaly_List_Flag=TRUE;
					}
					else 
						LCD_Current_Page = 149;
				 }
			   break;
				 
		  //Z��㱣��
			case 0x4044:	                                           //����-����		 
//				 gs_ZSavePoint_Code=LCD_Var_Data;
//				 g_Z_SavePoint_Flag = TRUE;
			     break;
			//Z��ȷ��-�·������
			case 0x4046:	 
				 for(i=0;i<25;i++)
				 {
					if(Manul_Save_PointZ[i].Name==gs_ZSavePoint_Code)    //�Ƿ���������ͬ
					{
						gs_Save_Point_Num=i;
						break;
					}
				 }			
				 Manul_Save_PointZ[gs_Save_Point_Num].Flag  =1;
				 Manul_Save_PointZ[gs_Save_Point_Num].Name  = LCD_Var_Data;
				 Manul_Save_PointZ[gs_Save_Point_Num].Point = gs_Manul_ZAxis_Position;
				 Order_Temp[0] = ZAXIS_SAVE_POINT;	                   //X��-���棬����	 0x05
				 Order_Temp[1] = gs_Save_Point_Num+1;	                 //���	 -1���������±�
				 Order_Temp[2] = Manul_Save_PointZ[gs_Save_Point_Num].Flag;
				 Order_Temp[3] = Manul_Save_PointZ[gs_Save_Point_Num].Name;
				 Order_Temp[4] = Manul_Save_PointZ[gs_Save_Point_Num].Name>>8;
				 Order_Temp[5] = Manul_Save_PointZ[gs_Save_Point_Num].Name>>16;
				 Order_Temp[6] = Manul_Save_PointZ[gs_Save_Point_Num].Name>>24;
				 Order_Temp[7] = Manul_Save_PointZ[gs_Save_Point_Num].Point;
				 Order_Temp[8] = Manul_Save_PointZ[gs_Save_Point_Num].Point>>8;
				 Order_Temp[9] = Manul_Save_PointZ[gs_Save_Point_Num].Point>>16;
				 Order_Temp[10]= Manul_Save_PointZ[gs_Save_Point_Num].Point>>24;
				 USART1_SendData(11,MANUL_OPERATE,&Order_Temp[0]);     //ͨ�ţ���λ��ǰ-��λ�ں�	
				 gs_Save_Point_Num=0;
				 LC_Point_Flag=0;
				 LCD_Current_Page = 149;	
	       break;

			//Z�ᱣ��-ȡ��
			case 0x4047:	 
				 if(g_FreeProgram_Setting_XZ)
				 {
				 	LCD_Current_Page = 99;		                        //�ֶ�����-Z��������
				 }
				 else
				 {
					if(Program_Or_Point==2)
					{
						LCD_Main_Page = FREE_PROGRAMME;
						LCD_Current_Page        = 8;
						g_Watch_IO_Control      = 0; 						          //IO��ʵʱ��ع�
						g_Refresh_Dispaly_List_Flag=TRUE;                         //ˢ�³�����ʾ�б�
						g_Robot_Auto_Reset = FALSE;                               //��е���Զ���λ��
						g_Refresh_Dispaly_List_Flag=TRUE;						
					}
					else 
						LCD_Current_Page = 149;		                        //�ص�Z���ֶ����Խ���
				 }
			   break;

			//V�ᱣ�� ͨ��O�� 
			case 0x4742:	  
				 	LCD_Current_Page = 71;
					g_O_SavePoint_Flag = TRUE;		 
			   break;
			//W��㱣�� ͨ��O�� 
			case 0x474F:	                                        //����-����			 
				 	LCD_Current_Page = 71;
					g_O_SavePoint_Flag = TRUE;		 
			     break;
			case 0x4040:
				Point_Name=LCD_Var_Data;
				break;
      //΢��-����
			case 0x4054:	 
				 if(Robot_Status == ACTION_STOP )                    //��е��״̬Ϊֹͣ
				 {
					 Order_Temp[0] = MANUL_FINE_ADJUST;                //΢������
					 Order_Temp[1] = 0;
					 USART1_SendData(2,MANUL_OPERATE,&Order_Temp[0]);
					 TIM_Cmd(TIM3, DISABLE);                           //ʧ�ܶ�ʱ��
					 if(g_FreeProgram_Setting_XZ)                      //���ɱ��X-Z�����ò�����־λ
					 {
						 if(g_Current_Manul_Axis==X_AXIS)	               //�ֶ�΢��ʱ��ǰѡ��X��
						 {
						 	LCD_Current_Page = 98;					               //�ֶ�����-X��������
						 }
					   else if(g_Current_Manul_Axis==Z_AXIS)           //�ֶ�΢��ʱ��ǰѡ��Z��
						 {
						 	LCD_Current_Page = 99;                         //�ֶ�����-Z��������
						 }
					   else if(g_Current_Manul_Axis==L_AXIS)           //�ֶ�΢��ʱ��ǰѡ��L��
						 {
						 	LCD_Current_Page = 113;                        //�ֶ�����-L��������
						 }	
						 else                                            //�ֶ�΢��ʱ��ǰѡ��O��
						 {
						 	LCD_Current_Page = 121;	                       //�ֶ�����-O��������
						 }	
					 }
					 else
					 {
						 if(g_Current_Manul_Axis==X_AXIS)	               //�ֶ�΢��ʱ��ǰѡ��X��
						 {
						 	LCD_Current_Page = 14;					               //�ص�X���ֶ����Խ���
						 }
					   else if(g_Current_Manul_Axis==Z_AXIS)           //�ֶ�΢��ʱ��ǰѡ��Z��
						 {
						 	LCD_Current_Page = 91;                         //�ص�Z���ֶ����Խ���
						 }
					   else if(g_Current_Manul_Axis==L_AXIS)           //�ֶ�΢��ʱ��ǰѡ��L��
						 {
						 	LCD_Current_Page = 67;                         //�ص�L���ֶ����Խ���
						 }	
						 else                                            //�ֶ�΢��ʱ��ǰѡ��O��
						 {
						 	LCD_Current_Page = 69;	                       //�ص�O���ֶ����Խ���
						 }					 					 
					 }			 
					 //g_Manule_Pulse=TIM_GetCounter(TIM3);
				 }
			   break;

			//�洢������-����
			case 0x4056:	 
				 LCD_Current_Page=gs_Save_Point_Pre_Page;            //LCD��ǰ����Ϊ�������������ҳ
				 gs_Save_Point_Pre_Page=0;
			   break;

	 /*********************-------------�ֶ�-��������------------------************************/
			//X-���ư���
			case 0x4020:
				 if(LCD_Key_Value==1&&g_Axis_Is_Run==0)
				 {
					 Order_Temp[0] =XAXIS_MOVE_LEFT;                 //X��-����              
					 Order_Temp[1] = LINK_MOVE_MODE;	                   //����ģʽ
					 Order_Temp[2] = 1;
					 Order_Temp[3] = gs_LinkMove_Speed[X_Axsis];
					 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);    //ͨ�ţ���λ��ǰ-��λ�ں�
				 	 g_Manul_Link_Flag = TRUE;
				 }
				 if(LCD_Key_Value==0)
				 {
					 Order_Temp[0] =XAXIS_MOVE_LEFT;                 //X��-����              
					 Order_Temp[1] = LINK_MOVE_MODE;	                   //����ģʽ
					 Order_Temp[2] = 0;
					 Order_Temp[3] = gs_LinkMove_Speed[X_Axsis];
					 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);    //ͨ�ţ���λ��ǰ-��λ�ں�
					 g_Manul_Link_Flag = FALSE;			 
				 }	
				 Axis_Position_Scan=TRUE;			 									      //�Զ�����ʱ����λ�ò�ѯ					 
				 break;	
				
      //X-���ư���				 
			case 0x4022:
				 if(LCD_Key_Value==1&&g_Axis_Is_Run==0)
				 {
					 Order_Temp[0] =XAXIS_MOVE_RIGHT;                //X��-����
					 Order_Temp[1] = LINK_MOVE_MODE;	                   //����ģʽ
					 Order_Temp[2] = 1;
					 Order_Temp[3] = gs_LinkMove_Speed[X_Axsis];
					 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);    //ͨ�ţ���λ��ǰ-��λ�ں�//---��Ӧ��λ��manual.c929 ��MANUL_OPERATE ��������Ӧ����λ����signawatch.c283
					 g_Manul_Link_Flag = TRUE;
				 }
				 if(LCD_Key_Value==0)
				 {
					 Order_Temp[0] =XAXIS_MOVE_RIGHT;                 //X��-����              
					 Order_Temp[1] = LINK_MOVE_MODE;	                   //����ģʽ
					 Order_Temp[2] = 0;
					 Order_Temp[3] = gs_LinkMove_Speed[X_Axsis];
					 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);    //ͨ�ţ���λ��ǰ-��λ�ں�    //ͨ�ţ���λ��ǰ-��λ�ں�
					 g_Manul_Link_Flag = FALSE;			 
				 }	
				 Axis_Position_Scan=TRUE;			 									 			//�Զ�����ʱ����λ�ò�ѯ		 
				 break;	

			//Z-���ư���
			case 0x4024:	  
				 if(LCD_Key_Value==1&&g_Axis_Is_Run==0)
				 {
					 Order_Temp[0] =ZAXIS_MOVE_LEFT;                  //Z��-����
					 Order_Temp[1] = LINK_MOVE_MODE;	                    //����ģʽ
					 Order_Temp[2] = 1;
					 Order_Temp[3] = gs_LinkMove_Speed[Z_Axsis];
					 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);    //ͨ�ţ���λ��ǰ-��λ�ں�     //ͨ�ţ���λ��ǰ-��λ�ں�
					 g_Manul_Link_Flag = TRUE;
				 }
				 if(LCD_Key_Value==0)
				 {
					 Order_Temp[0] =ZAXIS_MOVE_LEFT;                 //X��-����              
					 Order_Temp[1] = LINK_MOVE_MODE;	                   //����ģʽ
					 Order_Temp[2] = 0;
					 Order_Temp[3] = gs_LinkMove_Speed[Z_Axsis];
					 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);    //ͨ�ţ���λ��ǰ-��λ�ں�    //ͨ�ţ���λ��ǰ-��λ�ں�
					 g_Manul_Link_Flag = FALSE;			 
				 }
				 Axis_Position_Scan=TRUE;			 									 			 //�Զ�����ʱ����λ�ò�ѯ		 			 					 
			   break;		

			//Z-���ư���
			case 0x4026:	
				 if(LCD_Key_Value==1&&g_Axis_Is_Run==0)
				 {
					 Order_Temp[0] =ZAXIS_MOVE_RIGHT;                  //Z��-����
					 Order_Temp[1] = LINK_MOVE_MODE;	                     //����ģʽ
					 Order_Temp[2] = 1;
					 Order_Temp[3] = gs_LinkMove_Speed[Z_Axsis];
					 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);    //ͨ�ţ���λ��ǰ-��λ�ں�      //ͨ�ţ���λ��ǰ-��λ�ں�
					 g_Manul_Link_Flag = TRUE;
				 }
				 if(LCD_Key_Value==0)
				 {
					 Order_Temp[0] =ZAXIS_MOVE_RIGHT;                 //X��-����              
					 Order_Temp[1] = LINK_MOVE_MODE;	                   //����ģʽ
					 Order_Temp[2] = 0;
					 Order_Temp[3] = gs_LinkMove_Speed[Z_Axsis];
					 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);    //ͨ�ţ���λ��ǰ-��λ�ں�    //ͨ�ţ���λ��ǰ-��λ�ں�
					 g_Manul_Link_Flag = FALSE;			 
				 }				
				 Axis_Position_Scan=TRUE;		 					                 //�Զ�����ʱ����λ�ò�ѯ
			   break;	

			//L-ǰ�ư���
			case 0x4940:	  
				if(g_Axis_Is_Run==0)
				{
				 Axis_Move_Dir=1;
				 Order_Temp[0] = LAXIS_MOVE_LEFT;                      //L��-����
				 Order_Temp[1] = LINK_MOVE_MODE;	                     //����ģʽ
				 Order_Temp[2] = 1;
				 Order_Temp[3] = gs_LinkMove_Speed[L_Axsis];
				 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);    //ͨ�ţ���λ��ǰ-��λ�ں�      //ͨ�ţ���λ��ǰ-��λ�ں�
				 g_Manul_Link_Flag = TRUE;	
				 Axis_Position_Scan=TRUE;			 		                      //�Զ�����ʱ����λ�ò�ѯ	
				}					
			   break;		
			//L-̧��
			case 0x4960:	
				if(Axis_Move_Dir==1)
				 Order_Temp[0] = LAXIS_MOVE_LEFT;                      //L��-����
				else 	if(Axis_Move_Dir==2)
				 Order_Temp[0] = LAXIS_MOVE_RIGHT;                      //L��-����
				 Order_Temp[1] = LINK_MOVE_MODE;	                     //����ģʽ
				 Order_Temp[2] = 0;
				 Order_Temp[3] = gs_LinkMove_Speed[L_Axsis];
				 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);      //ͨ�ţ���λ��ǰ-��λ�ں�
				 g_Manul_Link_Flag = FALSE;				 
				 Axis_Position_Scan=TRUE;			 		                      //�Զ�����ʱ����λ�ò�ѯ	
				 Axis_Move_Dir=0;				
			   break;	
		  //L-���ư���
			case 0x4950:	
				if(g_Axis_Is_Run==0)
				{				
				 Axis_Move_Dir=2;
				 Order_Temp[0] = LAXIS_MOVE_RIGHT;                      //L��-����
				 Order_Temp[1] = LINK_MOVE_MODE;	                      //����ģʽ
				 Order_Temp[2] = 1;
				 Order_Temp[3] = gs_LinkMove_Speed[L_Axsis];
				 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);       //ͨ�ţ���λ��ǰ-��λ�ں�
				 g_Manul_Link_Flag = TRUE;
				 Axis_Position_Scan=TRUE;		 	                          //�Զ�����ʱ����λ�ò�ѯ
				}					
			   break;	

			//O-���ư���
			case 0x402C:	  
				 if(LCD_Key_Value==1&&g_Axis_Is_Run==0)
				 {
					 Order_Temp[0] = OAXIS_MOVE_LEFT;                       //O��-����   
					 Order_Temp[1] = LINK_MOVE_MODE;	                      //����ģʽ
					 Order_Temp[2] = 1;
					 Order_Temp[3] = gs_LinkMove_Speed[O_Axsis];
					 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);       //ͨ�ţ���λ��ǰ-��λ�ں�
					 g_Manul_Link_Flag = TRUE;
				 }
				 if(LCD_Key_Value==0)
				 {
					 Order_Temp[0] = OAXIS_MOVE_LEFT;                 //X��-����              
					 Order_Temp[1] = LINK_MOVE_MODE;	                   //����ģʽ
					 Order_Temp[2] = 0;
					 Order_Temp[3] = gs_LinkMove_Speed[O_Axsis];
					 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);    //ͨ�ţ���λ��ǰ-��λ�ں�
					 g_Manul_Link_Flag = FALSE;			 
				 }	
				 Axis_Position_Scan=TRUE;			 									 				 //�Զ�����ʱ����λ�ò�ѯ	 			 					 
			   break;		

			//O-���ư���  
			case 0x402E:	
				 if(LCD_Key_Value==1&&g_Axis_Is_Run==0)
				 {
					 Order_Temp[0] = OAXIS_MOVE_RIGHT;                       //O��-����
					 Order_Temp[1] = LINK_MOVE_MODE;	                       //����ģʽ
					 Order_Temp[2] = 1;
					 Order_Temp[3] = gs_LinkMove_Speed[O_Axsis];
					 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);         //ͨ�ţ���λ��ǰ-��λ�ں�
					 g_Manul_Link_Flag = TRUE;
				 }
				 if(LCD_Key_Value==0)
				 {
					 Order_Temp[0] = OAXIS_MOVE_RIGHT;                 //X��-����              
					 Order_Temp[1] = LINK_MOVE_MODE;	                   //����ģʽ
					 Order_Temp[2] = 0;
					 Order_Temp[3] = gs_LinkMove_Speed[O_Axsis];
					 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);     //ͨ�ţ���λ��ǰ-��λ�ں�
					 g_Manul_Link_Flag = FALSE;			 
				 }				
				 Axis_Position_Scan=TRUE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
			   break;		

			//A-���ư���
			case 0x49D0:	
				if(g_Axis_Is_Run==0)
				{		
				 Axis_Move_Dir=1;
				 Order_Temp[0] = AAXIS_MOVE_LEFT;                       //O��-����   
				 Order_Temp[1] = LINK_MOVE_MODE;	                      //����ģʽ
				 Order_Temp[2] = 1;
				 Order_Temp[3] = gs_LinkMove_Speed[A_Axsis];
				 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);        //ͨ�ţ���λ��ǰ-��λ�ں�
				 g_Manul_Link_Flag = TRUE;	
				 Axis_Position_Scan=TRUE;			 									 				 //�Զ�����ʱ����λ�ò�ѯ	 		
				}					
			   break;		

			//V-���ư���
			case 0x49E0:
				if(g_Axis_Is_Run==0)
				{						
				 Axis_Move_Dir=2;				
				 Order_Temp[0] = AAXIS_MOVE_RIGHT;                       //O��-����
				 Order_Temp[1] = LINK_MOVE_MODE;	                       //����ģʽ
				 Order_Temp[2] = 1;
				 Order_Temp[3] = gs_LinkMove_Speed[A_Axsis];
				 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);        //ͨ�ţ���λ��ǰ-��λ�ں�
				 g_Manul_Link_Flag = TRUE;
				 Axis_Position_Scan=TRUE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				}
			   break;	
				 
			//V-����̧��
			case 0x49F0:	  
				if(Axis_Move_Dir==1)
				 Order_Temp[0] = AAXIS_MOVE_LEFT;                      //L��-����
				else 	if(Axis_Move_Dir==2)
				 Order_Temp[0] = AAXIS_MOVE_RIGHT;                      //L��-����
				 Order_Temp[1] = LINK_MOVE_MODE;	                       //����ģʽ
				 Order_Temp[2] = 0;
				 Order_Temp[3] = gs_LinkMove_Speed[A_Axsis];
				 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);        //ͨ�ţ���λ��ǰ-��λ�ں�
				 g_Manul_Link_Flag = FALSE;				 
				 Axis_Position_Scan=TRUE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 Axis_Move_Dir=0;
			   break;		
				 
			//W-���ư���
			case 0x4A00:	  
				if(g_Axis_Is_Run==0)
				{		
				 Axis_Move_Dir=1;
				 Order_Temp[0] = BAXIS_MOVE_LEFT;                       //W��-����   
				 Order_Temp[1] = LINK_MOVE_MODE;	                      //����ģʽ
				 Order_Temp[2] = 1;
				 Order_Temp[3] = gs_LinkMove_Speed[B_Axsis];
				 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);       //ͨ�ţ���λ��ǰ-��λ�ں�
				 g_Manul_Link_Flag = TRUE;
				 Axis_Position_Scan=TRUE;			 									 				 //�Զ�����ʱ����λ�ò�ѯ	
				}					
			   break;		

			//W-���ư���
			case 0x4A10:
				if(g_Axis_Is_Run==0)
				{						
				 Axis_Move_Dir=2;
				 Order_Temp[0] = BAXIS_MOVE_RIGHT;                       //W��-����
				 Order_Temp[1] = LINK_MOVE_MODE;	                       //����ģʽ
				 Order_Temp[2] = 1;
				 Order_Temp[3] = gs_LinkMove_Speed[B_Axsis];
				 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);       //ͨ�ţ���λ��ǰ-��λ�ں�
				 g_Manul_Link_Flag = TRUE;
				 Axis_Position_Scan=TRUE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				}
			   break;

			case 0x4A20:	  //W����̧��
				if(Axis_Move_Dir==1)
				 Order_Temp[0] = BAXIS_MOVE_LEFT;                      //W��-����
				else 	if(Axis_Move_Dir==2)
				 Order_Temp[0] = BAXIS_MOVE_RIGHT;                      //W��-����
				 Order_Temp[1] = LINK_MOVE_MODE;	                       //����ģʽ
				 Order_Temp[2] = 0;
				 Order_Temp[3] = gs_LinkMove_Speed[B_Axsis];
				 USART1_SendData(4,MANUL_OPERATE,&Order_Temp[0]);        //ͨ�ţ���λ��ǰ-��λ�ں�
				 g_Manul_Link_Flag = FALSE;				 
				 Axis_Position_Scan=TRUE;		 					                    //�Զ�����ʱ����λ�ò�ѯ
				 Axis_Move_Dir=0;
			   break;
/**************2018.6.21 white_l**********************************/				 
			case 0x4400:											//�����ϲ�X���걣�水��
			   if(Manul_Save_PointX[24].Flag==1)
				 {
					gs_Save_Point_Num = 25;					          //��ǰ�����ı��,��ʵ�ʱ��С1,���������±�
				 }
				 else
				 {
					 for(i=0;i<25;i++)
					 {
						if(Manul_Save_PointX[i].Flag==0)
						{
							gs_Save_Point_Num = i;							
							break;
						}
				   }
				  }
				 if(gs_Save_Point_Num==25)
				 {
//				 	LCD_Popup_Windows = PW_4_POINTSAVEFULLX;	//������ʾ��-�洢�ռ�����
					gs_Save_Point_Pre_Page=LCD_Current_Page;  //�������������ҳΪLCD��ǰ����
					LCD_Current_Page = 43;					
					g_X_SavePoint_Flag=FALSE;
				 }
				 else
				 {
				 	LCD_Current_Page = 40;
					gs_XSavePoint_Code=gs_Save_Point_Num+1;
					gs_XSavePoint_Code = 0x70302020+((u32)gs_XSavePoint_Code<<16);
					LC_Point_Flag=1;
					g_X_SavePoint_Flag = TRUE;
				 }
				 
				 //����Y���ϲ�����
				 if(Manul_Save_PointL[24].Flag==1)
				 {
					gs_Save_Point_Num = 25;                      //��ǰ�����ı��,��ʵ�ʱ��С1,���������±�
				 }
				 else
				 {
					for(i=0;i<25;i++)
					{
						if(Manul_Save_PointL[i].Flag==0)
						{
							gs_Save_Point_Num = i;  
							break;
						}
				  }
			   }
				 if(gs_Save_Point_Num==25)
				 {
//				 	LCD_Popup_Windows = PW_4_POINTSAVEFULLZ;	 //������ʾ��-�洢�ռ�����
					gs_Save_Point_Pre_Page=LCD_Current_Page;     //�������������ҳΪLCD��ǰ����
					LCD_Current_Page = 43;	
				 	g_L_SavePoint_Flag=FALSE; 
				 }
				 else
				 {
					gs_LSavePoint_Code=gs_Save_Point_Num+1;
					gs_LSavePoint_Code = gs_XSavePoint_Code;
					LC_Point_Flag=1;
				 }
				 break;
			case 0x4401:											//�����ϲ�Z����
				 if(Manul_Save_PointZ[24].Flag==1)
				 {
					gs_Save_Point_Num = 25;                      //��ǰ�����ı��,��ʵ�ʱ��С1,���������±�
				 }
				 else
				 {
					for(i=0;i<25;i++)
					{
						if(Manul_Save_PointZ[i].Flag==0)
						{
							gs_Save_Point_Num = i;  
							break;
						}
				  }
			   }
				 if(gs_Save_Point_Num==25)
				 {
//				 	LCD_Popup_Windows = PW_4_POINTSAVEFULLZ;	 //������ʾ��-�洢�ռ�����
					gs_Save_Point_Pre_Page=LCD_Current_Page;     //�������������ҳΪLCD��ǰ����
					LCD_Current_Page = 43;	
				 	g_Z_SavePoint_Flag=FALSE; 
				 }
				 else
				 {
				 	LCD_Current_Page = 95;
					gs_ZSavePoint_Code=gs_Save_Point_Num+1;
					gs_ZSavePoint_Code = 0x70302020+((u32)gs_ZSavePoint_Code<<16);
					LC_Point_Flag=1;
					g_Z_SavePoint_Flag = TRUE;
				 }	
				break;
			case 0x4403:											//�����ϲ�O����
				 if(Manul_Save_PointZ[24].Flag==1)
				 {
					gs_Save_Point_Num = 25;                      //��ǰ�����ı��,��ʵ�ʱ��С1,���������±�
				 }
				 else
				 {
					for(i=0;i<25;i++)
					{
						if(Manul_Save_PointZ[i].Flag==0)
						{
							gs_Save_Point_Num = i;  
							break;
						}
				  }
			   }
				 if(gs_Save_Point_Num==25)
				 {
//				 	LCD_Popup_Windows = PW_4_POINTSAVEFULLZ;	 //������ʾ��-�洢�ռ�����
					gs_Save_Point_Pre_Page=LCD_Current_Page;     //�������������ҳΪLCD��ǰ����
					LCD_Current_Page = 43;	
				 	g_Z_SavePoint_Flag=FALSE; 
				 }
				 else
				 {
				 	LCD_Current_Page = 95;
					gs_ZSavePoint_Code=gs_Save_Point_Num+1;
					gs_ZSavePoint_Code = 0x70302020+((u32)gs_ZSavePoint_Code<<16);
					LC_Point_Flag=1;
					g_Z_SavePoint_Flag = TRUE;
				 }	
				break;
			case 0x4010://��������  IOѡ�� �ϲ�ѡ��
					LCD_Current_Page= 6;
				break;
			case 0x4F05:
				if(Order_Type!=3)//����ȡ��ָ��
				{
					LCD_Current_Page=4;
				}
				else
				{
					 //�����޸���
					 LCD_Current_Page = 66;
					 LCD_Pre_Page = LCD_Current_Page;
					 PageChange(LCD_Pre_Page);					
					Insert_Order(Order_Type);
					LCD_Current_Page=8;
				 /*******ˢ�����ɱ�̽���**********/
					LCD_Main_Page = FREE_PROGRAMME;
					g_Watch_IO_Control      = 0; 						          //IO��ʵʱ��ع�
					g_Refresh_Dispaly_List_Flag=TRUE;                         //ˢ�³�����ʾ�б�
					g_Robot_Auto_Reset = FALSE;                               //��е���Զ���λ��
					g_Refresh_Dispaly_List_Flag=TRUE; 					
				}
				break;
		  default:
			   break;
		}
	}
	else
	{
		if(LCD_Current_Page == START_UP)	                            //��ҳ��
		{
			if(Robot_Origined == FALSE) 
			{
				LCD_Popup_Windows = PW_1_HOMEFIRST;	                      //������ʾ��-���Ȼ���
			}
			else if(Robot_Status != ACTION_STOP)
			{
			    LCD_Popup_Windows = PW_1_STOPROBOT;	                    //������ʾ��-����ֹͣ��е��
			}
		}
		else if(LCD_Current_Page == 16)	                              //������һҳ
		{
			if(Robot_Origined == FALSE)
			{
				LCD_Popup_Windows = PW_6_HOMEFIRST1;	                    //������ʾ��-���Ȼ���
			}
			else if(Robot_Status != ACTION_STOP)
			{
			    LCD_Popup_Windows = PW_1_STOPROBOT;	                    //������ʾ��-����ֹͣ��е��
			}
		}	
		else if(LCD_Current_Page == 34)	                              //�����ڶ�ҳ
		{
			if(Robot_Origined == FALSE)
			{
				LCD_Popup_Windows = PW_6_HOMEFIRST2;	                    //������ʾ��-���Ȼ���
			}
			else if(Robot_Status != ACTION_STOP)
			{
			    LCD_Popup_Windows = PW_1_STOPROBOT;	                    //������ʾ��-����ֹͣ��е��
			}
		}	
		else if(LCD_Current_Page == SYSTEM_SETTING)	                  //ϵͳ����
		{
			if(Robot_Origined == FALSE)
			{
				LCD_Popup_Windows = PW_4_HOMEFIRST;	                      //������ʾ��-���Ȼ���
			}
			else if(Robot_Status != ACTION_STOP)
			{
			    LCD_Popup_Windows = PW_1_STOPROBOT;	                    //������ʾ��-����ֹͣ��е��
			}
		}
	}

	LCD_Key_Value=0;                                                //��ֵ���� 
	LCD_Var_Data=0;                                                 //������������
}


/*************************************************************************
**  ��������  LCDManulOperateDisply()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    wx   
**  �������ڣ�2015/6/26
**************************************************************************/
void LCDManulOperateDisply()
{
	u8 Mode_String[2][8]={"����","�綯"};
	//�綯����ģʽ��ʾ//
	if(Link_Step_Refresh==1)
	{
		Link_Step_Refresh=0;
		if(Link_Step_Flag==0)
			DisplayChineseString(0x2F01,Mode_String[0],8); 
		else
			DisplayChineseString(0x2F01,Mode_String[1],8);
	}
	//--�ֶ�����-�洢������--//
	if(gs_Save_Point_Pre_Page!=0 && gs_Save_Point_Pre_Page==LCD_Pre_Page)
	{
		if(gs_Save_Point_Pre_Page==14 || gs_Save_Point_Pre_Page==98)
		{
			Display_PageCut(PageCut_SavePointFullX);                    //��ʾX��洢������
		}
		else if(gs_Save_Point_Pre_Page==91 || gs_Save_Point_Pre_Page==99)
		{
			Display_PageCut(PageCut_SavePointFullZ);                    //��ʾZ��洢������
		}
		else if(gs_Save_Point_Pre_Page==67 || gs_Save_Point_Pre_Page==113)
		{
			Display_PageCut(PageCut_SavePointFullL);                    //��ʾL��洢������
		}
		else if(gs_Save_Point_Pre_Page==69 || gs_Save_Point_Pre_Page==121)
		{
			Display_PageCut(PageCut_SavePointFullO);                    //��ʾO��洢������
		}		
	}
	//--�����л�--//
	if(LCD_Pre_Page != LCD_Current_Page )	   
	{
		LCD_Pre_Page = LCD_Current_Page;
		PageChange(LCD_Pre_Page);
	}
	//--Xλ��ˢ��--//
	if(gs_Manul_XAxis_Position != g_XAxis_Position)	 
	{
		gs_Manul_XAxis_Position = g_XAxis_Position;
		WriteNum4(0x101A,gs_Manul_XAxis_Position);
		WriteNum4(0x4800,gs_Manul_XAxis_Position);// /JXS_Parameter.GearRatio.Ratio[0]  	  
	}
	//--Zλ��ˢ��--//
	if(gs_Manul_ZAxis_Position != g_ZAxis_Position)	 
	{
		gs_Manul_ZAxis_Position = g_ZAxis_Position;
		WriteNum4(0x101C,gs_Manul_ZAxis_Position);
		WriteNum4(0x480F,gs_Manul_ZAxis_Position);//	/JXS_Parameter.GearRatio.Ratio[1]	
	}
	//--Lλ��ˢ��--//
	if(gs_Manul_LAxis_Position != g_LAxis_Position)	 
	{
		gs_Manul_LAxis_Position = g_LAxis_Position;
		WriteNum4(0x101E,gs_Manul_LAxis_Position);	
		WriteNum4(0x4805,gs_Manul_LAxis_Position);//   /JXS_Parameter.GearRatio.Ratio[2]		
	}
	//--Oλ��ˢ��--//
	if(gs_Manul_OAxis_Position != g_OAxis_Position)	 
	{
		gs_Manul_OAxis_Position = g_OAxis_Position;
		WriteNum4(0x1018,gs_Manul_OAxis_Position);	
		WriteNum4(0x480A,gs_Manul_OAxis_Position);//   	/JXS_Parameter.GearRatio.Ratio[3]	
	}
	//--Vλ��ˢ��--//
	if(gs_Manul_AAxis_Position != g_AAxis_Position)	 
	{
		gs_Manul_AAxis_Position = g_AAxis_Position;
		WriteNum4(0x4735,gs_Manul_AAxis_Position);
		WriteNum4(0x4815,gs_Manul_AAxis_Position);//   /JXS_Parameter.GearRatio.Ratio[4]		
	}
	//--Wλ��ˢ��--//
	if(gs_Manul_BAxis_Position != g_BAxis_Position)	 
	{
		gs_Manul_BAxis_Position = g_BAxis_Position;
		WriteNum4(0x4EF0,gs_Manul_BAxis_Position);	
		WriteNum4(0x481A,gs_Manul_BAxis_Position);//   /JXS_Parameter.GearRatio.Ratio[5]				
	}
	//�ٶ�-����ˢ��
	if(gs_Refresh_Speed_Para)
	{
		WriteNum2(0x4004,gs_LinkMove_Speed[X_Axsis]);                        //X�ֶ�����-�����ٶ�
		WriteNum2(0x4005,gs_XStepMove_Distance);                     //X�ֶ�����-�綯����
		WriteNum2(0x4014,gs_LinkMove_Speed[Z_Axsis]);                        //Z�ֶ�����-�����ٶ�
		WriteNum2(0x4015,gs_ZStepMove_Distance);                     //Z�ֶ�����-�綯����	
		WriteNum2(0x4060,gs_LinkMove_Speed[L_Axsis]);                        //L�ֶ�����-�����ٶ�
		WriteNum2(0x4061,gs_LStepMove_Distance);                     //L�ֶ�����-�綯����
		WriteNum2(0x4070,gs_LinkMove_Speed[O_Axsis]);                        //O�ֶ�����-�����ٶ�
		WriteNum2(0x4071,gs_OStepMove_Distance);                     //O�ֶ�����-�綯����
		WriteNum2(0x4710,gs_LinkMove_Speed[A_Axsis]);                        //V�ֶ�����-�����ٶ�
		WriteNum2(0x4713,gs_AStepMove_Distance);                     //V�ֶ�����-�綯����
		WriteNum2(0x4745,gs_LinkMove_Speed[B_Axsis]);                        //W�ֶ�����-�����ٶ�
		WriteNum2(0x4747,gs_BStepMove_Distance);                     //W�ֶ�����-�綯����
		gs_Refresh_Speed_Para = FALSE;
	}
  //--X��-����--//
	if(g_X_SavePoint_Flag)			   
	{
		Temp_Display_Data[0]=gs_XSavePoint_Code>>24;
		Temp_Display_Data[1]=gs_XSavePoint_Code>>16;
		Temp_Display_Data[2]=gs_XSavePoint_Code>>8;
		Temp_Display_Data[3]=gs_XSavePoint_Code;
		DisplayChineseString(0x4040,Temp_Display_Data,4);
//		WriteNum4(0x4040,gs_XSavePoint_Code);
		g_X_SavePoint_Flag=FALSE;
	}
  //--Z��-����--//
	if(g_Z_SavePoint_Flag)			   
	{
		Temp_Display_Data[0]=gs_ZSavePoint_Code>>24;
		Temp_Display_Data[1]=gs_ZSavePoint_Code>>16;
		Temp_Display_Data[2]=gs_ZSavePoint_Code>>8;
		Temp_Display_Data[3]=gs_ZSavePoint_Code;
		DisplayChineseString(0x4040,Temp_Display_Data,4);
//		WriteNum4(0x4040,gs_ZSavePoint_Code);
		g_Z_SavePoint_Flag=FALSE;
	}
	
	//--����-����--//
	if(g_Programma_Flag)
	{
		WriteNum4(0x4610,Point_Num);
		g_Programma_Flag=FALSE;
	}
	//�ֶ�����-X��������
	if((LCD_Current_Page==96)&&(g_XAxis_Position==0))
	{
		if(Program_Or_Point==2)
			LCD_Current_Page=14;
		else
			LCD_Current_Page=148;			
	}
	//�ֶ�����-Z��������
	if((LCD_Current_Page==97)&&(g_ZAxis_Position==0))
	{
		if(Program_Or_Point==2)
			LCD_Current_Page=91;
		else
			LCD_Current_Page=151;		
	}
	//�ֶ�����-L��������
	if((LCD_Current_Page==111)&&(g_LAxis_Position==0))
	{
		if(Program_Or_Point==2)
			LCD_Current_Page=67;
		else
			LCD_Current_Page=149;		
	}
	//�ֶ�����-O��������
	if((LCD_Current_Page==72)&&(g_OAxis_Position==0))
	{
		if(Program_Or_Point==2)
			LCD_Current_Page=69;
		else
			LCD_Current_Page=150;		
	}
	//�ֶ�����-V��������
	if((LCD_Current_Page==156)&&(g_OAxis_Position==0))
	{
		if(Program_Or_Point==2)
			LCD_Current_Page=68;
		else
			LCD_Current_Page=152;		
	}
	//�ֶ�����-W��������
	if((LCD_Current_Page==157)&&(g_OAxis_Position==0))
	{
		if(Program_Or_Point==2)
			LCD_Current_Page=70;
		else
			LCD_Current_Page=153;		
	}
	//��������-X��������
	if((LCD_Current_Page==101)&&(g_XAxis_Position==0))
	{
 		LCD_Current_Page=98;
	}
	//��������-Z��������
	if((LCD_Current_Page==102)&&(g_ZAxis_Position==0))
	{
		LCD_Current_Page=99;
	}
	//��������-L��������
	if((LCD_Current_Page==112)&&(g_LAxis_Position==0))
	{
		LCD_Current_Page=113;
	}
	//��������-O��������
	if((LCD_Current_Page==122)&&(g_OAxis_Position==0))
	{
		LCD_Current_Page=121;
	}
	//��������-W��������
	if((LCD_Current_Page==156)&&(g_BAxis_Position==0))
	{
		LCD_Current_Page=153;
	}
	//��������-V��������
	if((LCD_Current_Page==157)&&(g_AAxis_Position==0))
	{
		LCD_Current_Page=152;
	}
	//X����-����ͼ��
	if((LCD_Current_Page==40)&&(g_FreeProgram_Setting_XZ==TRUE)&&(gs_FreeProgram_Setting_XZ==FALSE))
	{
		Display_PageCut(PageCut_XParameter_Setting);
		UnHiddenDisplay(0x4E00,0x4120);
		gs_FreeProgram_Setting_XZ=g_FreeProgram_Setting_XZ;
	}

	//Z����-����ͼ��
	if((LCD_Current_Page == 95)&&(g_FreeProgram_Setting_XZ==TRUE)&&(gs_FreeProgram_Setting_XZ==FALSE))
	{
		Display_PageCut(PageCut_ZParameter_Setting);
		UnHiddenDisplay(0x4E01,0x4130);
		gs_FreeProgram_Setting_XZ=g_FreeProgram_Setting_XZ;
	}
	//L����-����ͼ��
	if((LCD_Current_Page == 114)&&(g_FreeProgram_Setting_XZ==TRUE)&&(gs_FreeProgram_Setting_XZ==FALSE))
	{
		Display_PageCut(PageCut_LParameter_Setting);
		UnHiddenDisplay(0x4E02,0x4140);
		gs_FreeProgram_Setting_XZ=g_FreeProgram_Setting_XZ;
	}
	//O����-����ͼ��
	if((LCD_Current_Page == 71)&&(g_FreeProgram_Setting_XZ==TRUE)&&(gs_FreeProgram_Setting_XZ==FALSE))
	{
		Display_PageCut(PageCut_OParameter_Setting);
		UnHiddenDisplay(0x4E03,0x4150);
		gs_FreeProgram_Setting_XZ=g_FreeProgram_Setting_XZ;
	}
	//�洢λ������-����ͼ��
	if((LCD_Current_Page==43)&&(g_FreeProgram_Setting_XZ==TRUE)&&(gs_FreeProgram_Setting_XZ==FALSE))
	{
		if(gs_Save_Point_Pre_Page==98)
		{
			Display_PageCut(PageCut_XParameter_Setting);
			UnHiddenDisplay(0x4E00,0x4120);
		}
		if(gs_Save_Point_Pre_Page==99)
		{
			Display_PageCut(PageCut_ZParameter_Setting);
			UnHiddenDisplay(0x4E01,0x4130);
		}
		if(gs_Save_Point_Pre_Page==113)
		{
			Display_PageCut(PageCut_LParameter_Setting);
			UnHiddenDisplay(0x4E02,0x4140);
		}
		if(gs_Save_Point_Pre_Page==121)
		{
			Display_PageCut(PageCut_OParameter_Setting);
			UnHiddenDisplay(0x4E03,0x4150);
		}
		gs_FreeProgram_Setting_XZ=g_FreeProgram_Setting_XZ;
	}
	//--�ֶ�����-΢��--//
	if(gs_Current_Manul_Axis != g_Current_Manul_Axis)	 
	{
		DisplayChineseString(0x4050,gs_Axis_Choose[g_Current_Manul_Axis-1],4);
		gs_Current_Manul_Axis=g_Current_Manul_Axis;		
	}
	
	//--ʾ���ֶ�����ģʽ---//
	if((LCD_Current_Page==4||LCD_Current_Page==9||LCD_Current_Page==10||LCD_Current_Page==0x94)&&gs_Mode_Refresh_Flag==TRUE)
	{
		gs_Mode_Refresh_Flag=FALSE;
		DisplayChineseString(0x1EEF,Program_String[Run_Mode_Flag+4],16);	
	}
	//---�ϲ�IO��������---//
		if(LCD_Current_Page==6&&Parameter_Refresh)
		{
			Parameter_Refresh=FALSE;		
			//�ϲ�ѡ��
			if(LC_ID==0)
			{
				Display_PageCut(LC1_N);	
				Display_PageCut(LC2_N);	
				Display_PageCut(LC3_N);	
				Display_PageCut(LC4_N);	
				Display_PageCut(LC5_N);	
				Display_PageCut(LC6_N);	
				Display_PageCut(LC7_N);	
				Display_PageCut(LC8_N);	
				Display_PageCut(LC9_N);	
				Display_PageCut(LC10_N);	
			}
			if(LC_ID==1)
			Display_PageCut(LC1_Y);				
			else
			Display_PageCut(LC1_N);
			if(LC_ID==2)
			Display_PageCut(LC2_Y);				
			else
			Display_PageCut(LC2_N);
			if(LC_ID==3)
			Display_PageCut(LC3_Y);				
			else
			Display_PageCut(LC3_N);	
			if(LC_ID==4)
			Display_PageCut(LC4_Y);				
			else
			Display_PageCut(LC4_N);
			if(LC_ID==5)
			Display_PageCut(LC5_Y);				
			else
			Display_PageCut(LC5_N);
			if(LC_ID==6)
			Display_PageCut(LC6_Y);				
			else			
			Display_PageCut(LC6_N);
			if(LC_ID==7)
			Display_PageCut(LC7_Y);				
			else
			Display_PageCut(LC7_N);	
			if(LC_ID==8)
			Display_PageCut(LC8_Y);				
			else
			Display_PageCut(LC8_N);
			if(LC_ID==9)
			Display_PageCut(LC9_Y);				
			else
			Display_PageCut(LC9_N);	
			if(LC_ID==10)
			Display_PageCut(LC10_Y);				
			else
			Display_PageCut(LC10_N);	
			//IO��ѡ��
			if(ChuJiao1)
			Display_PageCut(ChuJiao1_Y);
			else
			Display_PageCut(ChuJiao1_N);
			if(ChuJiao2)
			Display_PageCut(ChuJiao2_Y);
			else
			Display_PageCut(ChuJiao2_N);
			if(ChuJiao3)
			Display_PageCut(ChuJiao3_Y);
			else
			Display_PageCut(ChuJiao3_N);
			if(JiaoTouXiaYi1)
			Display_PageCut(JiaoTouXiaYi1_Y);
			else
			Display_PageCut(JiaoTouXiaYi1_N);
			if(JiaoTouXiaYi2)
			Display_PageCut(JiaoTouXiaYi2_Y);
			else
			Display_PageCut(JiaoTouXiaYi2_N);
			if(JiaoTouXiaYi3)
			Display_PageCut(JiaoTouXiaYi3_Y);
			else
			Display_PageCut(JiaoTouXiaYi3_N);
			
			
			if(XiZuan1)
			Display_PageCut(XiZuan1_Y);
			else
			Display_PageCut(XiZuan1_N);
			if(XiZuan2)
			Display_PageCut(XiZuan2_Y);
			else
			Display_PageCut(XiZuan2_N);
			if(XiZuan3)
			Display_PageCut(XiZuan3_Y);
			else
			Display_PageCut(XiZuan3_N);
			if(XiZuan4)
			Display_PageCut(XiZuan4_Y);
			else
			Display_PageCut(XiZuan4_N);
			if(XiZuan5)
			Display_PageCut(XiZuan5_Y);
			else
			Display_PageCut(XiZuan5_N);
			if(XiZuan6)
			Display_PageCut(XiZuan6_Y);
			else
			Display_PageCut(XiZuan6_N);
			if(XiZuan7)
			Display_PageCut(XiZuan7_Y);
			else
			Display_PageCut(XiZuan7_N);
			if(XiZuan8)
			Display_PageCut(XiZuan8_Y);
			else
			Display_PageCut(XiZuan8_N);
			
			
			if(ZuanTouXiaYi1)
			Display_PageCut(ZuanTouXiaYi1_Y);
			else
			Display_PageCut(ZuanTouXiaYi1_N);
			if(ZuanTouXiaYi2)
			Display_PageCut(ZuanTouXiaYi2_Y);
			else
			Display_PageCut(ZuanTouXiaYi2_N);
			if(ZuanTouXiaYi3)
			Display_PageCut(ZuanTouXiaYi3_Y);
			else
			Display_PageCut(ZuanTouXiaYi3_N);
			if(ZuanTouXiaYi4)
			Display_PageCut(ZuanTouXiaYi4_Y);
			else
			Display_PageCut(ZuanTouXiaYi4_N);
			if(ZuanTouXiaYi5)
			Display_PageCut(ZuanTouXiaYi5_Y);
			else
			Display_PageCut(ZuanTouXiaYi5_N);
			if(ZuanTouXiaYi6)
			Display_PageCut(ZuanTouXiaYi6_Y);
			else
			Display_PageCut(ZuanTouXiaYi6_N);
			if(ZuanTouXiaYi7)
			Display_PageCut(ZuanTouXiaYi7_Y);
			else
			Display_PageCut(ZuanTouXiaYi7_N);
			if(ZuanTouXiaYi8)
			Display_PageCut(ZuanTouXiaYi8_Y);
			else
			Display_PageCut(ZuanTouXiaYi8_N);
		}
}

/*************************************************************************
**  ��������  Is_Leap_Year()
**	���������
**	�����������
**	�������ܣ��ж��Ƿ�Ϊ����
**	��ע��	  ��
**  ���ߣ�    White_L   
**  �������ڣ�2018/7/17
**************************************************************************/
u8 Is_Leap_Year(u16 year)
{			  
	if(year%4==0) //�����ܱ�4����
	{ 
		if(year%100==0) 
		{ 
			if(year%400==0)return 1;//�����00��β,��Ҫ�ܱ�400���� 	   
			else return 0;   
		}else return 1;   
	}else return 0;	
}	
/*************************************************************************
**  ��������  Program_Save_Time()
**	���������
**	�����������
**	�������ܣ�����ʱ���
**	��ע��	  ��
**  ���ߣ�    White_L   
**  �������ڣ�2018/7/17
**************************************************************************/
u8 Program_Save_Time(void)
{
	u16 syear=0;
	u8 smon=0,sday=0,hour=0,min=0,sec=0;
	u16 t=0;
	const u8 mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};
	
		seccount=0;
		syear=2000+(LCD_RTC_Day>>24)/16*10+(LCD_RTC_Day>>24)%16;//���Բ�ȡǿ��ת��
		smon=((LCD_RTC_Day>>16)&0xFF)/16*10+(LCD_RTC_Day>>16)%16;
		sday=((LCD_RTC_Day>>8)&0xFF)/16*10+(LCD_RTC_Day>>8)%16;
		hour=(LCD_RTC_Time>>24)/16*10+(LCD_RTC_Time>>24)%16;
		min=(LCD_RTC_Time>>16)/16*10+(LCD_RTC_Time>>16)%16;
		sec=((LCD_RTC_Time>>8)&0xFF)/16*10+((LCD_RTC_Time>>8)&0xFF)%16;

		if(syear<1970||syear>2099)return 1;	   
		for(t=1970;t<syear;t++)	//��������ݵ��������
		{
			if(Is_Leap_Year(t))seccount+=31622400;//�����������
			else seccount+=31536000;			  //ƽ���������
		}
		for(t=0;t<smon-1;t++)	   //��ǰ���·ݵ����������
		{
			seccount+=(u32)mon_table[t]*86400;//�·����������
			if(Is_Leap_Year(syear)&&t==1)seccount+=86400;//����2�·�����һ���������	   
		}
		seccount+=(u32)(sday-1)*86400;//��ǰ�����ڵ���������� 
		seccount+=(u32)hour*3600;//Сʱ������
		seccount+=(u32)min*60;	 //����������
		seccount=seccount+sec-8*3600;//�������Ӽ���ȥ
		return 0;
}
/*************************************************************************
**  ��������  Program_Save()
**	���������
**	�����������
**	�������ܣ��������
**	��ע��	  ��
**  ���ߣ�    White_L   
**  �������ڣ�2018/7/17
**************************************************************************/
extern u8 Program_Edit_OrNot;
void Program_Save(void)
{
	u8 i=0,j=0,temp[60];
	if(Program_Save_OrNot==1&&Program_Edit_OrNot==1)//�����޸ı���
	{
			Program_Edit_OrNot=0;
 /**************��༭���汣�����**************************/
		  LCD_New_Order=FALSE;
		  ReadRTC();	
		  DelayNms(10);
			temp[0] =PROGRAM_INFO;
			temp[1] =1;//flag
			temp[2] =Free_Program_Operate.Code;
			temp[3] =Free_Program_Operate.Name;
			temp[4] =Free_Program_Operate.Name>>8;
			temp[5] =Free_Program_Operate.Name>>16;
			temp[6] =Free_Program_Operate.Name>>24;
			temp[7] =Free_Program_Operate.Num;
			temp[8]	=Free_Program_Operate.Num>>8;
			temp[9] =seccount;
			temp[10] =seccount>>8;
			temp[11] =seccount>>16;
			temp[12] =seccount>>24;
			temp[13] =Point_Num;
			temp[14] =Point_Num>>8;
			AT24CXX_Write(0x1100+(g_Run_Program_Num-1)*14,&temp[1],14);	
			
			OperateProgram[Current_Select_Program].Flag = 1;
			OperateProgram[Current_Select_Program].Order_Num  = Free_Program_Operate.Num;
			OperateProgram[Current_Select_Program].time = seccount;
			Saved_Program[Current_Select_Program].Order_Num =Free_Program_Operate.Num;
			Saved_Program[Current_Select_Program].Time      =seccount;
			USART2_SendDataArray(15,FREE_PROGRAM_SEND,temp);  //�ȷ������������Ϣ����
			DelayNms(1000);
			seccount=0;
			Order_Num=0;		
			if(Free_Program_Operate.Num!=0)
			{
				IIC_Temp[0] = 0xED;
				IIC_Temp[1]	= 0xAA;
				IIC_Temp[2] = g_Run_Program_Num;
				AT24CXX_Read(CURRENT_PROGRAM_ADR2+ORDER_LENTH*Order_Num,&IIC_Temp[3],ORDER_LENTH);
				AT24CXX_Write(CURRENT_PROGRAM_ADR1+ORDER_LENTH*Order_Num,&IIC_Temp[3],ORDER_LENTH);
				Free_Program_Operate.Program[Order_Num].Flag  = 1;
				Free_Program_Operate.Program[Order_Num].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
				Free_Program_Operate.Program[Order_Num].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
				Free_Program_Operate.Program[Order_Num].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
				Free_Program_Operate.Program[Order_Num].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
				Free_Program_Operate.Program[Order_Num].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
				Free_Program_Operate.Program[Order_Num].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
				Free_Program_Operate.Program[Order_Num].LC_ID	= IIC_Temp[25];	
				Free_Program_Operate.Program[Order_Num].Key   = IIC_Temp[26];
					if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 �㽺 2���� 3ȡ�� 4 ��ʱ 5����
					 {
						Free_Program_Operate.Program[Order_Num].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
					 }
					else if(4==IIC_Temp[26])//��ʱָ��
					{
						Free_Program_Operate.Program[Order_Num].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
					}
				USART2_SendDataArray(ORDER_LENTH+3,FREE_PROGRAM_SEND,IIC_Temp);     //��ȡ��Ӧ�������
				USART1RecieveFinishedFlag=FALSE;
				Program_Transport_Flag=1;//��ʱ����1��û�л�Ӧ��Ӧ�쳣
				while(Order_Num<Free_Program_Operate.Num)
				{
					if(USART1RecieveFinishedFlag==TRUE)	   //�����������
					{
						Program_Transport_Count=0;
						 /*------------------------*/
						USART1RecieveDataDecode();		   //����Ӧ�����ݺ���
						/*------------------------*/
					}		
					if(Program_Transport_Count>200)
					{
						Program_Transport_Flag=0;
						break;//��ʱ����1��û�л�Ӧ����Ӧ�쳣
					}
				}
				Program_Transport_Flag=0;				
				if(Order_Num<LARGESTPROGRAMNUM)
				{
					for(j=Order_Num;j<LARGESTPROGRAMNUM;j++)
					{
						Free_Program_Operate.Program[j].List  = 0;
						Free_Program_Operate.Program[j].Value1= 0;	
						Free_Program_Operate.Program[j].Value2= 0;
						Free_Program_Operate.Program[j].Value3= 0;
						Free_Program_Operate.Program[j].Value4= 0;	
						Free_Program_Operate.Program[j].Value5= 0;							
						Free_Program_Operate.Program[j].LC_ID = 0;
						Free_Program_Operate.Program[j].Key   = 0;
						Free_Program_Operate.Program[j].IO_Group= 0;		
					}
				}
			}
			else//û��ʾ�̵�
			{
				for(i=0;i<LARGESTPROGRAMNUM;i++)
				{
					Free_Program_Operate.Program[i].Flag  = 0;
					Free_Program_Operate.Program[i].Key   = 0;
					Free_Program_Operate.Program[i].LC_ID = 0;
					Free_Program_Operate.Program[i].Value1= 0;
					Free_Program_Operate.Program[i].Value2= 0;
					Free_Program_Operate.Program[i].Value3= 0;
					Free_Program_Operate.Program[i].Value4= 0;	
					Free_Program_Operate.Program[j].Value5= 0;						
				}				
			}
			Program_Save_OrNot=0;
			/*---------------����༭����-------------------*/
			 g_Background_Color_ProgramEdit = 1;			      //�༭����ɫ
			 gs_Background_Color_ProgramEdit= 0;
			 gs_Current_Operate_List_ProgramEdit=1;				//��ѡ�������ɫ�������ڵĵ���
		}
		else if(Program_Save_OrNot==2&&Program_Edit_OrNot==1)//�����޸Ĳ����淵��
		{
			Program_Edit_OrNot=0;
			OperateProgram[gs_Current_Operate_List-1].Order_Num = Saved_Program[g_Run_Program_Num-1].Order_Num;
			Order_Num									= Saved_Program[g_Run_Program_Num-1].Order_Num;
			Free_Program_Operate.Num  = Saved_Program[g_Run_Program_Num-1].Order_Num;
				if(Free_Program_Operate.Num<LARGESTPROGRAMNUM)
					j=Free_Program_Operate.Num;
				else
					j=LARGESTPROGRAMNUM;					
			 /*List Value1 2 3 4  5 LC_ID Key IO_Group Delay_Time*/
				for(i=0;i<j;i++)//��ȡ֮ǰ��������Ϣ
				{
					AT24CXX_Read(CURRENT_PROGRAM_ADR1+ORDER_LENTH*i,&IIC_Temp[3],ORDER_LENTH);
					AT24CXX_Write(CURRENT_PROGRAM_ADR2+ORDER_LENTH*i,&IIC_Temp[3],ORDER_LENTH);
					Free_Program_Operate.Program[i].Flag  = 1;
					Free_Program_Operate.Program[i].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
					Free_Program_Operate.Program[i].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
					Free_Program_Operate.Program[i].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
					Free_Program_Operate.Program[i].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
					Free_Program_Operate.Program[i].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
					Free_Program_Operate.Program[i].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
					Free_Program_Operate.Program[i].LC_ID	= IIC_Temp[25];	
					Free_Program_Operate.Program[i].Key   = IIC_Temp[26];
					 if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 �㽺 2���� 3ȡ�� 4 ��ʱ 5����
					 {
						Free_Program_Operate.Program[i].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
					 }
					else if(4==IIC_Temp[26])//��ʱָ��
					{
						Free_Program_Operate.Program[i].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
					}
				}
				if(i<LARGESTPROGRAMNUM)
				{
					for(j=i;j<LARGESTPROGRAMNUM;j++)
					{
						Free_Program_Operate.Program[j].Flag  = 0;
						Free_Program_Operate.Program[j].List  = 0;
						Free_Program_Operate.Program[j].Key   = 0;
						Free_Program_Operate.Program[j].Value1= 0;	
						Free_Program_Operate.Program[j].Value2= 0;
						Free_Program_Operate.Program[j].Value3= 0;
						Free_Program_Operate.Program[j].Value4= 0;
						Free_Program_Operate.Program[j].Value5= 0;
						Free_Program_Operate.Program[i].LC_ID	= 0;	
					}
				}
			Program_Save_OrNot=0;
			/*---------------����༭����-------------------*/
			g_Background_Color_ProgramEdit = 1;			      //�༭����ɫ
			gs_Background_Color_ProgramEdit= 0;
			gs_Current_Operate_List_ProgramEdit=1;				//��ѡ�������ɫ�������ڵĵ���		
		}
			LCD_Current_Page = 2;
			Program_Or_Point=1;
			g_Refresh_Dispaly_List_Flag=TRUE;
			Select_Program_Flag    = TRUE;
}
/*************************************************************************
**  ��������  Order_Check()
**	���������
**	�����������
**	�������ܣ�ָ���
**	��ע��	  ��
**  ���ߣ�    White_L   
**  �������ڣ�2018/1/11
**************************************************************************/
extern u16 Delay_Time;
//IO_Choose1�ӵ�λ����λ����  ����1 2 3 ��ͷ���� 1 2 3 ����1 2 3 4 5 6 7 8 ��ͷ����1 2 3 4 5 6 7 8
//0x9  12 
u8 Order_Check( u8 Order_Type )
{
	 if(Order_Type==1||Order_Type==2||Order_Type==3)
	 {
		if(LC_ID>=0&&LC_ID<10)
		{
//		 if(((IO_Choose1&&0x0001)&&(IO_Choose1&&0x0008))||((IO_Choose1&&0x0002)&&(IO_Choose1&&0x0010))
//		 ||((IO_Choose1&&0x0004)&&(IO_Choose1&&0x0020))||((IO_Choose1&&0x0040)&&(IO_Choose1&&0x4000))
//		 ||((IO_Choose1&&0x0080)&&(IO_Choose1&&0x8000))||((IO_Choose1&&0x0100)&&(IO_Choose1&&0x00010000))
//		 ||((IO_Choose1&&0x0200)&&(IO_Choose1&&0x00020000))||((IO_Choose1&&0x0400)&&(IO_Choose1&&0x00040000))
//		 ||((IO_Choose1&&0x0800)&&(IO_Choose1&&0x00080000))||((IO_Choose1&&0x1000)&&(IO_Choose1&&0x00100000))
//		 ||((IO_Choose1&&0x2000)&&(IO_Choose1&&0x00200000)))//ָ����ȷ���ܲ���ָ��
			return 1;
		}
		else
			return 0;
	 }
	 else if(Order_Type==4)			//ָ����ȷ���ܲ���ָ��
	 {
		if(Delay_Time>0&&Delay_Time<10000&&LC_ID>=0&&LC_ID<10)
			return 1;
		else 
			return 0;
	 }
}

/*************************************************************************
**  ��������  Insert_Order()
**	���������
**	�����������
**	�������ܣ�����ָ��
**	��ע��	  ��
**  ���ߣ�    White_L   
**  �������ڣ�2018/10/11
**************************************************************************/
extern u8 LC_ID;
extern u16 Delay_Time;
void Insert_Order( u8 Order_Type )
{
	u16 i=0,j=0,n=0,k=0;
	u32 io_flag=0;
	if(Point_Action_Flag==0)
	{ 
		Program_Edit_OrNot=1;
		if(Free_Program_Operate.Num!=0)
			n = gs_Current_Operate_List_ProgramEdit+1;//��¼Ҫ����ָ���λ��
		else
			n=1;
	 //����������
	 LCD_Current_Page = 66;
	 LCD_Pre_Page = LCD_Current_Page;
	 PageChange(LCD_Pre_Page);
		for(i=Order_Num;i>n-1;i--)//����ָ�����
		{
			AT24CXX_Read(CURRENT_PROGRAM_ADR2+ORDER_LENTH*(i-1),IIC_Temp,ORDER_LENTH);
			IIC_Temp[0]=i+1;
			IIC_Temp[1]=(i+1)>>8;
			AT24CXX_Write(CURRENT_PROGRAM_ADR2+ORDER_LENTH*i,IIC_Temp,ORDER_LENTH);			
		}
		 /*List Value1 2 3 4  5 LC_ID Key IO_Group Delay_Time*/
		if(Order_Type==1||Order_Type==2||Order_Type==5)//�㽺 ���� ����ָ��
		{
		 IIC_Temp[0]=n;
		 IIC_Temp[1]=n>>8;
		 IIC_Temp[2] = gs_Manul_XAxis_Position;//X������
		 IIC_Temp[3] = gs_Manul_XAxis_Position>>8;
		 IIC_Temp[4] = gs_Manul_XAxis_Position>>16;
		 IIC_Temp[5] = (gs_Manul_XAxis_Position>>24)|0xF0;
		 IIC_Temp[6] = gs_Manul_LAxis_Position;//Y������
		 IIC_Temp[7] = gs_Manul_LAxis_Position>>8;
		 IIC_Temp[8] = gs_Manul_LAxis_Position>>16;
		 IIC_Temp[9] = (gs_Manul_ZAxis_Position>>24)|0xF0;
		 IIC_Temp[10] = gs_Manul_ZAxis_Position;//Z������
		 IIC_Temp[11] = gs_Manul_ZAxis_Position>>8;
		 IIC_Temp[12] = gs_Manul_ZAxis_Position>>16;
		 IIC_Temp[13] = (gs_Manul_ZAxis_Position>>24)|0xF0;
		 IIC_Temp[14] = gs_Manul_AAxis_Position;//V������
		 IIC_Temp[15] = gs_Manul_AAxis_Position>>8;
		 IIC_Temp[16] = gs_Manul_AAxis_Position>>16;
		 IIC_Temp[17] = (gs_Manul_AAxis_Position>>24)|0xF0;
		 IIC_Temp[18] = gs_Manul_BAxis_Position;//W������
		 IIC_Temp[19] = gs_Manul_BAxis_Position>>8;
		 IIC_Temp[20] = gs_Manul_BAxis_Position>>16;
		 IIC_Temp[21] = (gs_Manul_BAxis_Position>>24)|0xF0;		
		 IIC_Temp[22] = LC_ID;
		 IIC_Temp[23] = Order_Type;
		 IIC_Temp[24] = IO_Choose;
		 IIC_Temp[25] = IO_Choose>>8;
		 IIC_Temp[26] = IO_Choose>>16;	
		 IIC_Temp[27] = IO_Choose>>24;	
	 }
		else if(Order_Type==3)
		{	
		 IIC_Temp[0]=n;
		 IIC_Temp[1]=n>>8;
		 IIC_Temp[2] = 0;//X������
		 IIC_Temp[3] = 0;
		 IIC_Temp[4] = 0;
		 IIC_Temp[5] = 0;
		 IIC_Temp[6] = 0;//Y������
		 IIC_Temp[7] = 0;
		 IIC_Temp[8] = 0;
		 IIC_Temp[9] = 0;
		 IIC_Temp[10] = 0;//Z������
		 IIC_Temp[11] = 0;
		 IIC_Temp[12] = 0;
		 IIC_Temp[13] = 0;
		 IIC_Temp[14] = 0;//V������
		 IIC_Temp[15] = 0;
		 IIC_Temp[16] = 0;
		 IIC_Temp[17] = 0;
		 IIC_Temp[18] = 0;//W������
		 IIC_Temp[19] = 0;
		 IIC_Temp[20] = 0;
		 IIC_Temp[21] = 0;		
		 IIC_Temp[22] = LC_ID;
		 IIC_Temp[23] = Order_Type;
		 IIC_Temp[24] = IO_Choose;
		 IIC_Temp[25] = IO_Choose>>8;		
		 IIC_Temp[26] = IO_Choose>>16;	
		 IIC_Temp[27] = IO_Choose>>24;						
		}
		else if(Order_Type==4)//��ʱָ��
		{
		 IIC_Temp[0]=n;
		 IIC_Temp[1]=n>>8;
		 IIC_Temp[2] = 0;//X������
		 IIC_Temp[3] = 0;
		 IIC_Temp[4] = 0;
		 IIC_Temp[5] = 0;
		 IIC_Temp[6] = 0;//Y������
		 IIC_Temp[7] = 0;
		 IIC_Temp[8] = 0;
		 IIC_Temp[9] = 0;
		 IIC_Temp[10] = 0;//Z������
		 IIC_Temp[11] = 0;
		 IIC_Temp[12] = 0;
		 IIC_Temp[13] = 0;
		 IIC_Temp[14] = 0;//V������
		 IIC_Temp[15] = 0;
		 IIC_Temp[16] = 0;
		 IIC_Temp[17] = 0;
		 IIC_Temp[18] = 0;//W������
		 IIC_Temp[19] = 0;
		 IIC_Temp[20] = 0;
		 IIC_Temp[21] = 0;		
		 IIC_Temp[22] = 0;
		 IIC_Temp[23] = Order_Type;
		 IIC_Temp[24] = 0;
		 IIC_Temp[25] = 0;		
		 IIC_Temp[26] = Delay_Time;	
		 IIC_Temp[27] = Delay_Time>>8;				 
		}
		AT24CXX_Write(CURRENT_PROGRAM_ADR2+ORDER_LENTH*(n-1),IIC_Temp,ORDER_LENTH);	//����ָ��
		/*�жϲ���ָ��λ�����䣬������Ӧ����ĵ�ָ��*/
		j = ((n-1)/LARGESTPROGRAMNUM)*LARGESTPROGRAMNUM;
		if(Order_Num-j+1>LARGESTPROGRAMNUM)
			n=j+LARGESTPROGRAMNUM;
		else
			n=Order_Num+1;
		 Free_Program_Operate.Num++;												   //ָ����Ŀ��һ
		 OperateProgram[gs_Current_Operate_List-1].Order_Num++;//ָ����Ŀ��һ
		 Order_Num++;																					 //ָ����Ŀ��һ
		 Program_Edit_OrNot=1;																 //�����޸�
		 LCD_Current_Page = 8;
	   if(n!=1&&Free_Program_Operate.Program[(gs_Current_Operate_List_ProgramEdit-1)%LARGESTPROGRAMNUM].Flag!=0)
			gs_Current_Operate_List_ProgramEdit++;		 						 //ʼ��ָ��ǰ�������	
		 g_Background_Color_ProgramEdit = gs_Current_Operate_List_ProgramEdit%8;
		 if(g_Background_Color_ProgramEdit==0)
			 g_Background_Color_ProgramEdit  = 8;
		for(i=j;i<n;i++)
		 {
			 /*List Value1 2 3 4  5 LC_ID Key IO_Group Delay_Time*/
			AT24CXX_Read(CURRENT_PROGRAM_ADR2+i*ORDER_LENTH,&Order_Temp[3],ORDER_LENTH);
			Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Flag  = 1;
			Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].List  = Order_Temp[3]|Order_Temp[4]<<8;
			Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value1= Order_Temp[5]|Order_Temp[6]<<8|Order_Temp[7]<<16|Order_Temp[8]<<24;
			Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value2= Order_Temp[9]|Order_Temp[10]<<8|Order_Temp[11]<<16|Order_Temp[12]<<24;
			Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value3= Order_Temp[13]|Order_Temp[14]<<8|Order_Temp[15]<<16|Order_Temp[16]<<24;
			Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value4= Order_Temp[17]|Order_Temp[18]<<8|Order_Temp[19]<<16|Order_Temp[20]<<24;	
			Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value5= Order_Temp[21]|Order_Temp[22]<<8|Order_Temp[23]<<16|Order_Temp[24]<<24;		 
			Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].LC_ID	= Order_Temp[25];	
			Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Key   = Order_Temp[26];
			 if(1==Order_Temp[26]||2==Order_Temp[26]||3==Order_Temp[26]||5==Order_Temp[26])//Order_Type 1 �㽺 2���� 3ȡ�� 4 ��ʱ 5����
			 {
				Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].IO_Group  = Order_Temp[27]|Order_Temp[28]<<8|Order_Temp[29]<<16|Order_Temp[30]<<24;	
			 }
			else if(4==Order_Temp[26])//��ʱָ��
			{
				Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Delay_Time  = Order_Temp[29]|Order_Temp[30]<<8;				
			}
		 }
		 if(i%LARGESTPROGRAMNUM!=0)
		 {
			for(k=i%LARGESTPROGRAMNUM;k<LARGESTPROGRAMNUM;k++)
			{
				 /*List Value1 2 3 4  5 LC_ID Key IO_Group Delay_Time*/
				Free_Program_Operate.Program[k].Flag  = 0;
				Free_Program_Operate.Program[k].List  = 0;
				Free_Program_Operate.Program[k].Value1= 0;
				Free_Program_Operate.Program[k].Value2= 0;
				Free_Program_Operate.Program[k].Value3= 0;
				Free_Program_Operate.Program[k].Value4= 0;	
				Free_Program_Operate.Program[k].Value5= 0;			 
				Free_Program_Operate.Program[k].LC_ID	= 0;	
				Free_Program_Operate.Program[k].Key   = 0;
				Free_Program_Operate.Program[k].IO_Group  = 0;	
				Free_Program_Operate.Program[k].Delay_Time= 0;					
			 }
		 }
	 }
	else
	{
	 Program_Edit_OrNot=1;
		 /*List Value1 2 3 4  5 LC_ID Key IO_Group Delay_Time*/
		if(Order_Type==1||Order_Type==2||Order_Type==5)//�㽺 ����  ����ָ��
		{
		 IIC_Temp[0]=Point_Action_Flag;
		 IIC_Temp[1]=Point_Action_Flag>>8;
		 IIC_Temp[2] = gs_Manul_XAxis_Position;//X������
		 IIC_Temp[3] = gs_Manul_XAxis_Position>>8;
		 IIC_Temp[4] = gs_Manul_XAxis_Position>>16;
		 IIC_Temp[5] = (gs_Manul_XAxis_Position>>24)|0xF0;
		 IIC_Temp[6] = gs_Manul_LAxis_Position;//Y������
		 IIC_Temp[7] = gs_Manul_LAxis_Position>>8;
		 IIC_Temp[8] = gs_Manul_LAxis_Position>>16;
		 IIC_Temp[9] = (gs_Manul_ZAxis_Position>>24)|0xF0;
		 IIC_Temp[10] = gs_Manul_ZAxis_Position;//Z������
		 IIC_Temp[11] = gs_Manul_ZAxis_Position>>8;
		 IIC_Temp[12] = gs_Manul_ZAxis_Position>>16;
		 IIC_Temp[13] = (gs_Manul_ZAxis_Position>>24)|0xF0;
		 IIC_Temp[14] = gs_Manul_AAxis_Position;//V������
		 IIC_Temp[15] = gs_Manul_AAxis_Position>>8;
		 IIC_Temp[16] = gs_Manul_AAxis_Position>>16;
		 IIC_Temp[17] = (gs_Manul_AAxis_Position>>24)|0xF0;
		 IIC_Temp[18] = gs_Manul_BAxis_Position;//W������
		 IIC_Temp[19] = gs_Manul_BAxis_Position>>8;
		 IIC_Temp[20] = gs_Manul_BAxis_Position>>16;
		 IIC_Temp[21] = (gs_Manul_BAxis_Position>>24)|0xF0;		
		 IIC_Temp[22] = LC_ID;
		 IIC_Temp[23] = Order_Type;
		 IIC_Temp[24] = IO_Choose;
		 IIC_Temp[25] = IO_Choose>>8;
		 IIC_Temp[26] = IO_Choose>>16;	
		 IIC_Temp[27] = IO_Choose>>24;
		 AT24CXX_Write(CURRENT_PROGRAM_ADR2+ORDER_LENTH*(Point_Action_Flag-1),IIC_Temp,ORDER_LENTH);	//�޸�ָ��
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].IO_Group= IO_Choose;
		 if(Order_Type==1||Order_Type==2)//����ָ���ʾ�ϲ�
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].LC_ID   = LC_ID;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].IO_Group= IO_Choose;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value1  = gs_Manul_XAxis_Position|0xF0<<24;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value2  = gs_Manul_LAxis_Position|0xF0<<24;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value3  = gs_Manul_ZAxis_Position|0xF0<<24;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value4  = gs_Manul_AAxis_Position|0xF0<<24;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value5  = gs_Manul_BAxis_Position|0xF0<<24;
	 }
		else if(Order_Type==3)//ȡ��
		{
		 IIC_Temp[0]=Point_Action_Flag;
		 IIC_Temp[1]=Point_Action_Flag>>8;
		 IIC_Temp[2] = 0;//X������
		 IIC_Temp[3] = 0;
		 IIC_Temp[4] = 0;
		 IIC_Temp[5] = 0;
		 IIC_Temp[6] = 0;//Y������
		 IIC_Temp[7] = 0;
		 IIC_Temp[8] = 0;
		 IIC_Temp[9] = 0;
		 IIC_Temp[10] = 0;//Z������
		 IIC_Temp[11] = 0;
		 IIC_Temp[12] = 0;
		 IIC_Temp[13] = 0;
		 IIC_Temp[14] = 0;//V������
		 IIC_Temp[15] = 0;
		 IIC_Temp[16] = 0;
		 IIC_Temp[17] = 0;
		 IIC_Temp[18] = 0;//W������
		 IIC_Temp[19] = 0;
		 IIC_Temp[20] = 0;
		 IIC_Temp[21] = 0;		
		 IIC_Temp[22] = LC_ID;
		 IIC_Temp[23] = Order_Type;
		 IIC_Temp[24] = IO_Choose;
		 IIC_Temp[25] = IO_Choose>>8;
		 IIC_Temp[26] = IO_Choose>>16;	
		 IIC_Temp[27] = IO_Choose>>24;
		 AT24CXX_Write(CURRENT_PROGRAM_ADR2+ORDER_LENTH*(Point_Action_Flag-1),IIC_Temp,ORDER_LENTH);	//�޸�ָ��
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].IO_Group= IO_Choose;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].LC_ID	 = LC_ID;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value1  = 0;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value2  = 0;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value3  = 0;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value4  = 0;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value5  = 0;			
		}
		else if(Order_Type==4)//��ʱָ��
		{
		 IIC_Temp[0]=Point_Action_Flag;
		 IIC_Temp[1]=Point_Action_Flag>>8;
		 IIC_Temp[2] = 0;//X������
		 IIC_Temp[3] = 0;
		 IIC_Temp[4] = 0;
		 IIC_Temp[5] = 0;
		 IIC_Temp[6] = 0;//Y������
		 IIC_Temp[7] = 0;
		 IIC_Temp[8] = 0;
		 IIC_Temp[9] = 0;
		 IIC_Temp[10] = 0;//Z������
		 IIC_Temp[11] = 0;
		 IIC_Temp[12] = 0;
		 IIC_Temp[13] = 0;
		 IIC_Temp[14] = 0;//V������
		 IIC_Temp[15] = 0;
		 IIC_Temp[16] = 0;
		 IIC_Temp[17] = 0;
		 IIC_Temp[18] = 0;//W������
		 IIC_Temp[19] = 0;
		 IIC_Temp[20] = 0;
		 IIC_Temp[21] = 0;		
		 IIC_Temp[22] = 0;
		 IIC_Temp[23] = Order_Type;
		 IIC_Temp[24] = 0;
		 IIC_Temp[25] = 0;		 
		 IIC_Temp[26] = Delay_Time;	
		 IIC_Temp[27] = Delay_Time>>8;	
		 AT24CXX_Write(CURRENT_PROGRAM_ADR2+ORDER_LENTH*(Point_Action_Flag-1),IIC_Temp,ORDER_LENTH);	//�޸�ָ��
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Delay_Time = Delay_Time;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value1  = 0;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value2  = 0;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value3  = 0;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value4  = 0;
		 Free_Program_Operate.Program[(Point_Action_Flag-1)%LARGESTPROGRAMNUM].Value5  = 0;		 
		}
	 Program_Edit_OrNot=1;																 //�����޸�
	 LCD_Current_Page = 8;
	}
	 /*******ˢ�����ɱ�̽���**********/
		gs_Run_Program_Num=0;
		LCD_Main_Page = FREE_PROGRAMME;
		g_Watch_IO_Control      = 0; 						          //IO��ʵʱ��ع�
		g_Refresh_Dispaly_List_Flag=TRUE;                         //ˢ�³�����ʾ�б�
		g_Robot_Auto_Reset = FALSE;                               //��е���Զ���λ��
		g_Refresh_Dispaly_List_Flag=TRUE; 	
		
		IO_Parameter_Refresh=TRUE;

}
