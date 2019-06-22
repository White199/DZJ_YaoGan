/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __parameter_h_
#define __parameter_h_


/******************���������***************************/
#define PROGRAMSTARTACTION 1    //�Ƿ��п�������

#define LARGESTPROGRAMNUM 32	//������ָ����
#define SAVEPROGRAMNUM 104			//��ǰ����ĳ���
#define SAVEPROGRAMNUM_2 8			//��ȥ�ϲֳ���󱣴�ĳ���
#define SAVEPROINT 100				//��ǰ����ĵ�
#define SAVESAFEAREA 3				//��ǰ����İ�ȫ��
#define SAVESOFTLIMIT 6				//��ǰ���������λ

#define MINROBOTPOSITION   1000000    //
#define MAXROBOTPOSITION   2000000    //

#define LC_MAX_NUM   10 				//�ϲ���Ŀ����
#define ORDER_LENTH  30       //ָ����ֽ�

//���ɱ��
typedef struct 
{
	u8  Flag;  //��־λ
	u8  List;  //ָ����
	u8  Order; //ָ������
	u8  Key;   //������
	u32 Value1;//����1-����OYZVW
	u32 Value2;//����2-
	u32 Value3;//����3-
	u32 Value4;//����4-
	u32 Value5;//����5-
	u8 LC_ID;//�ϲ�ѡ��
	u16 IO_Group;//�㽺����ͷIO����
	u16 Delay_Time;//��ʱʱ��
	
}FreeProgram;
//extern FreeProgram Current_Process_Program[LARGESTPROGRAMNUM];
//����洢
typedef struct 
{
	u8  Flag;	   //�Ƿ��г���
	u8  Code;	   //����ı�ţ��·��������ư���ʶ����
	u32 Name;	   //��������
	u8  Num;	   //�������Ĺ�ģ��С����������ʱ��ʾ�ж�
    FreeProgram	Program[LARGESTPROGRAMNUM];
}SaveProgram;

//����洢
typedef struct 
{
	u8  Flag;	   //�Ƿ��г���
	u8  Code;	   //������
	u32 Name;	   //��������
	u8  Num;	   //�����ģ�����������
	u16 Address;   //IIC�Ĵ洢��ַ
}SaveProgramIICAddress;






//��洢
//typedef struct 
//{
//	u8  Flag;	   //�Ƿ��г���
//	u8  Code;	   //�����ģ�����������
//	u16 Address;   //IIC�Ĵ洢��ַ
//}SavePointIICAddress;

extern SaveProgramIICAddress Program_IIC_Address[SAVEPROGRAMNUM];
//extern SavePointIICAddress Point_IIC_Address[SAVEPROINT];
//extern SaveProgram Free_Program_Save[SAVEPROGRAMNUM];
extern SaveProgram Free_Program_Operate;
extern SaveProgram LC_Load_Program_Operate;
extern SaveProgram LC_UnLoad_Program_Operate;
extern SaveProgram Start_Action_Program_Operate;
extern SaveProgram Reset_Action_Program_Operate;

extern u8 Start_Recieve_Program_Flag;
extern u8 Current_Delete_Program;	        //��ǰɾ������


//-----------------------------------------------------------//
/***************���������Ϣ*********************************/
typedef struct 
{
	u8  Flag;	          //�Ƿ��г���
	u8  Code;	          //����ı�ţ��·��������ư���ʶ����
	u32 Name;	          //N
	u32 Address_Order;  //ָ��洢��ַ
	u16  Order_Num;			//ָ����Ŀ
	u32 Time;					  //ʱ��
}SavedProgramInf;   //13
//LC��ʾ�̴洢
typedef struct 
{
	u32 XPoint;	
	u32 YPoint;
}LCPosition;
//�ϲֱ�
typedef struct 
{
	LCPosition LC_Load_Position[22][20];
}LCTable;




typedef struct 
{
	u32 Left_Limit  ; //-
	u32 Right_Limit ; //+
	u8  Switch_Limit; //����
}SoftLimit;

typedef struct 
{
	u32 X_Left ;
	u32 Z_Up ;
	u32 X_Right ;
	u32 Z_Down ;
	u8  SafeArea_Switch;
}SafeAreas;


//��洢
typedef struct 
{
	u8  Code;		//���
	u8  Flag;
	u32 Name;		//�������
	u32 Point;		//X��ֵ
}SavePoint;

typedef struct //--6.29
{
	u8  Type[4];
	u16 Time[4];
}ACC;  //Accelerate

typedef struct 
{
	u8  Axis;
	u8  Origin[6];
// 	u16  Accelerate ;  6.29 ע��
	ACC Accelerate;  //--6.29
	u8  Auto_SpeedLevel;//�Զ���������ٶ�
	u8  Manual_SpeedLevel;//�ֶ�ʾ������ٶ�
	u8  Manual_Prescaler;
	u16 SPEED_CONTROL_TIME_MANUAL[3];         //18000ÿ��2ms()��һ�����壬���ÿ���������ڴ���2ms����ÿ���������ڶ���
	u32 OriginSpeed[6];
	u8  OriginPosition[6];
	u8  OriginDir[6];
	u32 PulseTime;
	u16 GearRatio[6];//X Z L O A B
	u8  ServoAlarm;
	//����λ�������ؼӹ�����IO��0001����>X32,0010����>X33,0100����>X34,1000����>X35Ĭ��0��δ���ã�
	//����λ����ȴ�ʱ�䣨min��0000��ʾ����ⳬʱ���õȴ���0001-1111��ʾ1-16min
	u8	NcComplete;
	u8  ManualSpeed_H[3];//XYZO  A  B  
	u8  ManualSpeed_L[3];//XYZO  A  B  
	u8  AutoSpeed_H[3];//XYZO  A  B  
	u8  AutoSpeed_L[3];//XYZO  A  B  
	u8  Acceleration_UP[3];//XYZO  A  B  
	u8  Acceleration_DOWN[3];//XYZO  A  B  
}JXSParameter;





typedef struct 
{
	u8  Switch ;
	u8  Layer ;
	u32 O_PositionS;
	u32 O_PositionE;
	u8  H_Num ;	
	u8  V_Num ;	
	LCPosition Position1;
	LCPosition Position2;
	LCPosition Position3;
	
	u8  ID;
	u32 LC_Hight;
	u32 HT_Hight;
	u16 Glue_Time;
	u16 Diamonds_Time;
}LCParameter;

typedef struct 
{
	u32  RW_Num ;	//����ƻ�
	u32  SC_Num ;	//����
	u32  CJ_Num ;	//���
	u32  JG_Num ;	//���
	u32  LJ_Num ;	//�ۼ�
}SCParameter;





extern JXSParameter JXS_Parameter;						  //�ϲֲ���
extern LCPosition LC_Load_Position[20][20];	              //����洢��λ�ñ�
extern LCPosition LC_UnLoad_Position[20][20];	          //����洢��λ�ñ�
extern u32 LC_Load_OAxis_Position[50] ;			          //���㴹ֱ��λ�ñ�
extern u32 LC_UnLoad_OAxis_Position[50] ;		          //���㴹ֱ��λ�ñ�
extern LCParameter LC_Parameter_Load[];					  //�ϲֲ���
//extern LCParameter LC_Parameter_UnLoad;					  //�ϲֲ���
extern SCParameter SC_Parameter ;						  //��������
extern SoftLimit  Robot_SoftLimit[4];		              //XZ����λ����
extern SafeAreas Robot_Safe_Area[SAVESAFEAREA];		      //��ȫ�������
extern u8 LC_Load_Pre_ListX;
extern u8 LC_Load_Pre_ListY;
extern u8 LC_Load_Pre_ListO_Get;
extern u8 LC_Load_Pre_ListO_Place;
extern u8 LC_UnLoad_Pre_ListX;
extern u8 LC_UnLoad_Pre_ListY;
extern u8 LC_UnLoad_Pre_ListO;
extern u8 LC_UnLoad_Pre_ListZ;
extern u8 LC_UnLoad_ZAxis_Num;
extern u8 g_DD_ResetAction;

extern SavePoint Manul_Save_PointX;
extern SavePoint Manul_Save_PointZ;
extern SavePoint Manul_Save_PointL;
extern SavePoint Manul_Save_PointO;
extern u8 Current_Delete_XZPoint;
extern u8 Current_Delete_LOPoint;
extern u8 g_Run_Program_Num ;
extern u16 SaveProgram_IIC_Address;
extern u8  SaveProgram_IIC_Num ;
extern u8 g_LDOperation_Pause;


//------------------------------------//
extern LCTable LC_Load_Table[LC_MAX_NUM];

//extern  ManualOperate Manual_Operate;
//extern  ProductCount  Product_Count;
//extern  ParmeterTime  Parameter_Time;

//extern AxsisSet Machine_Wait_Position;   //������
//extern AxsisSet Load_Wait_Position;      //ȡ�ϴ�����
//extern AxsisSet Load_Material_Position;  //ȡ�ϵ�
//extern AxsisSet Unload_Wait_Position;    //���ϵȴ���
//extern AxsisSet Unload_Material_Position;//���ϵ�
//extern AxsisSet Product_Place_Position;  //��Ʒ�ڷŵ�
//extern AxsisSet Product_Check_Position;  //����
//extern SafeArea Material_Safe_Area;
//extern SafeArea Machine_Safe_Area;
extern u32 X_Axsis_Maxlength;
extern u32 Z_Axsis_Maxlength;
extern u32 L_Axsis_Maxlength;
extern u32 O_Axsis_Maxlength;
extern u32 A_Axsis_Maxlength;
extern u32 B_Axsis_Maxlength;
extern u32 X_Axsis_Minlength;						
extern u32 Z_Axsis_Minlength;
extern u32 L_Axsis_Minlength;
extern u32 O_Axsis_Minlength;
extern u32 A_Axsis_Minlength;
extern u32 B_Axsis_Minlength;
//---0821
extern u32 X_Axsis_Minlength_Can ;
extern u32 Z_Axsis_Minlength_Can ;
extern u32 L_Axsis_Minlength_Can ;
extern u32 O_Axsis_Minlength_Can ;
extern u32 A_Axsis_Minlength_Can ;
extern u32 B_Axsis_Minlength_Can ;
extern u32 X_Axsis_Maxlength_Can ;
extern u32 Z_Axsis_Maxlength_Can ;
extern u32 L_Axsis_Maxlength_Can ;
extern u32 O_Axsis_Maxlength_Can ;		    
extern u32 A_Axsis_Maxlength_Can ;
extern u32 B_Axsis_Maxlength_Can ;


//extern u8 SoftLimit_Chang_Done;				
extern u32 Robot_Soft_Limit[2][2];

extern u8  PVD_data[4];
extern u8  IIC_Parameter[100];



extern SavedProgramInf Saved_Program[];
extern SaveProgram Free_Program_Operate1;



void ParameterOrder(void);
void FreeProgramSend(void);
void ReadIICData(void);
void ReadIICSysParameter(void);
void ReadIIC_Program(void);
void ReadIIC_One_Program(void);
void Read_LC_List(u8);
void Calculate_LC_Load_Parameter(u8 m);
void Calculate_LC_UnLoad_Parameter(void);
void Calculate_LC_Load_OAxis_Parameter(void);
void Calculate_LC_UnLoad_OAxis_Parameter(void);

//-----------------------------//
void Program_Send(void);

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

