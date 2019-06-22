/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : main.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "SpeedControl.h" 
#include "StatusControl.h"
#include "Auto.h"
#include "Manual.h"
#include "Parameter.h"
#include "Error.h"
#include "SpeedControl_CAN.h"


#define SET_X_PULSE_PERIOD_HALF(X)		{TIM3->ARR = (X + X - 1); TIM3->CCR1 = X; TIM3->CCR2 = X;}	   //X-TIM3
#define CLOSE_X_PULSE					TIM_Cmd(TIM3, DISABLE)

#define SET_Z_PULSE_PERIOD_HALF(X)		{TIM2->ARR = (X + X - 1); TIM2->CCR1 = X; TIM2->CCR2 = X;}	   //Z-TIM2
#define CLOSE_Z_PULSE					TIM_Cmd(TIM2, DISABLE)

#define SET_L_PULSE_PERIOD_HALF(X)		{TIM1->ARR = (X + X - 1); TIM1->CCR3 = X; TIM1->CCR4 = X;}	   //L-TIM1-CH3
#define CLOSE_L_PULSE					TIM_Cmd(TIM1, DISABLE)

#define SET_O_PULSE_PERIOD_HALF(X)		{TIM4->ARR = (X + X - 1); TIM4->CCR3 = X; TIM4->CCR3 = X;}	   //O-TIM4-CH3
#define CLOSE_O_PULSE					TIM_Cmd(TIM4, DISABLE)

#define SET_X_DIRECTION_H	     	GPIO_SetBits  (GPIOA, GPIO_Pin_7);		//PA7
#define SET_X_DIRECTION_L			GPIO_ResetBits(GPIOA, GPIO_Pin_7);

#define SET_Z_DIRECTION_H			GPIO_SetBits  (GPIOA, GPIO_Pin_1);		//PA1
#define SET_Z_DIRECTION_L			GPIO_ResetBits(GPIOA, GPIO_Pin_1);

#define SET_L_DIRECTION_H	     	GPIO_SetBits  (GPIOE, GPIO_Pin_14);		//PE14
#define SET_L_DIRECTION_L			GPIO_ResetBits(GPIOE, GPIO_Pin_14);

#define SET_O_DIRECTION_H			GPIO_SetBits  (GPIOB, GPIO_Pin_9);		//PB9
#define SET_O_DIRECTION_L			GPIO_ResetBits(GPIOB, GPIO_Pin_9);



/*------�ŷ�������ģʽ״̬------*/
u8 Servo_Control_Mode = LINKEDRUNNINGMODE;

u32 Pulse_TotalCounter[6] = {MINROBOTPOSITION, MINROBOTPOSITION ,MINROBOTPOSITION ,MINROBOTPOSITION,MINROBOTPOSITION,MINROBOTPOSITION};				//X�ᡢZ�ᡢL������������

#define SPEED_CONTROL_TIME		18000                //ÿ��2ms(36000)��һ�����壬���ÿ���������ڴ���2ms����ÿ���������ڶ���
#define MAX_SPEED	       	    100000 				 //����ٶ�100k
/*---��Ҫ���͵��������---*/				
u32 Servo_Pulse_Count[6] = {0};			
u32 PULSE_MINHALF_PERIOD=PULSE_MINHALFPERIOD; //�����ٶȵȼ��任�������ٶȶ�Ӧ����С������ -*/



/*---�������巢�͹����з��͵ĸ���---*/
u32 Pulse_Counter[6] = {0};						//һ�����ƹ��̵����巢���ܸ���
u32 Pulse_PresentHalfPeriod[6] = {18000,18000,18000,18000}; //��ǰ���룩����---*/	
u32 Pulse_InitialHalfPeriod[6] = {18000,18000,18000,18000};  //��������ڳ�ʼֵ����ʼ�ٶ�ֵ---*/ 
u32 Pulse_Maxspeed_HalfPeriod[6] = {180,180,180,180}; //����ٶȶ�Ӧ��С������ -*/
u32 Pulse_Maxspeed_HalfPeriod_i[6]={0};     

       u8  SpeedUp_Min_Acceleration=20;           //���ٶȳ�ʼֵ  50-->20
       u16 Present_SpeedUp_Jerk[6]={10,10,10,10};               //��ǰ�Ӽ��ٶ�ֵ 
       u16 Present_SpeedUp_Max_Acceleration[6]={200,200,200,200};  //��ǰ�����ٶ�ֵ
       u8  Pulse_Record_Flag[6]={FALSE,FALSE,FALSE,FALSE};			      //�Ӽ��ٶȼ�¼��־λ
       u16 Pulse_PresentAcceleration[6] ={20,20,20,20};	      //��ǰ���ٶ�
//	   u16 Pulse_LastAcceleration[4] ={20,20,20,20};	      //��¼���ٵ�����ٶ�ʱ�����һ�μ��ٶ�
       u32 Delta_Present_MaxSpeed[6]={0};              //��ǰ����ٶȵĲ������
       u32 Pulse_Present_MaxSpeed[6]={100000,100000,100000,100000};         //100k
static u32 Pulse_MinSpeed[6] = {1000,1000,1000,1000};			      //��ʼ��С�ٶ� �� ��ʼ�ٶȱ���һ��
static u32 Present_SpeedUp_Min_Acceleration[6] = {20,20,20,20}; //��ʼ��С���ٶ� �� ��ʼ�ٶȱ���һ��
       u32 Period_Counter_Acceleration[6]={0};  //��С���ٶ��ۼӵ������ٶ���Ҫ�������� (550-50)/10=50
static u32 Period_Counter[6]={0};               //���ٽ׶�������
static u32 Period_Counter2[6]={0};              //���ٽ׶�-Down������
static u32 Period_Counter_UpA[6]={0};           //���ٽ׶μ��ٶȱ仯��������
static u32 Period_Counter_HoldA[6]={0};         //���ٽ׶����ٶȱ仯��������
static u32 Period_Counter_DownA[6]={0};         //���ٽ׶μ��ٶȱ仯��������
       u32 Delta_Speed_Frequency[6]={0};        //����С���ٶȼ��ٵ������ٶ�����Ҫ������
       u32 Amin_Counter[6]={0};

/*---�ٶȲ���---*/	
       u32 Present_Pulse_SpeedControlStep_Counter[6]={0}; //��¼��ǰ���ٶȹ滮�������岽��
static u32 NextSpeed[6] = {0};                   	     //�޸�֮��Ϊ�¸��ж϶�Ӧ���ٶ�ֵ
static u32 Pulse_InitialSpeed[6] = {1000,1000,1000,1000};			     //��ǰ�����ڵĳ�ʼ�ٶ�-ʵʱ����
static u32 Pulse_SpeedControlStep_Counter[6] = {0};	     //ÿ��2ms�Ӽ�����ʹ�õ�����������
static u32 Remain_SpeedControlStep_Counter[6] = {0};	   //ÿ��2ms�Ӽ�����ʹ�õ�����������

/*---�Ӽ��ٲ���---*/	
u32 Pulse_ControlStep[6] = {SPEED_UP,SPEED_UP,SPEED_UP,SPEED_UP};	
/*---����ʹ��������---*/	
u32 SpeedUp_Pulse_Counter[6] = {0}; 			  //��¼���ٶ�ʹ�õ��������������ó��ٶȸ��ε�ʹ��������

//----------------------------------//  
u8 A_List[6]={0};//���ٶȱ��±�

                         
/***********************************************************
**  ���ܣ��ٶȹ滮   
**	ʱ�䣺2016-07-18
**	���ߣ�wx
**	���������ٶȹ滮����ת��Ϊ�Ӽ��ٶȵı仯����
**  �Ӽ��ٶȣ�   SpeedUp_Jerk   �����ٶȣ� SpeedUp_Max_Acceleration
**  18000:1k    3600:5K   1800:10k  360:50k  180:100k
% ***************************2016-07-24-01*********************************
%     Vmin=1k=1000;HP=36000;  Vmax=100K=100000; HP=180; Amin=50;
%        V           HP(min)  J       Amax         Ta(max)     Sa(max) 
%     1%~9%(Vmax)    18000    3     140(Amax)-30  	  t= 86*2ms   S=  9.95mm  
% 	  10%~19%(Vmax)  1800     4     170(Amax)-30      t=137*2ms   S= 29.85mm
%     20%~29%(Vmax)  900      5     200(Amax)-30      t=170*2ms   S= 53.71mm
% 	  30%~39%(Vmax)  600      6     230(Amax)-30  	  t=198*2ms   S= 83.37mm
%     40%~49%(Vmax)  450      6     260(Amax)-35      t=221*2ms   S=115.05mm  
%     50%~59%(Vmax)  360      8     330(Amax)-35	    t=213*2ms   S=132.57mm
% 	  60%~69%(Vmax)  300      10    400(Amax)-35      t=212*2ms   S=154.62mm	
% 	  70%~79%(Vmax)  257      10    450(Amax)-40      t=221*2ms   S=186.31mm	
% 	  80%~89%(Vmax)  225      10    500(Amax)-45      t=224*2ms   S=208.40mm	
% 	  90%~100%(Vmax) 200      10    550(Amax)-50      t=231*2ms   S=238.15mm
% ******************************************* ******************************/
//u16 SpeedUp_Jerk[10]             = {3,  4,  5,  6,  6,  8,  10, 10 ,10, 10 };
//u16 SpeedUp_Max_Acceleration[10] = {140,170,200,230,260,330,400,450,500,550};

//2016-09-14-wtmx
u16 *SpeedUp_Jerk=0;
u16 *SpeedUp_Max_Acceleration=0;

u16 SpeedUp_Jerk1[10]             = {10,  10,  5,  5,  5,  5,  5,  5,  5,  5};
u16 SpeedUp_Max_Acceleration1[10] = {200,220,300,320,340,360,380,400,420,440};

u16 SpeedUp_Jerk2[10]             = {15,  15, 10, 10, 10, 10, 10, 10, 10, 10};
u16 SpeedUp_Max_Acceleration2[10] = {200,260,300,340,380,420,460,500,540,580};

u16 SpeedUp_Jerk3[10]             = {15,  15, 10, 10, 10, 10, 10, 10, 10, 10};
u16 SpeedUp_Max_Acceleration3[10] = {200,260,300,340,380,420,460,500,540,580};

//2017-06-22-S_Style Sheet
//200ms  210ms 220ms ....990ms 1000ms
// u16 SpeedUp_Max_A=0;  6.29ע��
//---6.29ÿ�����Ӧһ�������ٶ�
u16 SpeedUp_Max_A_X=0;
u16 SpeedUp_Max_A_Z=0;
u16 SpeedUp_Max_A_L=0;
u16 SpeedUp_Max_A_O=0;
//---6.29
u16 SpeedUp_Max_A[6]={0};

u16 SpeedUp_Max_Acceleration_Sheet[81]=
{ 
  780, 670, 600, 550, 510, 480, 450, 420,
  400, 380, 370, 350, 340, 330, 320, 300,
  290, 280, 270, 270, 260, 250,	250, 240,
  240, 230, 220, 220, 210, 210, 200, 200,
  190, 190, 190, 180, 180, 180, 170, 170, 
  170, 160, 160, 160, 160, 150, 150, 150,
  150, 140, 140, 140, 140, 140,	130, 130,
  130, 130, 130, 120, 120, 120, 120, 120,
  120, 120, 110, 110, 110, 110, 110, 110,
  110, 100, 100, 100, 100, 100, 100, 100,
	100
};


u8 Timer_Enable_X = DISABLE;						 //������ƶ�ʱ���Ƿ�ʹ��
u8 Timer_Enable_Z = DISABLE;						 //������ƶ�ʱ���Ƿ�ʹ��
u8 Timer_Enable_L = DISABLE;						 //������ƶ�ʱ���Ƿ�ʹ��
u8 Timer_Enable_O = DISABLE;						 //������ƶ�ʱ���Ƿ�ʹ��
u8 Timer_Enable_A = DISABLE;						 //������ƶ�ʱ���Ƿ�ʹ��
u8 Timer_Enable_B = DISABLE;						 //������ƶ�ʱ���Ƿ�ʹ��
//---2017.8.1
u32 pulse_Present_MaxSpeed;
u32 servo_Pulse_Count;

//------------------------------------------//
u8 Sheet_Position[6]={0};//ң�˼Ӽ��ٶ�Ӧ�ļ��ٶ��ڱ��е�λ��
u8 Speed_Up_Flag[6]={0},Speed_Down_Flag[6]={0};//���ٻ��Ǽ��ٵı�־λ


extern u8 A_List[6];//���ٶȱ��±�

/**************************************************************************************************
**  ��������  SpeedPlanningReset()
**	�����������
**	�����������
**	�������ܣ���λ��ر�־λ
**	��ע��	  			  
**  ���ߣ�    ����     
**  �������ڣ�2016/10/11 
***************************************************************************************************/
void  SpeedPlanningReset(u8 Axis)
{
	 Period_Counter[Axis]=0;
	 Period_Counter2[Axis]=0;
	 Amin_Counter[Axis]=0;
	 Pulse_Record_Flag[Axis]=FALSE;
	 Period_Counter_UpA[Axis]=0;
	 Period_Counter_HoldA[Axis]=0;
	 Period_Counter_DownA[Axis]=0;
	 Pulse_Maxspeed_HalfPeriod_i[Axis]=0;
	 NextSpeed[X_Axsis] = 1000;
}
/**************************************************************************************************
**  ��������  MaxSpeedSection()
**	�����������
**	�����������
**	�������ܣ���������ٶ�ֵ���ֶμ�����ٶ�
**	��ע��	  			  
**  ���ߣ�    ����     
**  �������ڣ�2013/1/23 
***************************************************************************************************/
void  MaxSpeedSection(u8 Axis)
{
	  
	  /*
	  if(Pulse_Maxspeed_HalfPeriod[Axis]>18000)  //1%=36000 *2 =72000>65536 ������Χ
	  {
	  	  Pulse_Maxspeed_HalfPeriod[Axis]=18000; //1%	
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[0];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[0];
	  }
	  //1%<=����ٶ�<10%
	  if(Pulse_Maxspeed_HalfPeriod[Axis]>1800)	//����ٶ�<10%
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[0];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[0];
	  }
	  //10%<=����ٶ�<20%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>900 && Pulse_Maxspeed_HalfPeriod[Axis]<=1800)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[1];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[1];
	  }	
	  //20%<=����ٶ�<30%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>600 && Pulse_Maxspeed_HalfPeriod[Axis]<=900)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[2];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[2];
	  }	
	  //30%<=����ٶ�<40%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>450 && Pulse_Maxspeed_HalfPeriod[Axis]<=600)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[3];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[3];
	  }	
	  //40%<=����ٶ�<50%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>360 && Pulse_Maxspeed_HalfPeriod[Axis]<=450)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[4];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[4];
	  }	
	  //50<=����ٶ�<60%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>300 && Pulse_Maxspeed_HalfPeriod[Axis]<=360)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[5];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[5];
	  }	
	  //60%<=����ٶ�<70%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>257 && Pulse_Maxspeed_HalfPeriod[Axis]<=300)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[6];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[6];
	  }	
	  //70%<=����ٶ�<80%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>225 && Pulse_Maxspeed_HalfPeriod[Axis]<=257)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[7];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[7];
	  }	
	  //80%<=����ٶ�<90%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>200 && Pulse_Maxspeed_HalfPeriod[Axis]<=225)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[8];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[8];
	  }		  
	  //90%<=����ٶ�<=100%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>=180 && Pulse_Maxspeed_HalfPeriod[Axis]<=200)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[9];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[9];
	  }	
	  else
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[0];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[0];
	  }
	  */
// 		Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_A;  6.29ע��

    Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_A[Axis];
		
	    //���㵱ǰ���ڵ�����ٶ�ֵVmax
		Pulse_Present_MaxSpeed[Axis]=18000000/Pulse_Maxspeed_HalfPeriod[Axis]; 
		//�������С���ٶȵ������ٶ�ֵ����������
		Period_Counter_Acceleration[Axis]=(Present_SpeedUp_Max_Acceleration[Axis]-SpeedUp_Min_Acceleration)/Present_SpeedUp_Jerk[Axis];
		//�������С���ٶȵ������ٶ�ֵ���������
		Delta_Speed_Frequency[Axis]=(Present_SpeedUp_Max_Acceleration[Axis]+SpeedUp_Min_Acceleration)*(Period_Counter_Acceleration[Axis]+1)/2;
		
		//����ٶȲ���������� �������ٶ�Ϊ10000,ʵ�ʵĿɴﵽΪ9980
		if((Pulse_Present_MaxSpeed[Axis]-Pulse_MinSpeed[Axis])>(Delta_Speed_Frequency[Axis]+Delta_Speed_Frequency[Axis]))
		{
			Delta_Present_MaxSpeed[Axis]=(Pulse_Present_MaxSpeed[Axis]-Pulse_MinSpeed[Axis]-Delta_Speed_Frequency[Axis]-Delta_Speed_Frequency[Axis])%Present_SpeedUp_Max_Acceleration[Axis];
			Pulse_Present_MaxSpeed[Axis]=Pulse_Present_MaxSpeed[Axis]-Delta_Present_MaxSpeed[Axis];
			Pulse_Maxspeed_HalfPeriod[Axis]=18000000/Pulse_Present_MaxSpeed[Axis];
		}		
}


		
/**************************************************************************************************
**  ��������  SendPulse()
**	�����������
**	�����������
**	�������ܣ����ŷ�����������
**	��ע��	  ����ȫ�ֱ���Servo_Control_Mode��Servo_Pulse_Count��Axsis_Chosen
**			  ��ȷ�������������������Ϳ����ᡣ
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void SendPulse(u8 Axis)	 //��Ҫ����ControlMode,Axsis,PulseCount
{
	 switch(Servo_Control_Mode)
	 {	 	 
			 case BACKTOORIGIN:				 //��ԭ��ģʽ
				  Servo_Pulse_Count[Axis] = MINROBOTPOSITION;
				  break;
			 
			 case AUTORUNNINGMODE:			 //�Զ�����ģʽ
			  	  Servo_Pulse_Count[Axis] = Auto_Pulse_Count;
				  break;

			 case JOGRUNNINGMODE:			 //�綯����ģʽ
			      Servo_Pulse_Count[Axis] = Jog_Pulse_Count;
				  break;

			 case LINKEDRUNNINGMODE:		 //��������ģʽ
				  Servo_Pulse_Count[Axis] = Linked_Pulse_Count;
				  break;

			 default:
				  break;	  
	 }
	 
	 /*- ��������ٶȷֶμ�����ٶ� -*/
   SpeedPlanningReset(Axis);
   MaxSpeedSection(Axis);
	 /*-- �ж��ŷ����˶����� --*/
   DirectionJudge(Axis);		 
	 switch(Axis)
	 {
	 	 case X_Axsis: 
		  	  if( (Axsis_Move_Direction[Axis]!= NONE)&&(Servo_Pulse_Count[Axis]!=0) )
			  {
				  TIM_Cmd(TIM3, ENABLE);
				  Timer_Enable_X = ENABLE;
			  }
			  break;
		 
		 case Z_Axsis:
		  	  if( (Axsis_Move_Direction[Axis]!= NONE)&&(Servo_Pulse_Count[Axis]!=0) )
			  {
			      TIM_Cmd(TIM2, ENABLE);
		  	    Timer_Enable_Z = ENABLE;
			  }
			  break;

	 	 case L_Axsis: 		  //�ϲ���
		  	  if( (Axsis_Move_Direction[Axis]!= NONE)&&(Servo_Pulse_Count[Axis]!=0) )
			  {
			      TIM_Cmd(TIM1, ENABLE);
			      Timer_Enable_L = ENABLE;
			  }
			  break;
	 	 case O_Axsis: 		  //�ϲ���
		  	  if( (Axsis_Move_Direction[Axis]!= NONE)&&(Servo_Pulse_Count[Axis]!=0) )
			  {
			      TIM_Cmd(TIM4, ENABLE);
			      Timer_Enable_O = ENABLE;
			  }
			  break;			  				 
		 default:
		      break;		     	 
	 }
	  
	 
}



/**************************************************************************************************
**  ��������  SpeedPlanning()
**	�����������
**	�����������
**	�������ܣ��ɼӼ��ٶȺ������ٶ��Լ���ǰ�ٶ���ȷ�����ٶ�ֵ
**	��ע��	  			  
**  ���ߣ�    ����     
**  �������ڣ�2016/7/18 
***************************************************************************************************/
void  SpeedPlanning(u8 Axis)
{
     if( Error_Status != NO_ERROR)
	 {
	 	  if(Pulse_PresentHalfPeriod[Axis]>=180 && Pulse_PresentHalfPeriod[Axis]<=450)
		  {
		  	   Pulse_PresentAcceleration[Axis]=SpeedUp_Max_Acceleration[9];
		  }
		  else
		  {
		  	   Pulse_PresentAcceleration[Axis]=SpeedUp_Max_Acceleration[5];
		  }
	 }
	 else
	 {
          Period_Counter[Axis]++; 		  
		  /*----�ٶȼ��ٶȽ׶ν��й滮--------*/
		  if(Pulse_ControlStep[Axis] == SPEED_UP)
		  {			  
			  //�����ȼ��ٶκ�,��������ٽ׶�a=a-j
			  if( (Period_Counter_HoldA[Axis]!=0) &&(Pulse_Present_MaxSpeed[Axis]>NextSpeed[Axis])&& ((Pulse_Present_MaxSpeed[Axis]-NextSpeed[Axis])<=(Delta_Speed_Frequency[Axis]-Present_SpeedUp_Max_Acceleration[Axis])) )
			  {			  	   
				 Period_Counter_DownA[Axis]=Period_Counter[Axis]-(Period_Counter_UpA[Axis]+Period_Counter_HoldA[Axis]);
				 if(Pulse_PresentAcceleration[Axis]==Present_SpeedUp_Min_Acceleration[Axis])//<50
				 {	
                        Amin_Counter[Axis]++;          //���ٶδﵽ��С�ٶȵ�������							 
				 }
				 else
				 {
				    Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Max_Acceleration[Axis]-Period_Counter_DownA[Axis]*Present_SpeedUp_Jerk[Axis];
					if(Pulse_PresentAcceleration[Axis]<Present_SpeedUp_Min_Acceleration[Axis])
				    {
						Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Min_Acceleration[Axis];
					}
				 }
				 //Period_Counter_DownA=Period_Counter-(Period_Counter_UpA+Period_Counter_HoldA);
			  }
			  //�Ӽ��ٶȽ׶� �� a=a+j
			  else if((Pulse_Record_Flag[Axis]==FALSE)&&(Period_Counter_UpA[Axis]<=Period_Counter_Acceleration[Axis]))   
			  {	  	     
				 Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Min_Acceleration[Axis]+Period_Counter_UpA[Axis]*Present_SpeedUp_Jerk[Axis];		   
				 Period_Counter_UpA[Axis]=Period_Counter[Axis];
			   //�Ӽ��ٵ�����ٶ�ʱ,��¼�������
				 if( (Pulse_Record_Flag[Axis]==FALSE)&&(Pulse_PresentAcceleration[Axis]>=Present_SpeedUp_Max_Acceleration[Axis]) )
				 {
					  Pulse_Record_Flag[Axis]=TRUE;
					  Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Max_Acceleration[Axis];
                      Period_Counter_UpA[Axis]=Period_Counter[Axis]-1;
				 }
			  }
			  //�ȼ��ٶȽ׶� �� a=Amax
			  else if(Period_Counter_DownA[Axis]==0)   
			  {
		  	       Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Max_Acceleration[Axis];
				   Period_Counter_HoldA[Axis]=Period_Counter[Axis]-Period_Counter_UpA[Axis];
			  }
			  else
			  {
		 	  }

		  }
     /*----�ٶ����ٶȽ׶ν��й滮--�����ȥ������------*/
		  else if(Pulse_ControlStep[Axis] == SPEED_HOLD)
		  {
		      if(Period_Counter_DownA[Axis]>0)
			  {
			      Pulse_PresentAcceleration[Axis]=Pulse_PresentAcceleration[Axis]+Present_SpeedUp_Jerk[Axis];
			  }					  
		  }
     /*----�ٶȼ��ٽ׶ν��й滮--------*/
		  else if(Pulse_ControlStep[Axis] == SPEED_DOWN)
		  {			  
		      //�Ӽ��ٶȽ׶� �� a=a+j
			  if((Period_Counter_DownA[Axis]>0)&&(Period_Counter2[Axis]<=(Period_Counter_DownA[Axis]-1)) )
			  {					  	  						   
				   if( Period_Counter2[Axis]>Amin_Counter[Axis])
				   {
				   		Pulse_PresentAcceleration[Axis]=Pulse_PresentAcceleration[Axis]+Present_SpeedUp_Jerk[Axis];
				        if(Pulse_PresentAcceleration[Axis]>Present_SpeedUp_Max_Acceleration[Axis])
						{
						    Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Max_Acceleration[Axis];
						}
				   }
			  }
			  //�����ٶȽ׶� �� a=a-j
			  else if((Period_Counter_HoldA[Axis]==0)||
			          ((Period_Counter_HoldA[Axis]>0) && (Period_Counter2[Axis]>=(Period_Counter_HoldA[Axis]+Period_Counter_DownA[Axis]-1)))) 
			  {
				   Pulse_PresentAcceleration[Axis]=Pulse_PresentAcceleration[Axis]-Present_SpeedUp_Jerk[Axis];
				   if(Pulse_PresentAcceleration[Axis]<Present_SpeedUp_Min_Acceleration[Axis])
				   {  
				       Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Min_Acceleration[Axis];
				   }
			  }
			  //�ȼ��ٶȽ׶� �� a=Amax
			  else    
			  {
		  	  	   Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Max_Acceleration[Axis];
			  }	
			  Period_Counter2[Axis]++;	
		  }//else if(Pulse_ControlStep == SPEED_DOWN)
		  else
		  {
		  	 Pulse_PresentAcceleration[Axis]=SpeedUp_Max_Acceleration[0];		  
		  }
	 }
}

/**************************************************************************************************
**  ��������  ClosePulseReset()
**	�����������
**	�����������
**	�������ܣ����巢�ͽ�����λ��־λ
**	��ע��	  			  
**  ���ߣ�    ����     
**  �������ڣ�2016/7/18 
***************************************************************************************************/
void  ClosePulseReset(u8 Axis)
{
	 Pulse_ControlStep[Axis] = SPEED_UP;	
	 Pulse_Maxspeed_HalfPeriod[Axis] = 180;	
	 Pulse_PresentHalfPeriod[Axis] = 18000;
	 SpeedUp_Pulse_Counter[Axis] = 0;
	 Pulse_Counter[Axis] = 0;
	 Pulse_SpeedControlStep_Counter[Axis] = 0;
	 Pulse_InitialSpeed[Axis] = 1000;
	 NextSpeed[Axis]=0;		
	 Period_Counter[Axis]=0;
	 Period_Counter2[Axis]=0;
	 Amin_Counter[Axis]=0;
	 Pulse_Record_Flag[Axis]=FALSE;
	 Period_Counter_UpA[Axis]=0;
	 Period_Counter_HoldA[Axis]=0;
	 Period_Counter_DownA[Axis]=0;
	 Present_Pulse_SpeedControlStep_Counter[Axis]=0;
	 Servo_Stop_Done[Axis]=FALSE;   //�ŷ���ֹͣ������־λ��λ	 		
   Axsis_Move_Direction[Axis] = NONE;
}
/**************************************************************************************************
**  ��������  TIM3_IRQHandler()-X
**	�����������
**	�����������
**	�������ܣ���ʱ��3�жϺ�����X��������ƺ���
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void TIM3_IRQHandler(void)
{	    
    if(TIM_GetITStatus(TIM3,TIM_IT_Update) == SET)
    {    
         TIM_ClearITPendingBit(TIM3,TIM_IT_Update);	 
// 			 NextSpeed[X_Axsis] = 0;
			 switch(Pulse_ControlStep[X_Axsis])
			 {
				  case SPEED_UP:       
					   if( (u32)(Pulse_SpeedControlStep_Counter[X_Axsis]*Pulse_PresentHalfPeriod[X_Axsis]) >= SPEED_CONTROL_TIME )
					   {
						  Present_Pulse_SpeedControlStep_Counter[X_Axsis]=Pulse_SpeedControlStep_Counter[X_Axsis];
							Pulse_SpeedControlStep_Counter[X_Axsis] = 0;   
							SpeedPlanning(X_Axsis);		 //������ٶ�20160718	
							NextSpeed[X_Axsis] = Pulse_InitialSpeed[X_Axsis] + Pulse_PresentAcceleration[X_Axsis];  //�����ٶ�ֵ NextSpeed=fi  ===  Vi  
							if(NextSpeed[X_Axsis]>MAX_SPEED)
							{
								 NextSpeed[X_Axsis]=MAX_SPEED-Pulse_PresentAcceleration[X_Axsis];
							}
							Pulse_PresentHalfPeriod[X_Axsis] = 18000000 / NextSpeed[X_Axsis]; 
							Pulse_InitialSpeed[X_Axsis] = NextSpeed[X_Axsis];
							if(NextSpeed[X_Axsis]>=Pulse_Present_MaxSpeed[X_Axsis])
							{
						     SpeedUp_Pulse_Counter[X_Axsis] = Pulse_Counter[X_Axsis];                              
								 Pulse_ControlStep[X_Axsis] =  SPEED_HOLD;							
							}
					  }
					  if((Pulse_Counter[X_Axsis] + Pulse_Counter[X_Axsis] + Present_Pulse_SpeedControlStep_Counter[X_Axsis]+2) >= Servo_Pulse_Count[X_Axsis])
					  {  
							Pulse_ControlStep[X_Axsis] = SPEED_HOLD; 
							Remain_SpeedControlStep_Counter[X_Axsis]=Pulse_SpeedControlStep_Counter[X_Axsis];
							Pulse_SpeedControlStep_Counter[X_Axsis] = 0;
							SpeedUp_Pulse_Counter[X_Axsis] = Pulse_Counter[X_Axsis];				
					  }
					  break;

				 case SPEED_HOLD:			         
					  if(Pulse_SpeedControlStep_Counter[X_Axsis]>300)
					  {
					  	  Pulse_SpeedControlStep_Counter[X_Axsis]=0;
					  }
					  if(Servo_Pulse_Count[X_Axsis] - Pulse_Counter[X_Axsis] <= (SpeedUp_Pulse_Counter[X_Axsis]-Remain_SpeedControlStep_Counter[X_Axsis]+2) )
					  {				  						  						  
						  Pulse_SpeedControlStep_Counter[X_Axsis] = 0; 
						  Remain_SpeedControlStep_Counter[X_Axsis]=0;
						  //������һ�ν�����ٶ�
						  NextSpeed[X_Axsis]=NextSpeed[X_Axsis]-Pulse_PresentAcceleration[X_Axsis];  
						  Pulse_PresentHalfPeriod[X_Axsis] = 18000000 / NextSpeed[X_Axsis];
						  Pulse_InitialSpeed[X_Axsis] = NextSpeed[X_Axsis];
						  SpeedPlanning(X_Axsis);
						  Pulse_ControlStep[X_Axsis] = SPEED_DOWN; 
					  }
					  break;

				 case SPEED_DOWN:
					  if((u32)(Pulse_SpeedControlStep_Counter[X_Axsis] * Pulse_PresentHalfPeriod[X_Axsis]) >= SPEED_CONTROL_TIME)
					  {			   
							Pulse_SpeedControlStep_Counter[X_Axsis] = 0;
						    SpeedPlanning(X_Axsis);		 //������ٶ�20160718					  			
							NextSpeed[X_Axsis] = Pulse_InitialSpeed[X_Axsis] - Pulse_PresentAcceleration[X_Axsis];
							if(NextSpeed[X_Axsis]<Pulse_MinSpeed[X_Axsis])
							{
							    NextSpeed[X_Axsis]=Pulse_MinSpeed[X_Axsis];
							}
							Pulse_PresentHalfPeriod[X_Axsis] = 18000000 / NextSpeed[X_Axsis];
							Pulse_InitialSpeed[X_Axsis] = NextSpeed[X_Axsis];	
					  }
					  break;

				default:
					  break;
			 }  
			if(Pulse_PresentHalfPeriod[X_Axsis]>18000)
			{
				Pulse_PresentHalfPeriod[X_Axsis]=18000;
			}
			else if(Pulse_PresentHalfPeriod[X_Axsis]<180)
			{
				Pulse_PresentHalfPeriod[X_Axsis]=180;
			}
			SET_X_PULSE_PERIOD_HALF(Pulse_PresentHalfPeriod[X_Axsis]);
			Pulse_SpeedControlStep_Counter[X_Axsis]++;
			Pulse_Counter[X_Axsis]++;
			
			//��¼X�����������������
			if(Axsis_Move_Direction[X_Axsis] == POSITIVE)
			{ 
				 Pulse_TotalCounter[0] ++;
			}
			else
			{
				 Pulse_TotalCounter[0] --;
			}
			if(Error_Enable_Flag == TRUE)
			{					
				 CLOSE_X_PULSE;  
				 ClosePulseReset(X_Axsis);
				 Timer_Enable_X = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;	
			}		 

			/*=======================���������ɣ�ֹͣ���壬���������=========================*/
			if( Pulse_Counter[X_Axsis] == Servo_Pulse_Count[X_Axsis])
			{
				 CLOSE_X_PULSE; 
				 ClosePulseReset(X_Axsis);
				 Timer_Enable_X = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;							 
			 }	   	  
	}
}

/**************************************************************************************************
**  ��������  TIM2_IRQHandler()-Z
**	�����������
**	�����������
**	�������ܣ���ʱ��3�жϺ�����X��������ƺ���
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void TIM2_IRQHandler(void)
{	    
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)
    {    
             TIM_ClearITPendingBit(TIM2,TIM_IT_Update);	 
// 			 NextSpeed[Z_Axsis] = 0;
			 switch(Pulse_ControlStep[Z_Axsis])
			 {
				  case SPEED_UP:       
					   if( (u32)(Pulse_SpeedControlStep_Counter[Z_Axsis]*Pulse_PresentHalfPeriod[Z_Axsis]) >= SPEED_CONTROL_TIME )
					   {
						    Present_Pulse_SpeedControlStep_Counter[Z_Axsis]=Pulse_SpeedControlStep_Counter[Z_Axsis];
							Pulse_SpeedControlStep_Counter[Z_Axsis] = 0;   
							SpeedPlanning(Z_Axsis);		 //������ٶ�20160718	
							NextSpeed[Z_Axsis] = Pulse_InitialSpeed[Z_Axsis] + Pulse_PresentAcceleration[Z_Axsis];  //�����ٶ�ֵ NextSpeed=fi  ===  Vi  
							if(NextSpeed[Z_Axsis]>MAX_SPEED)
							{
								 NextSpeed[Z_Axsis]=MAX_SPEED-Pulse_PresentAcceleration[Z_Axsis];
							}
							Pulse_PresentHalfPeriod[Z_Axsis] = 18000000 / NextSpeed[Z_Axsis]; 
							Pulse_InitialSpeed[Z_Axsis] = NextSpeed[Z_Axsis];
							if(NextSpeed[Z_Axsis]>=Pulse_Present_MaxSpeed[Z_Axsis])
							{
						         SpeedUp_Pulse_Counter[Z_Axsis] = Pulse_Counter[Z_Axsis];                              
								 Pulse_ControlStep[Z_Axsis] =  SPEED_HOLD;							
							}
					  }
					  if((Pulse_Counter[Z_Axsis] + Pulse_Counter[Z_Axsis] + Present_Pulse_SpeedControlStep_Counter[Z_Axsis]+2) >= Servo_Pulse_Count[Z_Axsis])
					  {  
							Pulse_ControlStep[Z_Axsis] = SPEED_HOLD; 
							Remain_SpeedControlStep_Counter[Z_Axsis]=Pulse_SpeedControlStep_Counter[Z_Axsis];
							Pulse_SpeedControlStep_Counter[Z_Axsis] = 0;
							SpeedUp_Pulse_Counter[Z_Axsis] = Pulse_Counter[Z_Axsis];				
					  }
					  break;

				 case SPEED_HOLD:			         
					  if(Pulse_SpeedControlStep_Counter[Z_Axsis]>300)
					  {
					  	  Pulse_SpeedControlStep_Counter[Z_Axsis]=0;
					  }
					  if(Servo_Pulse_Count[Z_Axsis] - Pulse_Counter[Z_Axsis] <= (SpeedUp_Pulse_Counter[Z_Axsis]-Remain_SpeedControlStep_Counter[Z_Axsis]+2) )
					  {				  						  						  
						  Pulse_SpeedControlStep_Counter[Z_Axsis] = 0; 
						  Remain_SpeedControlStep_Counter[Z_Axsis]=0;
						  //������һ�ν�����ٶ�
						  NextSpeed[Z_Axsis]=NextSpeed[Z_Axsis]-Pulse_PresentAcceleration[Z_Axsis];  
						  Pulse_PresentHalfPeriod[Z_Axsis] = 18000000 / NextSpeed[Z_Axsis];
						  Pulse_InitialSpeed[Z_Axsis] = NextSpeed[Z_Axsis];
						  SpeedPlanning(Z_Axsis);
						  Pulse_ControlStep[Z_Axsis] = SPEED_DOWN; 
					  }
					  break;

				 case SPEED_DOWN:
					  if((u32)(Pulse_SpeedControlStep_Counter[Z_Axsis] * Pulse_PresentHalfPeriod[Z_Axsis]) >= SPEED_CONTROL_TIME)
					  {			   
							Pulse_SpeedControlStep_Counter[Z_Axsis] = 0;
						    SpeedPlanning(Z_Axsis);		 //������ٶ�20160718					  			
							NextSpeed[Z_Axsis] = Pulse_InitialSpeed[Z_Axsis] - Pulse_PresentAcceleration[Z_Axsis];
							if(NextSpeed[Z_Axsis]<Pulse_MinSpeed[Z_Axsis])
							{
							    NextSpeed[Z_Axsis]=Pulse_MinSpeed[Z_Axsis];
							}
							Pulse_PresentHalfPeriod[Z_Axsis] = 18000000 / NextSpeed[Z_Axsis];
							Pulse_InitialSpeed[Z_Axsis] = NextSpeed[Z_Axsis];	
					  }
					  break;

				default:
					  break;
			 }   
			if(Pulse_PresentHalfPeriod[Z_Axsis]>18000)
			{
				Pulse_PresentHalfPeriod[Z_Axsis]=18000;
			}
			else if(Pulse_PresentHalfPeriod[Z_Axsis]<180)
			{
				Pulse_PresentHalfPeriod[Z_Axsis]=180;
			}
			SET_Z_PULSE_PERIOD_HALF(Pulse_PresentHalfPeriod[Z_Axsis]);
			Pulse_SpeedControlStep_Counter[Z_Axsis]++;
			Pulse_Counter[Z_Axsis]++;
			
			//��¼Z�����������������
			if(Axsis_Move_Direction[Z_Axsis] == POSITIVE)
			{ 
				Pulse_TotalCounter[1] ++;
			}
			else
			{
				Pulse_TotalCounter[1] --;
			}
			if(Error_Enable_Flag == TRUE)
			{					
				 CLOSE_Z_PULSE;  
				 ClosePulseReset(Z_Axsis);
				 Timer_Enable_Z = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;	
			}		 

			/*=======================���������ɣ�ֹͣ���壬���������=========================*/
			if( Pulse_Counter[Z_Axsis] == Servo_Pulse_Count[Z_Axsis])
			{
				 CLOSE_Z_PULSE; 
				 ClosePulseReset(Z_Axsis);
				 Timer_Enable_Z = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;							 
			}		   	  
	}
}


/**************************************************************************************************
**  ��������  TIM1_UP_IRQChannel()-L
**	�����������
**	�����������
**	�������ܣ���ʱ��1�жϺ�����L��������ƺ���
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void TIM1_UP_IRQHandler(void)
{	    
    if(TIM_GetITStatus(TIM1,TIM_IT_Update) == SET)
    {    
             TIM_ClearITPendingBit(TIM1,TIM_IT_Update);	 
// 			 NextSpeed[L_Axsis] = 0;
			 switch(Pulse_ControlStep[L_Axsis])
			 {
				  case SPEED_UP:       
					   if( (u32)(Pulse_SpeedControlStep_Counter[L_Axsis]*Pulse_PresentHalfPeriod[L_Axsis]) >= SPEED_CONTROL_TIME )
					   {
						    Present_Pulse_SpeedControlStep_Counter[L_Axsis]=Pulse_SpeedControlStep_Counter[L_Axsis];
							Pulse_SpeedControlStep_Counter[L_Axsis] = 0;   
							SpeedPlanning(L_Axsis);		 //������ٶ�20160718	
							NextSpeed[L_Axsis] = Pulse_InitialSpeed[L_Axsis] + Pulse_PresentAcceleration[L_Axsis];  //�����ٶ�ֵ NextSpeed=fi  ===  Vi  
							if(NextSpeed[L_Axsis]>MAX_SPEED)
							{
								 NextSpeed[L_Axsis]=MAX_SPEED-Pulse_PresentAcceleration[L_Axsis];
							}
							Pulse_PresentHalfPeriod[L_Axsis] = 18000000 / NextSpeed[L_Axsis]; 
							Pulse_InitialSpeed[L_Axsis] = NextSpeed[L_Axsis];
							if(NextSpeed[L_Axsis]>=Pulse_Present_MaxSpeed[L_Axsis])
							{
						         SpeedUp_Pulse_Counter[L_Axsis] = Pulse_Counter[L_Axsis];                              
								 Pulse_ControlStep[L_Axsis] =  SPEED_HOLD;							
							}
					  }
					  if((Pulse_Counter[L_Axsis] + Pulse_Counter[L_Axsis] + Present_Pulse_SpeedControlStep_Counter[L_Axsis]+2) >= Servo_Pulse_Count[L_Axsis])
					  {  
							Pulse_ControlStep[L_Axsis] = SPEED_HOLD; 
							Remain_SpeedControlStep_Counter[L_Axsis]=Pulse_SpeedControlStep_Counter[L_Axsis];
							Pulse_SpeedControlStep_Counter[L_Axsis] = 0;
							SpeedUp_Pulse_Counter[L_Axsis] = Pulse_Counter[L_Axsis];				
					  }
					  break;

				 case SPEED_HOLD:			         
					  if(Pulse_SpeedControlStep_Counter[L_Axsis]>300)
					  {
					  	  Pulse_SpeedControlStep_Counter[L_Axsis]=0;
					  }
					  if(Servo_Pulse_Count[L_Axsis] - Pulse_Counter[L_Axsis] <= (SpeedUp_Pulse_Counter[L_Axsis]-Remain_SpeedControlStep_Counter[L_Axsis]+2) )
					  {				  						  						  
						  Pulse_SpeedControlStep_Counter[L_Axsis] = 0; 
						  Remain_SpeedControlStep_Counter[L_Axsis]=0;
						  //������һ�ν�����ٶ�
						  NextSpeed[L_Axsis]=NextSpeed[L_Axsis]-Pulse_PresentAcceleration[L_Axsis];  
						  Pulse_PresentHalfPeriod[L_Axsis] = 18000000 / NextSpeed[L_Axsis];
						  Pulse_InitialSpeed[L_Axsis] = NextSpeed[L_Axsis];
						  SpeedPlanning(L_Axsis);
						  Pulse_ControlStep[L_Axsis] = SPEED_DOWN; 
					  }
					  break;

				 case SPEED_DOWN:
					  if((u32)(Pulse_SpeedControlStep_Counter[L_Axsis] * Pulse_PresentHalfPeriod[L_Axsis]) >= SPEED_CONTROL_TIME)
					  {			   
							Pulse_SpeedControlStep_Counter[L_Axsis] = 0;
						    SpeedPlanning(L_Axsis);		 //������ٶ�20160718					  			
							NextSpeed[L_Axsis] = Pulse_InitialSpeed[L_Axsis] - Pulse_PresentAcceleration[L_Axsis];
							if(NextSpeed[L_Axsis]<Pulse_MinSpeed[L_Axsis])
							{
							    NextSpeed[L_Axsis]=Pulse_MinSpeed[L_Axsis];
							}
							Pulse_PresentHalfPeriod[L_Axsis] = 18000000 / NextSpeed[L_Axsis];
							Pulse_InitialSpeed[L_Axsis] = NextSpeed[L_Axsis];	
					  }
					  break;

				default:
					  break;
			 } 
			if(Pulse_PresentHalfPeriod[L_Axsis]>18000)
			{
				Pulse_PresentHalfPeriod[L_Axsis]=18000;
			}
			else if(Pulse_PresentHalfPeriod[L_Axsis]<180)
			{
				Pulse_PresentHalfPeriod[L_Axsis]=180;
			}
			SET_L_PULSE_PERIOD_HALF(Pulse_PresentHalfPeriod[L_Axsis]);
			Pulse_SpeedControlStep_Counter[L_Axsis]++;
			Pulse_Counter[L_Axsis]++;
			
			//��¼Z�����������������
			if(Axsis_Move_Direction[L_Axsis] == POSITIVE)
			{ 
				Pulse_TotalCounter[2] ++;
			}
			else
			{
				Pulse_TotalCounter[2] --;
			}
			if(Error_Enable_Flag == TRUE)
			{					
				 CLOSE_L_PULSE;  
				 ClosePulseReset(L_Axsis);
				 Timer_Enable_L = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;	
			}		 

			/*=======================���������ɣ�ֹͣ���壬���������=========================*/
			if( Pulse_Counter[L_Axsis] == Servo_Pulse_Count[L_Axsis])
			{
				 CLOSE_L_PULSE; 
				 ClosePulseReset(L_Axsis);
				 Timer_Enable_L = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;							 
			}	   	  
	}
}

/**************************************************************************************************
**  ��������  TIM4_UP_IRQChannel()-O
**	�����������
**	�����������
**	�������ܣ���ʱ��4�жϺ�����O��������ƺ���
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2016/11/26 
***************************************************************************************************/
void TIM4_IRQHandler(void)
{	    
    if(TIM_GetITStatus(TIM4,TIM_IT_Update) == SET)
    {    
             TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	 
// 			 NextSpeed[O_Axsis] = 0;
			 switch(Pulse_ControlStep[O_Axsis])
			 {
				  case SPEED_UP:       
					   if( (u32)(Pulse_SpeedControlStep_Counter[O_Axsis]*Pulse_PresentHalfPeriod[O_Axsis]) >= SPEED_CONTROL_TIME )
					   {
						    Present_Pulse_SpeedControlStep_Counter[O_Axsis]=Pulse_SpeedControlStep_Counter[O_Axsis];
							Pulse_SpeedControlStep_Counter[O_Axsis] = 0;   
							SpeedPlanning(O_Axsis);		 //������ٶ�20160718	
							NextSpeed[O_Axsis] = Pulse_InitialSpeed[O_Axsis] + Pulse_PresentAcceleration[O_Axsis];  //�����ٶ�ֵ NextSpeed=fi  ===  Vi  
							if(NextSpeed[O_Axsis]>MAX_SPEED)
							{
								 NextSpeed[O_Axsis]=MAX_SPEED-Pulse_PresentAcceleration[O_Axsis];
							}
							Pulse_PresentHalfPeriod[O_Axsis] = 18000000 / NextSpeed[O_Axsis]; 
							Pulse_InitialSpeed[O_Axsis] = NextSpeed[O_Axsis];
							if(NextSpeed[O_Axsis]>=Pulse_Present_MaxSpeed[O_Axsis])
							{
						         SpeedUp_Pulse_Counter[O_Axsis] = Pulse_Counter[O_Axsis];                              
								 Pulse_ControlStep[O_Axsis] =  SPEED_HOLD;							
							}
					  }
					  if((Pulse_Counter[O_Axsis] + Pulse_Counter[O_Axsis] + Present_Pulse_SpeedControlStep_Counter[O_Axsis]+2) >= Servo_Pulse_Count[O_Axsis])
					  {  
							Pulse_ControlStep[O_Axsis] = SPEED_HOLD; 
							Remain_SpeedControlStep_Counter[O_Axsis]=Pulse_SpeedControlStep_Counter[O_Axsis];
							Pulse_SpeedControlStep_Counter[O_Axsis] = 0;
							SpeedUp_Pulse_Counter[O_Axsis] = Pulse_Counter[O_Axsis];				
					  }
					  break;

				 case SPEED_HOLD:			         
					  if(Pulse_SpeedControlStep_Counter[O_Axsis]>300)
					  {
					  	  Pulse_SpeedControlStep_Counter[O_Axsis]=0;
					  }
					  if(Servo_Pulse_Count[O_Axsis] - Pulse_Counter[O_Axsis] <= (SpeedUp_Pulse_Counter[O_Axsis]-Remain_SpeedControlStep_Counter[O_Axsis]+2) )
					  {				  						  						  
						  Pulse_SpeedControlStep_Counter[O_Axsis] = 0; 
						  Remain_SpeedControlStep_Counter[O_Axsis]=0;
						  //������һ�ν�����ٶ�
						  NextSpeed[O_Axsis]=NextSpeed[O_Axsis]-Pulse_PresentAcceleration[O_Axsis];  
						  Pulse_PresentHalfPeriod[O_Axsis] = 18000000 / NextSpeed[O_Axsis];
						  Pulse_InitialSpeed[O_Axsis] = NextSpeed[O_Axsis];
						  SpeedPlanning(O_Axsis);
						  Pulse_ControlStep[O_Axsis] = SPEED_DOWN; 
					  }
					  break;

				 case SPEED_DOWN:
					  if((u32)(Pulse_SpeedControlStep_Counter[O_Axsis] * Pulse_PresentHalfPeriod[O_Axsis]) >= SPEED_CONTROL_TIME)
					  {			   
							Pulse_SpeedControlStep_Counter[O_Axsis] = 0;
						    SpeedPlanning(O_Axsis);		 //������ٶ�20160718					  			
							NextSpeed[O_Axsis] = Pulse_InitialSpeed[O_Axsis] - Pulse_PresentAcceleration[O_Axsis];
							if(NextSpeed[O_Axsis]<Pulse_MinSpeed[O_Axsis])
							{
							    NextSpeed[O_Axsis]=Pulse_MinSpeed[O_Axsis];
							}
							Pulse_PresentHalfPeriod[O_Axsis] = 18000000 / NextSpeed[O_Axsis];
							Pulse_InitialSpeed[O_Axsis] = NextSpeed[O_Axsis];	
					  }
					  break;

				default:
					  break;
			 }
			if(Pulse_PresentHalfPeriod[O_Axsis]>18000)
			{
				Pulse_PresentHalfPeriod[O_Axsis]=18000;
			}
			else if(Pulse_PresentHalfPeriod[O_Axsis]<180)
			{
				Pulse_PresentHalfPeriod[O_Axsis]=180;
			}
			SET_O_PULSE_PERIOD_HALF(Pulse_PresentHalfPeriod[O_Axsis]);
			Pulse_SpeedControlStep_Counter[O_Axsis]++;
			Pulse_Counter[O_Axsis]++;
			
			//��¼Z�����������������
			if(Axsis_Move_Direction[O_Axsis] == POSITIVE)
			{ 
				Pulse_TotalCounter[3]++;
			}
			else
			{
				Pulse_TotalCounter[3]--;
			}
			if(Error_Enable_Flag == TRUE)
			{					
				 CLOSE_O_PULSE;  
				 ClosePulseReset(O_Axsis);
				 Timer_Enable_O = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;	
			}		 

			/*=======================���������ɣ�ֹͣ���壬���������=========================*/
			if( Pulse_Counter[O_Axsis] == Servo_Pulse_Count[O_Axsis])
			{
				 CLOSE_O_PULSE; 
				 ClosePulseReset(O_Axsis);
				 Timer_Enable_O = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;							 
			}	   	  
	}
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
