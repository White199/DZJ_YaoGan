//---2017.7.31
#include "CANopen.h"
#include "Homing.h"
#include "StatusControl.h"

vu8 Flag_ARM_Homing = 0;				 //��ԭ���ʶ
u8 Flag_Led = 0;
u8 Flag_Homing_Switch = 0;
u8 Flag_Homing_Switch_S=0;
//---2017.9.27
vu8 Flag_ARM_Homing_x=0;         //x������־
vu8 Flag_ARM_Homing_z=0;         //z������־
vu8 Flag_ARM_Homing_o=0;         //o������־
vu8 Flag_ARM_Homing_l=0;         //L������־
vu8 Flag_ARM_Homing_a=0;         //A������־
vu8 Flag_ARM_Homing_b=0;         //B������־

/***************************************************************************
**  ��������  ARM_Homing()
**	�����������
**	�����������
**	�������ܣ���е�ۻ�ԭ��
**	��ע��	  ����method 21 ����Ƭ����� Reset->Set->Reset��
			  �ⲿ����뵥Ƭ��Ϊ�෴��ϵ Set->Reset->Set
**  ���ߣ�    zhang   
**  �������ڣ�2017/07/31
***************************************************************************/
void ARM_Homing(u8 Servo_Node_ID)
{
	switch(Servo_Node_ID)
	{
		case X_Axsis:
			if(Homing_attained_status[X_Axsis]==0&&Flag_ARM_Homing_x==0)
			{
			  Flag_ARM_Homing_x = 1;					
			  Homing_Operation_Start_Reset(X_Axsis+1);
			  Delay(10);
			  Homing_Operation_Start_Set(X_Axsis+1);
			}			
			break;
		case Z_Axsis:
			if(Homing_attained_status[Z_Axsis]==0&&Flag_ARM_Homing_z==0)
			{
			  Flag_ARM_Homing_z=1;
			  Homing_Operation_Start_Reset(Z_Axsis+1);
			  Delay(10);
			  Homing_Operation_Start_Set(Z_Axsis+1); 
			}			
			break;
		case L_Axsis:
			if(Homing_attained_status[L_Axsis]==0&&Flag_ARM_Homing_l==0)
			{
			  Flag_ARM_Homing_l = 1;					
			  Homing_Operation_Start_Reset(L_Axsis+1);
			  Delay(10);
			  Homing_Operation_Start_Set(L_Axsis+1);
			  Homing_attained_status[L_Axsis]=0;					
			}			
			break;
		case O_Axsis:
			if(Homing_attained_status[O_Axsis]==0&&Flag_ARM_Homing_o==0)
			{
			  Flag_ARM_Homing_o = 1;					
			  Homing_Operation_Start_Reset(O_Axsis+1);
			  Delay(10);
			  Homing_Operation_Start_Set(O_Axsis+1);
			}			
			break;
		case A_Axsis:
			if(Homing_attained_status[A_Axsis]==0&&Flag_ARM_Homing_a==0)
			{
			  Flag_ARM_Homing_a = 1;					
			  Homing_Operation_Start_Reset(A_Axsis+1);
			  Delay(10);
			  Homing_Operation_Start_Set(A_Axsis+1);
			}			
			break;
		case B_Axsis:
			if(Homing_attained_status[B_Axsis]==0&&Flag_ARM_Homing_b==0)
			{
			  Flag_ARM_Homing_b = 1;					
			  Homing_Operation_Start_Reset(B_Axsis+1);
			  Delay(10);
			  Homing_Operation_Start_Set(B_Axsis+1);
			}			
			break;
		default:
			break;
	}

}
