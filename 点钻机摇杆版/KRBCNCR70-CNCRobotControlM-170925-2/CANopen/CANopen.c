/******************** COPYRIGHT 2013 KingRobot *********************************
* File Name          : CANopen.c
* Author             : WuXiang
* Version            : V1.0.0
* Date               : 11/14/2013
* Description        : This file complete TPDO and RPDO mapping
********************************************************************************/

#include "CANopen.h"

State_Machine_state  State_Machine;      //״̬��:Initialisation/Pre_Operate/Operate/Stop
Function             Function_Code;      //������
u8   ServoNodeID_0 = 0 ;                 //�ڵ�IDΪ0ʱ,���й㲥
u8   ServoNodeID_1 = 1 ;                 //�ڵ�1��ID��
u8   ServoNodeID_2 = 0 ;                 //�ڵ�2��ID��
u8   ServoNodeID_3 = 0 ;                 //�ڵ�3��ID��
u8   ServoNodeID_4 = 0 ;                 //�ڵ�4��ID��
u8   ServoNodeID_5 = 0 ;                 //�ڵ�5��ID��
u8   ServoNodeID_6 = 0 ;                 //�ڵ�6��ID��
u8   Flag_Reached_Point=0;
u8   Reached=0;
u8   Set_Point_Acknowledge = 0;
u8   Change_Velocity=0;
u8   Flag_Chang_Operation_Mode = 0 ;

vu8  Servo_Node_ID = 0;                  //�ŷ����ڵ�ID�ű���
vu8  CAN_Recieve_Data[8] ={0} ;
vu8  Servo_Operate_Mode = 6 ;			 //�ŷ�������ģʽ
vu16 Controlword = 0x0000 ;              //״̬��
vu16 COB_ID = 0;                         //Communication Object ID
vu16 Command_Specifier;                  //cs:�����
u32  CANopen_k = 0;
vu32 Target_Servo_Position=0;            //�ŷ�����˶���Ŀ��λ��ֵ
vu32 Target_Servo_Velocity=0;            //�ŷ�����˶���Ŀ���ٶ�ֵ
vu32 Target_Servo_Acceleration=200;      //�ŷ�����˶��ļ��ٶ�ֵ
vu32 Target_Servo_Deceleration=200;      //�ŷ�����˶��ļ��ٶ�ֵ


/***************************************************************************
**  ��������  CANopen_Init()
**	�����������
**	�����������
**	�������ܣ���ʼ���ŷ���CANopenͨ��
**	��ע��	  ��
**  ���ߣ�    ����    
**  �������ڣ�2013/11/14
***************************************************************************/
void CANopen_Init()
{
    /*****ʹ�õ��Ľڵ������ʹ�ܽڵ��******/
	ServoNodeID_0 = SERVO_NODE_ID_00 ;     //broadcast
	ServoNodeID_1 = SERVO_NODE_ID_01 ;
	ServoNodeID_2 = SERVO_NODE_ID_02 ;		
	ServoNodeID_3 = SERVO_NODE_ID_03 ;
	ServoNodeID_4 = SERVO_NODE_ID_04 ;
	ServoNodeID_5 = SERVO_NODE_ID_05 ;
	ServoNodeID_6 = SERVO_NODE_ID_06 ;
	
	Enter_Pre_Operational_State(ServoNodeID_0);
     /*��ʼ�����ŷ���1������operate״̬*/
	 if(ServoNodeID_1)
	 {       
		  SDO_Process(ServoNodeID_1);          //��ʼ���ŷ����ڵ��Լ�������ز���                                                                               
		  Start_Remote_Node(ServoNodeID_1);    //����Զ�̽ڵ㣬����operateģʽ
	 }

     /*��ʼ�����ŷ���2������operate״̬*/
     if(ServoNodeID_2)
     {
          SDO_Process(ServoNodeID_2);          //��ʼ���ŷ����ڵ��Լ�������ز���                                                                               
          Start_Remote_Node(ServoNodeID_2);    //����Զ�̽ڵ㣬����operateģʽ
     }

     /*��ʼ�����ŷ���3������operate״̬*/
     if(ServoNodeID_3)
     {
          SDO_Process(ServoNodeID_3);          //��ʼ���ŷ����ڵ��Լ�������ز���                                                                               
          Start_Remote_Node(ServoNodeID_3);    //����Զ�̽ڵ㣬����operateģʽ
     }

     /*��ʼ�����ŷ���4������operate״̬*/
     if(ServoNodeID_4)
     {
          SDO_Process(ServoNodeID_4);          //��ʼ���ŷ����ڵ��Լ�������ز���                                                                               
          Start_Remote_Node(ServoNodeID_4);    //����Զ�̽ڵ㣬����operateģʽ
     }

     /*��ʼ�����ŷ���5������operate״̬*/
     if(ServoNodeID_5)
     {
          SDO_Process(ServoNodeID_5);          //��ʼ���ŷ����ڵ��Լ�������ز���                                                                               
          Start_Remote_Node(ServoNodeID_5);    //����Զ�̽ڵ㣬����operateģʽ
     }

     /*��ʼ�����ŷ���6������operate״̬*/
     if(ServoNodeID_6)
     {
          SDO_Process(ServoNodeID_6);          //��ʼ���ŷ����ڵ��Լ�������ز���                                                                               
          Start_Remote_Node(ServoNodeID_6);    //����Զ�̽ڵ㣬����operateģʽ
     }
}

/***************************************************************************
**  ��������  Chang_Operation_Mode()
**	�����������
**	�����������
**	�������ܣ��޸��ŷ�������ģʽ
**	��ע��	  ��
**  ���ߣ�    ����    
**  �������ڣ�2013/11/14
***************************************************************************/
void Chang_Operation_Mode(u8 Node_ID, u8 OperationMode)
{
	/*
      SDO_Message.Node_ID   = Node_ID;
      SDO_Message.index     = 0x6040;
      SDO_Message.sub_index = 0x00;
      SDO_Message.len       = 2;
      SDO_Message.Data[0]   = 0x0f;   //0x000F
      SDO_Message.Data[1]   = 0x00;;      
      SDO_Protocol(INITIATE_SDO_DOWNLOAD);
      Delay(20); 
	 */ 
	  SDO_Message.Node_ID   = Node_ID;
      SDO_Message.index     = 0x6060;
      SDO_Message.sub_index = 0x00;
      SDO_Message.len       = 1;
      SDO_Message.Data[0]   = OperationMode;       
      SDO_Protocol(INITIATE_SDO_DOWNLOAD);
      Delay(20);	 
}

/***********************************************
 *       RPDO_Mapping��RPDOӳ��                *
 *       RPDO��1400~1403                       *
 *       RPDO MapPara��1600~1603               *
 ***********************************************/


/*********RPDO mapping deleted***********
 *  For changing PDO mapping first      *
 *  the PDO has to be deleted           *
 *  the sub-index 0 must be set to 0    *
 ****************************************/
void PDO_Mapping_Deleted(u8 Node_ID)
{
    /*** index: 1600h***/
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1600;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0x00;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    Delay(20); 

    /*** index: 1601h***/
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1601;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0x00;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    Delay(20); 
}

/*****************************************/
/*  Ĭ������ӳ�䣺1400��0x8000027f       */
/*                1401��0x8000037f       */
/*  ���ڲ���Ĭ��RPDO Mapping             */
/*****************************************/
void PDO_Mapping_test(u8 Node_ID)
{
    
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1400;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00+Node_ID; //1400h: COB_ID:200h+Node_ID
    SDO_Message.Data[1]   = 0x02;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x00;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    Delay(50);    
    
       
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1401;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00+Node_ID; //1401h: COB_ID:300h+Node_ID
    SDO_Message.Data[1]   = 0x03;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x00;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    Delay(50);

}

void Not_Valid_PDO(u8 Node_ID)
{   
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1400;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00;  
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x80;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    Delay(20);

    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1401;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00;  
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x80;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    Delay(20);

    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1402;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00;  
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x80;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    Delay(20);
    
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1800;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00;  
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x80;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    Delay(20);

}

/***************************************************************************
**  ��������  RPDO_Mapping()
**	�����������
**	�����������
**	�������ܣ�����PDOӳ�䣨����ڴ�վ-�ŷ������ԣ����ŷ�������PDO��Ϣ��
**	��ע��	  ��
**  ���ߣ�    ����    
**  �������ڣ�2013/11/14
***************************************************************************/
/*******************************************************
 *  RPDO_Mapping��PDOӳ��                              *
 *  RPDO1��1400-1600:6040 00 01h  COB-ID:200h+Node-ID  *
 *  RPDO2: 1401-1601:607A 00 20h  COB-ID:300h+Node-ID  *
 *                   6081 00 20h                       *
 *  RPDO3: 1402-1602:6083 00 20h  COB-ID:400h+Node-ID  *
 *                   6084 00 20h                       *
 *  6040h:Control Word            Default:0x0000       *
 *  607Ah:Target Position         Default:0            *
 *  6081h:Profile Velocity        Default:10000        *
 *  6083h:Profile acceleration    Default:200          *
 *  6084h:Profile deceleration    Default:200          *
 *******************************************************/
void RPDO_Mapping(u8 Node_ID)
{    
 //------------Not valid RPDO--------------//     
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1400;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00;  
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x80;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//---
	Delay(10);

    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1401;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00;  
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x80;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
 //---
	Delay(10);
 
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1402;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00;  
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x80;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
  //---
	Delay(10);
 //------------Not valid RPDO--------------//    
   
   
    /************************************
     * RPDO Communication Parameter set *         
     ************************************/    
 //------------index: 1400h------------------//
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1400;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00+Node_ID;  //1400h: COB_ID:200h+Node_ID
    SDO_Message.Data[1]   = 0x02;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x00;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
  //---
	Delay(10);

    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1400;
    SDO_Message.sub_index = 0x02;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0xff; //---�¼����� ��301  ��124ҳ
    SDO_Protocol(INITIATE_SDO_DOWNLOAD); 
  //---
	Delay(10);		
 //------------index: 1400h------------------//


 //------------index: 1401h------------------//
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1401;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00+Node_ID; //1401h: COB_ID:300h+Node_ID
    SDO_Message.Data[1]   = 0x03;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x00;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
  //---
	Delay(10);

    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1401;
    SDO_Message.sub_index = 0x02;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0xff;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD); 
	//---
	Delay(10);
 //------------index: 1401h------------------//


 //------------index: 1402h------------------//
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1402;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00+Node_ID; //1402h: COB_ID:400h+Node_ID
    SDO_Message.Data[1]   = 0x04;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x00;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
  //---
	Delay(10);

    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1402;
    SDO_Message.sub_index = 0x02;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0xff;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD); 
  //---
	Delay(10);
 //------------index: 1402h------------------//


    /***********************************
     *   RPDO Mapping Parameter set    *         
     ***********************************/
 //------------index: 1600h------------------//
    SDO_Message.Node_ID   = Node_ID;   
    SDO_Message.index     = 0x1600;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0x00;          //PDO mapping deleted
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//---
	Delay(10);

    /***������01h:6040h 00 10h ***/
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1600;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x10;
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x40;
    SDO_Message.Data[3]   = 0x60;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//---
	Delay(10);

    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1600;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0x01;         //PDO mapping 
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//---
	Delay(10);
 //------------index: 1600h------------------//


 //------------index: 1601h------------------//
    SDO_Message.Node_ID   = Node_ID;    
    SDO_Message.index     = 0x1601;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0x00;        //PDO mapping deleted
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//---
	Delay(10);

    /***������01h:607Ah 00 20h ***/
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1601;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x20;
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x7A;
    SDO_Message.Data[3]   = 0x60;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//---
	Delay(10);
 
    /***������02h:6081h 00 20h ***/
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1601;
    SDO_Message.sub_index = 0x02;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x20;
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x81;
    SDO_Message.Data[3]   = 0x60;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//---
	Delay(10);

    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1601;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0x02;        //PDO mapping 
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//---
	Delay(10);
 //------------index: 1601h------------------//


 //------------index: 1602h------------------//
    SDO_Message.Node_ID   = Node_ID;    
    SDO_Message.index     = 0x1602;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0x00;        //PDO mapping deleted
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//---
	Delay(10);

    /***������01h:6083h 00 20h ***/
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1602;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x20;
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x83;
    SDO_Message.Data[3]   = 0x60;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//---
	Delay(10);

    /***������02h:6084h 00 20h ***/
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1602;
    SDO_Message.sub_index = 0x02;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x20;
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x84;
    SDO_Message.Data[3]   = 0x60;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
  //---
	Delay(10);

    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1602;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0x02;        //PDO mapping 
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//---
	Delay(10);
 //------------index: 1602h------------------//
}

/***************************************************************************
**  ��������  TPDO_Mapping()
**	�����������
**	�����������
**	�������ܣ�����PDOӳ�䣨����ڴ�վ-�ŷ������ԣ����ŷ�������PDO��Ϣ��
**	��ע��	  ��
**  ���ߣ�    ����    
**  �������ڣ�2013/11/14
***************************************************************************/
/*******************************************************
 *  TPDO_Mapping��PDOӳ��                              *
 *  TPDO1��1800-1A00:6041 00 10h  COB-ID:180h+Node-ID  *
 *                  :6064 00 20h                       *
 *  6041h:status Word             Default:0            *
 *  6064h:Position actual value   Default:0            *
 *******************************************************/
//----ͨ��ͨ�ŷ�180+ID ����6041��6064����Ϣ��λ��ģʽ�� ���Զ�ȡ��ص���Ϣ��
void TPDO_Mapping(u8 Node_ID)
{ 
 //------------Not valid TPDO--------------//      
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1800;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00;  
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x80;//----�ر�PDO����
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
  //--  
	Delay(10);     

    /************************************
     * TPDO Communication Parameter set *         
     ************************************/         
 //------------index: 1800h------------------//      
  /*
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1800;
    SDO_Message.sub_index = 0x03;
    SDO_Message.len       = 2;
    SDO_Message.Data[0]   = 0x64;  
    SDO_Message.Data[1]   = 0x00;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    */ 
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1800;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x80+Node_ID;  //180h: COB_ID:180h+Node_ID
    SDO_Message.Data[1]   = 0x01;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x00;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//--  
	Delay(10);  
    
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1800;
    SDO_Message.sub_index = 0x02;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0xff;  //---��������
    SDO_Protocol(INITIATE_SDO_DOWNLOAD); 
  //--  
	Delay(10);  		

    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1800;
    SDO_Message.sub_index = 0x05;
    SDO_Message.len       = 2;
    SDO_Message.Data[0]   = 0x1E;  
    SDO_Message.Data[1]   = 0x00;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
  //--  
	Delay(10);  		
    
 //------------index: 1800h------------------//


    /***********************************
     *   TPDO Mapping Parameter set    *         
     ***********************************/
 //------------index: 1A00h------------------//
    SDO_Message.Node_ID   = Node_ID;    
    SDO_Message.index     = 0x1A00;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0x00;        //PDO mapping deleted
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//--  
	Delay(10);  

    /***������01h:6041h 00 10h ***/
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1A00;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;  
    SDO_Message.Data[0]   = 0x10;
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x41;
    SDO_Message.Data[3]   = 0x60;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//--  
	Delay(10);  

    /***������02h:6064h 00 20h ***/
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1A00;
    SDO_Message.sub_index = 0x02;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x20;
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x64;
    SDO_Message.Data[3]   = 0x60;
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//--  
	Delay(10);  

    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x1A00;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0x02;        //PDO mapping ---02��ʾ������01��02��Ч����301��134ҳ
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	//--  
	Delay(10);  
    
 //------------index: 1A00h------------------//
}

/***********************************************
 *    2017.8.15                *
 ***********************************************/
void PP_Mode_Init()
{
	   /*****ʹ�õ��Ľڵ������ʹ�ܽڵ��******/
		ServoNodeID_0 = SERVO_NODE_ID_00 ;                 //broadcast
		ServoNodeID_1 = SERVO_NODE_ID_01 ;
		ServoNodeID_2 = SERVO_NODE_ID_02 ;		
		ServoNodeID_3 = SERVO_NODE_ID_03 ;
		ServoNodeID_4 = SERVO_NODE_ID_04 ;
		ServoNodeID_5 = SERVO_NODE_ID_05 ;
		ServoNodeID_6 = SERVO_NODE_ID_06 ;
	
     Enter_Pre_Operational_State(ServoNodeID_0);
	   /*��ʼ�����ŷ���1������operate״̬*/
     if(ServoNodeID_1)
     {    
          Servo_Node_ID = ServoNodeID_1;			 
          SDO_Process_PositionMode(Servo_Node_ID);      //��ʼ���ŷ����ڵ��Լ�������ز��� 		 
          Start_Remote_Node(Servo_Node_ID);             //����Զ�̽ڵ㣬����operateģʽ
		  Servo_Node_ID = 0;
     }

     /*��ʼ�����ŷ���2������operate״̬*/
     if(ServoNodeID_2)
     {
          Servo_Node_ID = ServoNodeID_2;	
		  SDO_Process_PositionMode(Servo_Node_ID);      //��ʼ���ŷ����ڵ��Լ�������ز��� 	                                                                            
          Start_Remote_Node(Servo_Node_ID);             //����Զ�̽ڵ㣬����operateģʽ
		  Servo_Node_ID = 0;
     }

     /*��ʼ�����ŷ���3������operate״̬*/
     if(ServoNodeID_3)
     {
          Servo_Node_ID = ServoNodeID_3;
		  SDO_Process_PositionMode(Servo_Node_ID);      //��ʼ���ŷ����ڵ��Լ�������ز���                                                                                
          Start_Remote_Node(Servo_Node_ID);             //����Զ�̽ڵ㣬����operateģʽ
		  Servo_Node_ID = 0; 
     }

     /*��ʼ�����ŷ���4������operate״̬*/
     if(ServoNodeID_4)
     {
          Servo_Node_ID = ServoNodeID_4; 
		  SDO_Process_PositionMode(Servo_Node_ID);      //��ʼ���ŷ����ڵ��Լ�������ز���                                                                              
          Start_Remote_Node(Servo_Node_ID);             //����Զ�̽ڵ㣬����operateģʽ
		  Servo_Node_ID = 0; 
     }
     /*��ʼ�����ŷ���5������operate״̬*/
     if(ServoNodeID_5)
     {
          Servo_Node_ID = ServoNodeID_5; 
		  SDO_Process_PositionMode(Servo_Node_ID);      //��ʼ���ŷ����ڵ��Լ�������ز���                                                                              
          Start_Remote_Node(Servo_Node_ID);             //����Զ�̽ڵ㣬����operateģʽ
		  Servo_Node_ID = 0; 
     }

     /*��ʼ�����ŷ���6������operate״̬*/
     if(ServoNodeID_6)
     {
          Servo_Node_ID = ServoNodeID_6; 
		  SDO_Process_PositionMode(Servo_Node_ID);      //��ʼ���ŷ����ڵ��Լ�������ز���                                                                              
          Start_Remote_Node(Servo_Node_ID);             //����Զ�̽ڵ㣬����operateģʽ
		  Servo_Node_ID = 0; 
     }
}
/***********************************************
 *    2017.8.16               *
 ***********************************************/
void Homing_Mode_Init()
{
	   /*****ʹ�õ��Ľڵ������ʹ�ܽڵ��******/
		ServoNodeID_0 = SERVO_NODE_ID_00 ;                 //broadcast
		ServoNodeID_1 = SERVO_NODE_ID_01 ;
		 ServoNodeID_2 = SERVO_NODE_ID_02 ;		
		 ServoNodeID_3 = SERVO_NODE_ID_03 ;
		 ServoNodeID_4 = SERVO_NODE_ID_04 ;
		 ServoNodeID_5 = SERVO_NODE_ID_05 ;
		 ServoNodeID_6 = SERVO_NODE_ID_06 ;
	
     Enter_Pre_Operational_State(ServoNodeID_0);
	   /*��ʼ�����ŷ���1������operate״̬*/
     if(ServoNodeID_1)
     {    			 
          Servo_Node_ID = ServoNodeID_1;			 
          SDO_Process_HomingMode(Servo_Node_ID);      //��ʼ���ŷ����ڵ��Լ�������ز��� 		 
          Start_Remote_Node(Servo_Node_ID);             //����Զ�̽ڵ㣬����operateģʽ
		  Servo_Node_ID = 0;
     }
	Delay(10);
     /*��ʼ�����ŷ���2������operate״̬*/
     if(ServoNodeID_2)
     {
          Servo_Node_ID = ServoNodeID_2;			 
          SDO_Process_HomingMode(Servo_Node_ID);      //��ʼ���ŷ����ڵ��Լ�������ز��� 		 
          Start_Remote_Node(Servo_Node_ID);             //����Զ�̽ڵ㣬����operateģʽ
		  Servo_Node_ID = 0;
     }
	Delay(10);

     /*��ʼ�����ŷ���3������operate״̬*/
     if(ServoNodeID_3)
     {
          Servo_Node_ID = ServoNodeID_3;			 
          SDO_Process_HomingMode(Servo_Node_ID);      //��ʼ���ŷ����ڵ��Լ�������ز��� 		 
          Start_Remote_Node(Servo_Node_ID);             //����Զ�̽ڵ㣬����operateģʽ
		  Servo_Node_ID = 0;
     }
	Delay(10);

     /*��ʼ�����ŷ���4������operate״̬*/
     if(ServoNodeID_4)
     {
          Servo_Node_ID = ServoNodeID_4;			 
          SDO_Process_HomingMode(Servo_Node_ID);      //��ʼ���ŷ����ڵ��Լ�������ز��� 		 
          Start_Remote_Node(Servo_Node_ID);             //����Զ�̽ڵ㣬����operateģʽ
		  Servo_Node_ID = 0;
     }
	Delay(10);
	 
     /*��ʼ�����ŷ���5������operate״̬*/
     if(ServoNodeID_5)
     {
          Servo_Node_ID = ServoNodeID_5;			 
          SDO_Process_HomingMode(Servo_Node_ID);      //��ʼ���ŷ����ڵ��Լ�������ز��� 		 
          Start_Remote_Node(Servo_Node_ID);             //����Զ�̽ڵ㣬����operateģʽ
		  Servo_Node_ID = 0;
     }
	Delay(10);

     /*��ʼ�����ŷ���6������operate״̬*/
     if(ServoNodeID_6)
     {
          Servo_Node_ID = ServoNodeID_6;			 
          SDO_Process_HomingMode(Servo_Node_ID);      //��ʼ���ŷ����ڵ��Լ�������ز��� 		 
          Start_Remote_Node(Servo_Node_ID);             //����Զ�̽ڵ㣬����operateģʽ
		  Servo_Node_ID = 0;
     }
}
