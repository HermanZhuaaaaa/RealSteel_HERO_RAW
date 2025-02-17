#ifndef __CHASSIS_TASK__
#define __CHASSIS_TASK__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can_bsp.h"
#include "bsp_usart.h"
#include "Motor.h"
#include "remoter.h"
#include "pid.h"


#define PI 3.14159f 
#define GIMBAL_FOWARD_AIM 100


typedef enum
{
	CHASSIS_SILENCE, //��Ĭģʽ
	
	CHASSIS_Normal,     //�ϵ縴λģʽ(���ڵ�Ƕ�)
	
	CHASSIS_GYRO,     	//��̨����ģʽ(����������)
	
	//CHASSIS_FOLLOW			//���̸�����̨
	
}Chassis_Mode_t;

typedef enum
{
	CHASSIS_REMOTECONTROL_CONTROLER,      //ң����ģʽ
	
	CHASSIS_REMOTECONTROL_KEYBOARDMOUSE,  //����ģʽ
	
	CHASSIS_REMOTECONTROL_MINIPC,         //MiniPCģʽ
	
}Chassis_RemoteControl_Mode_t;


typedef struct
{
	Chassis_RemoteControl_Mode_t Chassis_Remote_Mode;		//����ģʽ
	Chassis_Mode_t Chassis_Mode;					//����ģʽ
	
	float Vx_set;
	float Vy_set;
	float Wz_set;
	
	float Chassis_Angle;									//��������
	int16_t Chassis_Motor_Speed_Set[4];		//�ٶ��趨ֵ
	int16_t Chassis_Motor_Speed_Ref[4];		//�ٶȵ�ǰֵ
	float Chassis_Yaw;			// ���ڵ��̸���

	
	float distance_set;
	float distance_ref;
	float last_Yaw_Set;
	float setup_ref;
	
	float Chassis_yaw_Littletop;
	float Chassis_yaw_Temp;
	
	int16_t Motor_Torque[4];							//Ť�ص���
	
}Chassis_Data_t;

void Chassis_Feedback_Data(Chassis_Data_t *Chassis_Data_p);

#endif
