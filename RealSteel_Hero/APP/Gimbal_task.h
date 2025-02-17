#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "main.h"
#include "usart.h"
#include "Motor.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "pid.h"
#include "BMI088driver.h"
#include "remoter.h"
#include "Chassis_task.h"

#define PITCH_SETUP  300//285-306
#define YAW_SETUP 5


typedef enum {
	GIMBAL_SILENCE,
	GIMBAL_INIT,
	GIMBAL_GYRO,
	GIMBAL_FOLLOW
}Gimbal_Mode_t;

typedef enum {
	GIMBAL_REMOTECONTROL_CONTROLER,
	GIMBAL_REMOTECONTROL_KEYBOARDMOUSE,
	GIMBAL_REMOTECONTROL_MINIPC,
}Gimbal_RemoteControl_Mode_t;

typedef struct
{
	Gimbal_Mode_t Gimbal_Mode;
	Gimbal_RemoteControl_Mode_t Gimbal_RemoteControl_Mode;
	
	float	Yaw_Set,Pitch_Set;						//Yaw,Pitch�趨ֵ
	float Yaw_Ref,Pitch_Ref;						//Yaw,Pitch��ȡֵ
	float Distance_Set,Distance_Ref;		//���·���趨���ȡ�����������������
	float Last_Yaw_Set,Setup_Ref;				//����С����(Ӣ�۵��ǲ�̫��Ҫ)
	
	int16_t Gimbal_Motor_Speed_Set[2];	//�ٶ��趨ֵ
	int16_t Gimbal_Motor_Speed_Ref[2];	//�ٶȵ�ǰֵ
	int16_t Motor_Torque[2];						//Ť�ص���
}Gimbal_Data_t;

void gimbal_task_main(void *argument);
void Gimbal_Init(Gimbal_Data_t * init);
void Gimbal_Remote_Mode_Set(Gimbal_Data_t * Gimbal_Data_p);
void Gimbal_Move_Set(Gimbal_Data_t * Gimbal_Data_p);
void Gimbal_Motor_Loop(Gimbal_Data_t * Gimbal_Data_p);
void Gimbal_Yaw_PID_Calc(Gimbal_Data_t * Gimbal_Data_p);

#endif
