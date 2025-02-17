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
	CHASSIS_SILENCE, //静默模式
	
	CHASSIS_Normal,     //上电复位模式(基于电角度)
	
	CHASSIS_GYRO,     	//云台运行模式(基于陀螺仪)
	
	//CHASSIS_FOLLOW			//底盘跟随云台
	
}Chassis_Mode_t;

typedef enum
{
	CHASSIS_REMOTECONTROL_CONTROLER,      //遥控器模式
	
	CHASSIS_REMOTECONTROL_KEYBOARDMOUSE,  //键鼠模式
	
	CHASSIS_REMOTECONTROL_MINIPC,         //MiniPC模式
	
}Chassis_RemoteControl_Mode_t;


typedef struct
{
	Chassis_RemoteControl_Mode_t Chassis_Remote_Mode;		//控制模式
	Chassis_Mode_t Chassis_Mode;					//运行模式
	
	float Vx_set;
	float Vy_set;
	float Wz_set;
	
	float Chassis_Angle;									//用于陀螺
	int16_t Chassis_Motor_Speed_Set[4];		//速度设定值
	int16_t Chassis_Motor_Speed_Ref[4];		//速度当前值
	float Chassis_Yaw;			// 用于底盘跟随

	
	float distance_set;
	float distance_ref;
	float last_Yaw_Set;
	float setup_ref;
	
	float Chassis_yaw_Littletop;
	float Chassis_yaw_Temp;
	
	int16_t Motor_Torque[4];							//扭矩电流
	
}Chassis_Data_t;

void Chassis_Feedback_Data(Chassis_Data_t *Chassis_Data_p);

#endif
