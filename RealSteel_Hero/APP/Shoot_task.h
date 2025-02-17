#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#include "Motor.h"
#include "pid.h"
#include "main.h"
#include "remoter.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "stdio.h"

typedef enum
{
	SHOOT_SILENCE,
	SHOOT_AIM_AUTO,
	SHOOT_KEEP,
	SHOOT_ANGLE,
	SHOOT_ROLLBACK
}Shoot_Mode_t;

typedef struct
{
	Shoot_Mode_t Shoot_Mode;
	
	float Shoot_RPM_Set[2];
	float Trigger_RPM_Set;
	float Trigger_Angle_Set;
	
	int16_t Shoot_Torque[2];
	int16_t Trigger_Torque;
	
	int16_t Keep_Speed;
	
}Shoot_Data_t;

void shoot_task_main(void *argument);
void Shoot_Init(Shoot_Data_t *Shoot_Data_p);
void Shoot_Motor_Loop(Shoot_Data_t *Shoot_Data_p);
void Shoot_Move_Choose(Shoot_Data_t *Shoot_Data_p);
int8_t Shoot_Done_Check(Shoot_Data_t *Shoot_Data_p);

#endif
