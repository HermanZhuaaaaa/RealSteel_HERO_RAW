#include "Shoot_task.h"

extern RC_ctrl_t_Dbus Rc_ctrl;
extern osEventFlagsId_t Shoot_EventHandle;

int16_t Shoot_Angle_Mode_Cnt = 0;
Shoot_Data_t Shoot_Data = {0};

extern Motor_dat_t motor_chassis[4];
extern Motor_dat_t motor_gimbal[2];
extern Motor_dat_t motor_shoot[3];
extern Motor_Angle motor_trigger;

pid_type_def PID_Shooter[2];
pid_type_def PID_Trigger[2];

int16_t speed_t = 0;

void shoot_task_main(void *argument)
{
  /* USER CODE BEGIN shoot_task_main */
  
  osDelay(1000);
	Shoot_Init(&Shoot_Data);
	motor_trigger.offset_angle = motor_shoot[2].ecd;
  /* Infinite loop */
  for(;;)
  {
		Shoot_Move_Choose(&Shoot_Data);
		Shoot_Motor_Loop(&Shoot_Data);
    osDelay(1);
  }
  /* USER CODE END shoot_task_main */
}

void Shoot_Init(Shoot_Data_t *Shoot_Data_p)
{
    float PID_Shooter_args[2][3] = {{100, 0.1, 0}, {100, 0.1, 0}};
    float PID_Trigger_args[2][3] = {{100, 0.1, 0}, {100, 0.1, 0}};
    for (int i = 0; i < 2; i++)
        PID_init(&(PID_Shooter[i]), PID_POSITION, PID_Shooter_args[i], 16383, 10000);
    for (int i = 0; i < 2; i++)
        PID_init(&(PID_Trigger[i]), PID_POSITION, PID_Trigger_args[i], 16383, 10000);
    
    Shoot_Data_p->Shoot_RPM_Set[0] = 5000;
    Shoot_Data_p->Shoot_RPM_Set[1] = -5000;
		//TODO 单发射击
    //Shoot_Data_p->Trigger_Angle_Set = motor_trigger.angle;
}

void Shoot_Motor_Loop(Shoot_Data_t *Shoot_Data_p)
{
    PID_calc(&(PID_Shooter[0]), motor_shoot[0].speed_rpm, Shoot_Data.Shoot_RPM_Set[0]);
	PID_calc(&(PID_Shooter[1]), motor_shoot[1].speed_rpm, Shoot_Data.Shoot_RPM_Set[1]);

	switch (Shoot_Data_p->Shoot_Mode)
	{
		case SHOOT_SILENCE:   
            PID_calc(&(PID_Trigger[1]), motor_shoot[2].speed_rpm, 0);
						break;
//		未完成、用不上
//		case SHOOT_AIM_AUTO	:
//						break;
		case SHOOT_ANGLE:		
            PID_calc(&(PID_Trigger[0]), motor_trigger.angle, Shoot_Data.Trigger_Angle_Set);
						PID_calc(&(PID_Trigger[1]), motor_shoot[2].speed_rpm, PID_Trigger[0].out);
						break;
		case SHOOT_KEEP: 	
            PID_calc(&(PID_Trigger[1]), motor_shoot[2].speed_rpm, -600);
						break;
//		英雄是无法退蛋的
//		case SHOOT_ROLLBACK :   
//            PID_calc(&(PID_Trigger[1]), motor_shoot[2].speed_rpm, -500);
//						break;
	}
  	Shoot_cmd_CAN(PID_Shooter[0].out, PID_Shooter[1].out, PID_Trigger[1].out);
		Shoot_cmd_CAN(PID_Shooter[0].out, PID_Shooter[1].out, 0);
}
void Shoot_Move_Choose(Shoot_Data_t *Shoot_Data_p)
{
if (Rc_ctrl.rc.s[1] == 1) 
		{
			Shoot_Data_p->Shoot_Mode = SHOOT_SILENCE;
			Shoot_Data.Shoot_RPM_Set[0] = 0;
			Shoot_Data.Shoot_RPM_Set[1] = 0;
		}
		//连射模式判断
		if (Rc_ctrl.rc.s[1] == 3) 
		{

			Shoot_Data.Shoot_RPM_Set[0] = 5000;
			Shoot_Data.Shoot_RPM_Set[1] = -5000;
			if(Rc_ctrl.rc.ch[4] <= -100) Shoot_Data_p->Shoot_Mode = SHOOT_KEEP;
			else Shoot_Data_p->Shoot_Mode = SHOOT_SILENCE;
		}
		 
		//三发点射模式判断
//		if (Rc_ctrl.rc.s[1] == 2)
//		{
//			Shoot_Data.Shoot_RPM_Set[0] = 6000;
//			Shoot_Data.Shoot_RPM_Set[1] = -6000;
//			if (Rc_ctrl.rc.ch[4] <= -300)
//			{
//				osDelay(20);
//				if (Rc_ctrl.rc.ch[4] >= -300) Shoot_Data_p->Shoot_Mode = SHOOT_ANGLE;
//			}
//			
//		}
		//退弹模式判断
		if (Rc_ctrl.rc.ch[4] >= 300)
		{
			Shoot_Data_p->Shoot_Mode = SHOOT_ROLLBACK;
		}
}
int8_t Shoot_Done_Check(Shoot_Data_t *Shoot_Data_p)
{
	if (motor_trigger.angle >=Shoot_Data_p->Trigger_Angle_Set-5 && motor_trigger.angle <= Shoot_Data_p->Trigger_Angle_Set+5 )
		return 1;
	else 
		return 0;
}
