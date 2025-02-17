#include "Chassis_task.h"
#include "Chassis_perform.h"

Chassis_Data_t Chassis_data;

extern Motor_dat_t motor_chassis[4];
extern Motor_dat_t motor_gimbal[2];

extern RC_ctrl_t_Dbus Rc_ctrl;

pid_type_def PID_Chassis_motor[4];					//四个底盘电机的PID参数
pid_type_def PID_Chassis_Follow_Control;		//底盘跟随云台的PID参数
pid_type_def PID_Chassis_Follow_Gimbal;

float angle_temp;
extern Motor_Angle motor_gimbal_yaw;

void Chassis_Control_Loop(Chassis_Data_t *loop)
{
	for (int i = 0; i < 4; i++)
	{
		loop->Motor_Torque[i] = PID_calc(&PID_Chassis_motor[i], motor_chassis[i].speed_rpm, loop->Chassis_Motor_Speed_Set[i]);
	}
}

void chassis_task_main(void *argument)
{
	Chassis_Init(&Chassis_data);
	osDelay(1000);
  /* USER CODE BEGIN chassis_task_main */

  /* Infinite loop */
  for(;;)
  {
		Chassis_Feedback_Data(&Chassis_data);
		Chassis_Remote_Mode_Set(&Chassis_data);
		
		Chassis_Move_Set(&Chassis_data,&Rc_ctrl);

		Chassis_Control_Loop(&Chassis_data);
		Chassis_cmd_CAN(Chassis_data.Motor_Torque[0],Chassis_data.Motor_Torque[1],Chassis_data.Motor_Torque[2],Chassis_data.Motor_Torque[3]);
		osDelay(2);
	}
  /* USER CODE END chassis_task_main */
}





