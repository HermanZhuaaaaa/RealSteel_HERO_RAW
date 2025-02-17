#include "Chassis_perform.h"

extern float angle_temp;
extern RC_ctrl_t_Dbus Rc_ctrl;

extern IMU_Data_t IMU_Data;				//imu数据结构体

extern pid_type_def PID_Chassis_motor[4];					//四个底盘电机的PID参数
extern pid_type_def PID_Chassis_Follow_Control;		//底盘跟随云台的PID参数
extern pid_type_def PID_Chassis_Follow_Gimbal;

extern Chassis_Data_t Chassis_data;
extern Gimbal_Data_t Gimbal_Data;

extern Motor_dat_t motor_chassis[4];
extern Motor_dat_t motor_gimbal[2];

extern Motor_Angle motor_gimbal_yaw;

float cos_yaw,sin_yaw;

void Chassis_Init(Chassis_Data_t *init)
{
	float motor_pid_t_1[3] = {10,0.1,0};
	float motor_pid_t_2[3] = {10,0.1,0};
	float motor_pid_t_3[3] = {10,0.1,0};
	float motor_pid_t_4[3] = {10,0.1,0};

	float PID_Chassis_Follow_Control_args[3] = {20, 0, 0.2};
	float PID_Chassis_Follow_Gimbal_Set[3] = {800,0,0.1};
	init->Vx_set = 0;
	init->Vy_set = 0;
	init->Wz_set = 0;
	init->Chassis_Angle = 0;
	
	for(int i = 0; i < 4; i++)
	{
		init->Chassis_Motor_Speed_Set[i] = 0;
		init->Motor_Torque[i] = 0;
	}
	 PID_init(&PID_Chassis_motor[0],PID_POSITION,motor_pid_t_1,15000,5000);
	 PID_init(&PID_Chassis_motor[1],PID_POSITION,motor_pid_t_2,15000,5000);
	 PID_init(&PID_Chassis_motor[2],PID_POSITION,motor_pid_t_3,15000,5000);
	 PID_init(&PID_Chassis_motor[3],PID_POSITION,motor_pid_t_4,15000,5000);
	 PID_init(&PID_Chassis_Follow_Control,PID_POSITION,PID_Chassis_Follow_Control_args,2000,3000);
	 PID_init(&PID_Chassis_Follow_Gimbal,PID_POSITION,PID_Chassis_Follow_Gimbal_Set,15000,3000);
	init->Chassis_Remote_Mode = CHASSIS_REMOTECONTROL_CONTROLER;   //CHASSIS_REMOTECONTROL_CONTROLER
	init->Chassis_Mode = CHASSIS_SILENCE;
}

void Chassis_Remote_Mode_Set(Chassis_Data_t *Chassis_Data_p)
{
	//选择底盘运动模式,2键默认为键鼠模式，上电默认静默，不通过拨杆控制静默
	//if(Rc_ctrl.rc.s[0] == 2) Chassis_Data_p->Chassis_Mode = CHASSIS_SILENCE;
	if(Rc_ctrl.rc.s[0] == 1) Chassis_Data_p->Chassis_Mode = CHASSIS_Normal;
	if(Rc_ctrl.rc.s[0] == 3) Chassis_Data_p->Chassis_Mode = CHASSIS_GYRO;
	switch(Chassis_Data_p->Chassis_Remote_Mode)
	{
		case CHASSIS_REMOTECONTROL_CONTROLER			:			//选择底盘运动模式,2键默认为键鼠模式，上电默认静默，不通过拨杆控制静默
																							//if(Rc_ctrl.rc.s[0] == 2) Chassis_Data_p->Chassis_Mode = CHASSIS_SILENCE;
//																							if(Rc_ctrl.rc.s[0] == 1) Chassis_Data_p->Chassis_Mode = CHASSIS_Normal;
//																							if(Rc_ctrl.rc.s[0] == 3) Chassis_Data_p->Chassis_Mode = CHASSIS_GYRO;
																							if(Rc_ctrl.rc.ch[3]<=10 && Rc_ctrl.rc.ch[3]>=-10) Rc_ctrl.rc.ch[3] = 0;
																							if(Rc_ctrl.rc.ch[2]<=10 && Rc_ctrl.rc.ch[2]>=-10) Rc_ctrl.rc.ch[2] = 0;
																							Chassis_Data_p->Vy_set  = -Rc_ctrl.rc.ch[3]*3;
																							Chassis_Data_p->Vx_set = -Rc_ctrl.rc.ch[2]*3;
																							break;
		
		case CHASSIS_REMOTECONTROL_KEYBOARDMOUSE  :		
//																							if(Rc_ctrl.rc.s[0] == 1) Chassis_Data_p->Chassis_Mode = CHASSIS_Normal;
//																							if(Rc_ctrl.rc.s[0] == 3) Chassis_Data_p->Chassis_Mode = CHASSIS_GYRO;
																							if(Rc_ctrl.key.v  == 0)	Chassis_Data_p->Vy_set  = Chassis_Data_p->Vx_set = 0;
																							if(Rc_ctrl.key.v & KEY_PRESSED_OFFSET_W )	Chassis_Data_p->Vy_set  = 400*3;
																							if(Rc_ctrl.key.v & KEY_PRESSED_OFFSET_S )	Chassis_Data_p->Vy_set  = -400*3;
																							if(Rc_ctrl.key.v & KEY_PRESSED_OFFSET_A )	Chassis_Data_p->Vx_set = 400*3;
																							if(Rc_ctrl.key.v & KEY_PRESSED_OFFSET_D )	Chassis_Data_p->Vx_set = -400*3;
																							//Chassis_Data_p->Vy_set  = -Rc_ctrl.rc.ch[3]*3;
																							//Chassis_Data_p->Vx_set = -Rc_ctrl.rc.ch[2]*3;
																							break;
		
		case CHASSIS_REMOTECONTROL_MINIPC      		:		
																									break;
	}
}	
	

void Chassis_Move_Set(Chassis_Data_t *Chassis_Data_p,RC_ctrl_t_Dbus *Rc_ctrl_p)
{
	//根据遥控器通道进行平移量的设定，Vx， Vy
	//Chassis_Data_p->Vx_set =  Rc_ctrl_p->rc.ch[3] * 3.0f;
	//Chassis_Data_p->Vy_set = -Rc_ctrl_p->rc.ch[2] * 3.0f;
	//旋转量，需要改动
	//Chassis_Data_p->Wz_set =  Rc_ctrl_p->rc.ch[0] * 3.0f;
	
	switch(Chassis_Data_p->Chassis_Mode)
	{
		case CHASSIS_SILENCE :	Chassis_Data_p->Wz_set = 0 ;
														for(int i=0;i<4;i++)
														{
															Chassis_Data_p->Chassis_Motor_Speed_Set[i]=0;
														}
															break;
		case CHASSIS_Normal  : 
														//Chassis_Data_p->Wz_set = Chassis_Data_p->distance_set;
														if(Chassis_Data_p->Chassis_Yaw-GIMBAL_FOWARD_AIM<5&&Chassis_Data_p->Chassis_Yaw-GIMBAL_FOWARD_AIM>-5) Chassis_Data_p->Chassis_Yaw = GIMBAL_FOWARD_AIM;	
														PID_calc(&(PID_Chassis_Follow_Control), Chassis_Data_p->Chassis_Yaw, GIMBAL_FOWARD_AIM);
														//Chassis_Follow_Control(&Gimbal_Data,Chassis_Data_p);
															Chassis_Normal_Control(Chassis_Data_p->Vx_set,Chassis_Data_p->Vy_set,Chassis_Data_p->Wz_set,Chassis_Data_p);

														break;
		case CHASSIS_GYRO		:  Chassis_Data_p->Wz_set = 1000 ;
														 Chassis_Move_LittleTop(Chassis_Data_p->Vx_set,Chassis_Data_p->Vy_set,Chassis_Data_p->Wz_set,Chassis_Data_p);
															break;
	}
}

//底盘跟随云台
//TODO 云台旋转一定角度，底盘自动跟随过去
void Chassis_Follow_Control(Gimbal_Data_t *Gimbal_Yaw_Distance,Chassis_Data_t *Chassis_move_top)
{	
	float vx_set = 0.0f, vy_set = 0.0f;
	float sin_yaw = 0.0f,cos_yaw = 0.0f;
	sin_yaw = arm_sin_f32(Gimbal_Yaw_Distance->Distance_Set);
	cos_yaw = arm_sin_f32(Gimbal_Yaw_Distance->Distance_Set);
	
	Chassis_move_top->Vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
	Chassis_move_top->Vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
	Chassis_move_top->Wz_set = -PID_calc(&PID_Chassis_Follow_Control,Gimbal_Yaw_Distance->Distance_Ref,Gimbal_Yaw_Distance->Distance_Set);
	Chassis_Normal_Control(Chassis_move_top->Vx_set,Chassis_move_top->Vy_set,Chassis_move_top->Wz_set,Chassis_move_top);

}
//底盘位置
//TODO 只有前后左右，需要添加旋转
void Chassis_Normal_Control(float Vx, float Vy, float Wz, Chassis_Data_t *Chassis_data_p)
{
	if (Wz > -5 && Wz < 5)	Wz = 0;
	Chassis_data_p->Chassis_Motor_Speed_Set[3] = -Vx - Vy + Wz ;
	Chassis_data_p->Chassis_Motor_Speed_Set[0] =  Vx - Vy + Wz ;
	Chassis_data_p->Chassis_Motor_Speed_Set[1] =  Vx + Vy + Wz ;
	Chassis_data_p->Chassis_Motor_Speed_Set[2] = -Vx + Vy + Wz ;
}

void Chassis_Move_LittleTop(float Vx, float Vy, float Wz, Chassis_Data_t *Chassis_move_top)
{
	angle_temp = -(IMU_Data.yaw + 180);
	Chassis_move_top->Chassis_Motor_Speed_Set[3] = (int16_t)(-Vx * sin((45 + angle_temp) * (PI / 180.f)) + Vy * cos((45 + angle_temp) * (PI / 180.f))+ Wz);
	Chassis_move_top->Chassis_Motor_Speed_Set[0] = (int16_t)(Vx * sin((45 - angle_temp) * (PI / 180.f)) + Vy * cos((45 - angle_temp) * (PI / 180.f))+ Wz);
	Chassis_move_top->Chassis_Motor_Speed_Set[1] = (int16_t)(Vx * sin((45 + angle_temp) * (PI / 180.f)) - Vy * cos((45 + angle_temp) * (PI / 180.f))+ Wz);
	Chassis_move_top->Chassis_Motor_Speed_Set[2] = (int16_t)(-Vx * sin((45 - angle_temp) * (PI / 180.f)) - Vy * cos((45 - angle_temp) * (PI / 180.f))+ Wz);
}

void Chassis_Feedback_Data(Chassis_Data_t *Chassis_Data_p)
{
	for (int i = 0; i < 4; i++)
	{
		Chassis_Data_p->Chassis_Motor_Speed_Ref[i] = motor_chassis[i].speed_rpm;
	}
	Chassis_Data_p->Chassis_Yaw = motor_gimbal_yaw.angle;
}
