#include "Gimbal_task.h"

Gimbal_Data_t Gimbal_Data;
pid_type_def PID_Gimbal_Yaw[2];
pid_type_def PID_Gimbal_Pitch[2];

extern RC_ctrl_t_Dbus Rc_ctrl;
extern IMU_Data_t IMU_Data;

extern Motor_dat_t motor_gimbal[2];
extern Motor_Angle motor_gimbal_pitch;
extern Motor_Angle motor_gimbal_yaw;

extern Chassis_Data_t Chassis_data;
extern pid_type_def PID_Chassis_motor[4];									//四个底盘电机的PID参数
extern pid_type_def PID_Chassis_Follow_Control;		        //底盘跟随云台的PID参数

void gimbal_task_main(void *argument)
{
  /* USER CODE BEGIN gimbal_task_main */
	osDelay(100);
	Gimbal_Init(&Gimbal_Data);
  /* Infinite loop */
  for(;;)
  {
		Gimbal_Remote_Mode_Set(&Gimbal_Data);
		Gimbal_Move_Set(&Gimbal_Data);
		Gimbal_Motor_Loop(&Gimbal_Data);		
    osDelay(1);
  }
  /* USER CODE END gimbal_task_main */
}

void Gimbal_Init(Gimbal_Data_t * init)
{
	float PID_Gimbal_Yaw_args[2][3] = {{-20, 0, 0},{80, 0, 0}};
  float PID_Gimbal_Pitch_args[2][3] = {{25, 0, 100},{80, 0, 0}};
	for(int i=0;i<2;i++) PID_init(&(PID_Gimbal_Yaw[i]), PID_POSITION, PID_Gimbal_Yaw_args[i], 16383, 8000);
  for(int i=0;i<2;i++) PID_init(&(PID_Gimbal_Pitch[i]), PID_POSITION, PID_Gimbal_Pitch_args[i], 16383, 10000);
	
	init->Gimbal_RemoteControl_Mode = GIMBAL_REMOTECONTROL_CONTROLER;
	init->Gimbal_Mode = GIMBAL_INIT;
	init->Yaw_Set = YAW_SETUP;
	init->Pitch_Set = PITCH_SETUP;
	for(int i = 0; i<3500;i++)
  {
    init->Yaw_Ref = 5;
    init->Pitch_Ref = motor_gimbal_pitch.angle;
    Gimbal_Motor_Loop(&Gimbal_Data);
		osDelay(1);
  }
  //init->Yaw_Set = YAW_SETUP;
}

void Gimbal_Remote_Mode_Set(Gimbal_Data_t * Gimbal_Data_p)
{
  switch (Gimbal_Data_p->Gimbal_RemoteControl_Mode)
  {
  case GIMBAL_REMOTECONTROL_CONTROLER:
		
    if(Rc_ctrl.rc.s[0] == 2 || Rc_ctrl.rc.s[0] == 0) 
			Gimbal_Data_p->Gimbal_Mode = GIMBAL_SILENCE; //静默模式
		
    if(Rc_ctrl.rc.s[0] == 3)  	//原本为小陀螺模式，感觉在英雄上是无用的
      Gimbal_Data_p->Gimbal_Mode = GIMBAL_GYRO; //陀螺仪模式
		
		else if (Rc_ctrl.rc.s[0] == 1 )		//云台跟随
			Gimbal_Data_p->Gimbal_Mode = GIMBAL_FOLLOW;
    break;
		
	case GIMBAL_REMOTECONTROL_KEYBOARDMOUSE	:	
    break;
		
	case GIMBAL_REMOTECONTROL_MINIPC        : 
    break;
    
  default:
    break;
  }
}

void Gimbal_Move_Set(Gimbal_Data_t * Gimbal_Data_p)
{
	Gimbal_Data_p->Yaw_Ref = IMU_Data.yaw;    //读取Yaw角
	Gimbal_Data_p->Pitch_Ref = motor_gimbal_pitch.angle;    //读取Pitch角
      //TODO 需要启用  
  switch (Gimbal_Data_p->Gimbal_Mode)
  {
  case GIMBAL_SILENCE:
											/* code */
											Gimbal_Data_p->Yaw_Set = Gimbal_Data_p->Yaw_Ref;
											Gimbal_Data_p->Pitch_Set = Gimbal_Data_p->Pitch_Ref;
											break;
	//TODO 底盘跟随云台
  case GIMBAL_FOLLOW:
										/* code */
										if(Rc_ctrl.rc.ch[0]<=50 && Rc_ctrl.rc.ch[0]>=-50) Rc_ctrl.rc.ch[0] = 0;       //当遥控器Yaw轴值在0~50时，设置为0
										if(Rc_ctrl.rc.ch[1]<=50 && Rc_ctrl.rc.ch[1]>=-50) Rc_ctrl.rc.ch[1] = 0;       //当遥控器Pitch轴值在0~50时，设置为0

										Gimbal_Data_p->Yaw_Set -= Rc_ctrl.rc.ch[0]/6000.0;                            //当遥控器Yaw轴值在-50~50时，设置为0~50
										if(Gimbal_Data_p->Yaw_Set > 360) Gimbal_Data_p->Yaw_Set -= 360;               //当Yaw角大于360度时，减去360度
										if(Gimbal_Data_p->Yaw_Set < 0) Gimbal_Data_p->Yaw_Set += 360;                 //当Yaw角小于0度时，加上360度
										
										Gimbal_Data_p->Pitch_Set -= Rc_ctrl.rc.ch[1]/6000.0;                          //当遥控器Pitch轴值在-50~50时，设置为0~50
										if(Gimbal_Data_p->Pitch_Set > PITCH_SETUP+5) Gimbal_Data_p->Pitch_Set = PITCH_SETUP+5;    //当Pitch角大于25度时，设置为25度
										if(Gimbal_Data_p->Pitch_Set < PITCH_SETUP-15) Gimbal_Data_p->Pitch_Set = PITCH_SETUP-15;    //当Pitch角小于-25度时，设置为-25度
										break;
  case GIMBAL_GYRO:
										/* code */
										Gimbal_Data_p->Yaw_Ref = IMU_Data.yaw;    //读取Yaw角
										Gimbal_Data_p->Pitch_Ref = motor_gimbal_pitch.angle;    //读取Pitch角

										if(Rc_ctrl.rc.ch[0]<=50 && Rc_ctrl.rc.ch[0]>=-50) Rc_ctrl.rc.ch[0] = 0;       //当遥控器Yaw轴值在0~50时，设置为0
										if(Rc_ctrl.rc.ch[1]<=50 && Rc_ctrl.rc.ch[1]>=-50) Rc_ctrl.rc.ch[1] = 0;       //当遥控器Pitch轴值在0~50时，设置为0

										Gimbal_Data_p->Yaw_Set -= Rc_ctrl.rc.ch[0]/6000.0;                            //当遥控器Yaw轴值在-50~50时，设置为0~50
										if(Gimbal_Data_p->Yaw_Set > 360) Gimbal_Data_p->Yaw_Set -= 360;               //当Yaw角大于360度时，减去360度
										if(Gimbal_Data_p->Yaw_Set < 0) Gimbal_Data_p->Yaw_Set += 360;                 //当Yaw角小于0度时，加上360度
										
										Gimbal_Data_p->Pitch_Set -= Rc_ctrl.rc.ch[1]/6000.0;                          //当遥控器Pitch轴值在-50~50时，设置为0~50
										if(Gimbal_Data_p->Pitch_Set > PITCH_SETUP+5) Gimbal_Data_p->Pitch_Set = PITCH_SETUP+5;    //当Pitch角大于25度时，设置为25度
										if(Gimbal_Data_p->Pitch_Set < PITCH_SETUP-15) Gimbal_Data_p->Pitch_Set = PITCH_SETUP-15;    //当Pitch角小于-25度时，设置为-25度
										break;
  default:  break;
  }
}

// 函数：Gimbal_Motor_Loop
// 功能：云台电机的主循环函数，进行 PID 计算和电机控制操作
// 参数：Gimbal_Data_t 类型的指针 Gimbal_Data_p，指向云台数据结构
void Gimbal_Motor_Loop(Gimbal_Data_t * Gimbal_Data_p)
{
    // 调用 Gimbal_Yaw_PID_Calc 函数进行 Yaw 轴的 PID 计算
    Gimbal_Yaw_PID_Calc(Gimbal_Data_p);
    // 对 Pitch 轴的第一个 PID 进行计算，输入为电机的角度和设定值
    PID_calc(&PID_Gimbal_Pitch[0],motor_gimbal_pitch.angle,Gimbal_Data_p->Pitch_Set);
    
    // 如果电机 1 的转速超过 1000rpm，则将其设置为 0
    if(motor_gimbal[1].speed_rpm>300) motor_gimbal[1].speed_rpm=0;

    // 对 Pitch 轴的第二个 PID 进行计算，输入为电机 1 的转速和第一个 PID 的输出
    PID_calc(&(PID_Gimbal_Pitch[1]), motor_gimbal[1].speed_rpm, PID_Gimbal_Pitch[0].out);
    // 如果云台处于静默模式，发送停止命令
    if(Gimbal_Data_p->Gimbal_Mode == GIMBAL_SILENCE) Gimbal_cmd_CAN(0,0);
    // 否则，根据 Yaw 和 Pitch 的 PID 输出发送 CAN 命令
    else Gimbal_cmd_CAN(PID_Gimbal_Yaw[1].out, PID_Gimbal_Pitch[1].out);    
}

// 函数：Gimbal_Yaw_PID_Calc
// 功能：计算 Yaw 轴的 PID 相关参数
// 参数：Gimbal_Data_t 类型的指针 Gimbal_Data_p，指向云台数据结构
void Gimbal_Yaw_PID_Calc(Gimbal_Data_t * Gimbal_Data_p)
{
	
    // 如果 Yaw 的设定值与上一次的设定值不同
    if (Gimbal_Data_p->Yaw_Set!= Gimbal_Data_p->Last_Yaw_Set)
    {
        // 更新上一次的 Yaw 设定值
        Gimbal_Data_p->Last_Yaw_Set = Gimbal_Data_p->Yaw_Set;
        // 计算 Yaw 设定值和参考值之间的距离
        Gimbal_Data_p->Distance_Set = Gimbal_Data_p->Yaw_Set - Gimbal_Data_p->Yaw_Ref;
        // 处理距离大于 180 度的情况，将其调整到 -180 到 180 度范围
        if (Gimbal_Data_p->Distance_Set > 180) Gimbal_Data_p->Distance_Set = Gimbal_Data_p->Distance_Set - 360;
        if (Gimbal_Data_p->Distance_Set <-180) Gimbal_Data_p->Distance_Set = 360 + Gimbal_Data_p->Distance_Set;
        // 更新参考设定值
        Gimbal_Data_p->Setup_Ref = Gimbal_Data_p->Yaw_Ref;
    }
		if(Gimbal_Data_p->Distance_Ref -  Gimbal_Data_p->Distance_Set < 5) Gimbal_Data_p->Distance_Ref = Gimbal_Data_p->Distance_Set;
		if(Gimbal_Data_p->Distance_Ref -  Gimbal_Data_p->Distance_Set > -5) Gimbal_Data_p->Distance_Ref = Gimbal_Data_p->Distance_Set;
		
    // 计算 Yaw 参考值和设定参考值之间的距离
    Gimbal_Data_p->Distance_Ref = Gimbal_Data_p->Yaw_Ref - Gimbal_Data_p->Setup_Ref;
    // 处理距离大于 180 度的情况，将其调整到 -180 到 180 度范围
    if (Gimbal_Data_p->Distance_Ref > 180) Gimbal_Data_p->Distance_Ref = Gimbal_Data_p->Yaw_Ref - 360 - Gimbal_Data_p->Setup_Ref;
    if (Gimbal_Data_p->Distance_Ref <-180) Gimbal_Data_p->Distance_Ref = Gimbal_Data_p->Yaw_Ref + 360 - Gimbal_Data_p->Setup_Ref;
    // 死区处理，可能是为了避免抖动或误差
		
    // 对 Yaw 轴的第一个 PID 进行计算，输入为距离参考值和距离设定值
    PID_calc(&(PID_Gimbal_Yaw[0]), Gimbal_Data_p->Distance_Ref, Gimbal_Data_p->Distance_Set);    
    // 如果电机 0 的转速超过 1500rpm，则将其设置为 0
    if(motor_gimbal[0].speed_rpm>1500) motor_gimbal[0].speed_rpm=0;
		
//		if(PID_Gimbal_Yaw[0].out - motor_gimbal[0].speed_rpm <= 0.5) PID_Gimbal_Yaw[0].out = 0;
//		if(PID_Gimbal_Yaw[0].out - motor_gimbal[0].speed_rpm >= -0.5) PID_Gimbal_Yaw[0].out = 0;
    // 对 Yaw 轴的第二个 PID 进行计算，输入为电机 0 的转速和第一个 PID 的输出
    PID_calc(&(PID_Gimbal_Yaw[1]), motor_gimbal[0].speed_rpm, PID_Gimbal_Yaw[0].out);
}
