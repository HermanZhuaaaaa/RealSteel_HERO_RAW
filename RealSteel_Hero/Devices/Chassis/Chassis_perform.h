#ifndef __CHASSIS_PERFORM_H
#define __CHASSIS_PERFORM_H

#include "Chassis_task.h"
#include "Gimbal_task.h"
#include "main.h"
#include "imu_temp_ctrl.h"


//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f


void Chassis_Init(Chassis_Data_t *init);
void Chassis_Remote_Mode_Set(Chassis_Data_t *Chassis_Data_p);
void Chassis_Move_Set(Chassis_Data_t *Chassis_Data_p,RC_ctrl_t_Dbus *Rc_ctrl_p);
void Chassis_Follow_Control(Gimbal_Data_t *Gimbal_Yaw_Distance,Chassis_Data_t *Chassis_move_top);
void Chassis_Normal_Control(float Vx, float Vy, float Wz, Chassis_Data_t *Chassis_data_p);
void Chassis_Move_LittleTop(float Vx, float Vy, float Wz, Chassis_Data_t *Chassis_move_top);
void chassis_control_loop();
void Chassis_Go(Chassis_Data_t *Chassis_Data_p,RC_ctrl_t_Dbus *Rc_ctrl_p);
#endif
