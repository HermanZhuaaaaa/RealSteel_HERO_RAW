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
extern pid_type_def PID_Chassis_motor[4];									//�ĸ����̵����PID����
extern pid_type_def PID_Chassis_Follow_Control;		        //���̸�����̨��PID����

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
			Gimbal_Data_p->Gimbal_Mode = GIMBAL_SILENCE; //��Ĭģʽ
		
    if(Rc_ctrl.rc.s[0] == 3)  	//ԭ��ΪС����ģʽ���о���Ӣ���������õ�
      Gimbal_Data_p->Gimbal_Mode = GIMBAL_GYRO; //������ģʽ
		
		else if (Rc_ctrl.rc.s[0] == 1 )		//��̨����
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
	Gimbal_Data_p->Yaw_Ref = IMU_Data.yaw;    //��ȡYaw��
	Gimbal_Data_p->Pitch_Ref = motor_gimbal_pitch.angle;    //��ȡPitch��
      //TODO ��Ҫ����  
  switch (Gimbal_Data_p->Gimbal_Mode)
  {
  case GIMBAL_SILENCE:
											/* code */
											Gimbal_Data_p->Yaw_Set = Gimbal_Data_p->Yaw_Ref;
											Gimbal_Data_p->Pitch_Set = Gimbal_Data_p->Pitch_Ref;
											break;
	//TODO ���̸�����̨
  case GIMBAL_FOLLOW:
										/* code */
										if(Rc_ctrl.rc.ch[0]<=50 && Rc_ctrl.rc.ch[0]>=-50) Rc_ctrl.rc.ch[0] = 0;       //��ң����Yaw��ֵ��0~50ʱ������Ϊ0
										if(Rc_ctrl.rc.ch[1]<=50 && Rc_ctrl.rc.ch[1]>=-50) Rc_ctrl.rc.ch[1] = 0;       //��ң����Pitch��ֵ��0~50ʱ������Ϊ0

										Gimbal_Data_p->Yaw_Set -= Rc_ctrl.rc.ch[0]/6000.0;                            //��ң����Yaw��ֵ��-50~50ʱ������Ϊ0~50
										if(Gimbal_Data_p->Yaw_Set > 360) Gimbal_Data_p->Yaw_Set -= 360;               //��Yaw�Ǵ���360��ʱ����ȥ360��
										if(Gimbal_Data_p->Yaw_Set < 0) Gimbal_Data_p->Yaw_Set += 360;                 //��Yaw��С��0��ʱ������360��
										
										Gimbal_Data_p->Pitch_Set -= Rc_ctrl.rc.ch[1]/6000.0;                          //��ң����Pitch��ֵ��-50~50ʱ������Ϊ0~50
										if(Gimbal_Data_p->Pitch_Set > PITCH_SETUP+5) Gimbal_Data_p->Pitch_Set = PITCH_SETUP+5;    //��Pitch�Ǵ���25��ʱ������Ϊ25��
										if(Gimbal_Data_p->Pitch_Set < PITCH_SETUP-15) Gimbal_Data_p->Pitch_Set = PITCH_SETUP-15;    //��Pitch��С��-25��ʱ������Ϊ-25��
										break;
  case GIMBAL_GYRO:
										/* code */
										Gimbal_Data_p->Yaw_Ref = IMU_Data.yaw;    //��ȡYaw��
										Gimbal_Data_p->Pitch_Ref = motor_gimbal_pitch.angle;    //��ȡPitch��

										if(Rc_ctrl.rc.ch[0]<=50 && Rc_ctrl.rc.ch[0]>=-50) Rc_ctrl.rc.ch[0] = 0;       //��ң����Yaw��ֵ��0~50ʱ������Ϊ0
										if(Rc_ctrl.rc.ch[1]<=50 && Rc_ctrl.rc.ch[1]>=-50) Rc_ctrl.rc.ch[1] = 0;       //��ң����Pitch��ֵ��0~50ʱ������Ϊ0

										Gimbal_Data_p->Yaw_Set -= Rc_ctrl.rc.ch[0]/6000.0;                            //��ң����Yaw��ֵ��-50~50ʱ������Ϊ0~50
										if(Gimbal_Data_p->Yaw_Set > 360) Gimbal_Data_p->Yaw_Set -= 360;               //��Yaw�Ǵ���360��ʱ����ȥ360��
										if(Gimbal_Data_p->Yaw_Set < 0) Gimbal_Data_p->Yaw_Set += 360;                 //��Yaw��С��0��ʱ������360��
										
										Gimbal_Data_p->Pitch_Set -= Rc_ctrl.rc.ch[1]/6000.0;                          //��ң����Pitch��ֵ��-50~50ʱ������Ϊ0~50
										if(Gimbal_Data_p->Pitch_Set > PITCH_SETUP+5) Gimbal_Data_p->Pitch_Set = PITCH_SETUP+5;    //��Pitch�Ǵ���25��ʱ������Ϊ25��
										if(Gimbal_Data_p->Pitch_Set < PITCH_SETUP-15) Gimbal_Data_p->Pitch_Set = PITCH_SETUP-15;    //��Pitch��С��-25��ʱ������Ϊ-25��
										break;
  default:  break;
  }
}

// ������Gimbal_Motor_Loop
// ���ܣ���̨�������ѭ������������ PID ����͵�����Ʋ���
// ������Gimbal_Data_t ���͵�ָ�� Gimbal_Data_p��ָ����̨���ݽṹ
void Gimbal_Motor_Loop(Gimbal_Data_t * Gimbal_Data_p)
{
    // ���� Gimbal_Yaw_PID_Calc �������� Yaw ��� PID ����
    Gimbal_Yaw_PID_Calc(Gimbal_Data_p);
    // �� Pitch ��ĵ�һ�� PID ���м��㣬����Ϊ����ĽǶȺ��趨ֵ
    PID_calc(&PID_Gimbal_Pitch[0],motor_gimbal_pitch.angle,Gimbal_Data_p->Pitch_Set);
    
    // ������ 1 ��ת�ٳ��� 1000rpm����������Ϊ 0
    if(motor_gimbal[1].speed_rpm>300) motor_gimbal[1].speed_rpm=0;

    // �� Pitch ��ĵڶ��� PID ���м��㣬����Ϊ��� 1 ��ת�ٺ͵�һ�� PID �����
    PID_calc(&(PID_Gimbal_Pitch[1]), motor_gimbal[1].speed_rpm, PID_Gimbal_Pitch[0].out);
    // �����̨���ھ�Ĭģʽ������ֹͣ����
    if(Gimbal_Data_p->Gimbal_Mode == GIMBAL_SILENCE) Gimbal_cmd_CAN(0,0);
    // ���򣬸��� Yaw �� Pitch �� PID ������� CAN ����
    else Gimbal_cmd_CAN(PID_Gimbal_Yaw[1].out, PID_Gimbal_Pitch[1].out);    
}

// ������Gimbal_Yaw_PID_Calc
// ���ܣ����� Yaw ��� PID ��ز���
// ������Gimbal_Data_t ���͵�ָ�� Gimbal_Data_p��ָ����̨���ݽṹ
void Gimbal_Yaw_PID_Calc(Gimbal_Data_t * Gimbal_Data_p)
{
	
    // ��� Yaw ���趨ֵ����һ�ε��趨ֵ��ͬ
    if (Gimbal_Data_p->Yaw_Set!= Gimbal_Data_p->Last_Yaw_Set)
    {
        // ������һ�ε� Yaw �趨ֵ
        Gimbal_Data_p->Last_Yaw_Set = Gimbal_Data_p->Yaw_Set;
        // ���� Yaw �趨ֵ�Ͳο�ֵ֮��ľ���
        Gimbal_Data_p->Distance_Set = Gimbal_Data_p->Yaw_Set - Gimbal_Data_p->Yaw_Ref;
        // ���������� 180 �ȵ��������������� -180 �� 180 �ȷ�Χ
        if (Gimbal_Data_p->Distance_Set > 180) Gimbal_Data_p->Distance_Set = Gimbal_Data_p->Distance_Set - 360;
        if (Gimbal_Data_p->Distance_Set <-180) Gimbal_Data_p->Distance_Set = 360 + Gimbal_Data_p->Distance_Set;
        // ���²ο��趨ֵ
        Gimbal_Data_p->Setup_Ref = Gimbal_Data_p->Yaw_Ref;
    }
		if(Gimbal_Data_p->Distance_Ref -  Gimbal_Data_p->Distance_Set < 5) Gimbal_Data_p->Distance_Ref = Gimbal_Data_p->Distance_Set;
		if(Gimbal_Data_p->Distance_Ref -  Gimbal_Data_p->Distance_Set > -5) Gimbal_Data_p->Distance_Ref = Gimbal_Data_p->Distance_Set;
		
    // ���� Yaw �ο�ֵ���趨�ο�ֵ֮��ľ���
    Gimbal_Data_p->Distance_Ref = Gimbal_Data_p->Yaw_Ref - Gimbal_Data_p->Setup_Ref;
    // ���������� 180 �ȵ��������������� -180 �� 180 �ȷ�Χ
    if (Gimbal_Data_p->Distance_Ref > 180) Gimbal_Data_p->Distance_Ref = Gimbal_Data_p->Yaw_Ref - 360 - Gimbal_Data_p->Setup_Ref;
    if (Gimbal_Data_p->Distance_Ref <-180) Gimbal_Data_p->Distance_Ref = Gimbal_Data_p->Yaw_Ref + 360 - Gimbal_Data_p->Setup_Ref;
    // ��������������Ϊ�˱��ⶶ�������
		
    // �� Yaw ��ĵ�һ�� PID ���м��㣬����Ϊ����ο�ֵ�;����趨ֵ
    PID_calc(&(PID_Gimbal_Yaw[0]), Gimbal_Data_p->Distance_Ref, Gimbal_Data_p->Distance_Set);    
    // ������ 0 ��ת�ٳ��� 1500rpm����������Ϊ 0
    if(motor_gimbal[0].speed_rpm>1500) motor_gimbal[0].speed_rpm=0;
		
//		if(PID_Gimbal_Yaw[0].out - motor_gimbal[0].speed_rpm <= 0.5) PID_Gimbal_Yaw[0].out = 0;
//		if(PID_Gimbal_Yaw[0].out - motor_gimbal[0].speed_rpm >= -0.5) PID_Gimbal_Yaw[0].out = 0;
    // �� Yaw ��ĵڶ��� PID ���м��㣬����Ϊ��� 0 ��ת�ٺ͵�һ�� PID �����
    PID_calc(&(PID_Gimbal_Yaw[1]), motor_gimbal[0].speed_rpm, PID_Gimbal_Yaw[0].out);
}
