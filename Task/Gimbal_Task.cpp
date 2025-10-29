#include "Robot_Task.h"
#include "tasks.h"
#include "arm_math.h"
#include <math.h>
#include <vector>
#include "algorithm_SolveTrajectory.h"
#include "bsp_dwt.h"

Gimbal_Ctrl Gimbal;
float last_time;
float last_time2;
uint16_t anglesteering;
uint8_t nvflag=true;
uint8_t speed_flag;
extern bool fire_control;
uint8_t  aaaa;
uint16_t flag;
uint8_t up;
uint8_t down;
int8_t sign_=-1;
int8_t down_angle;
float lll;
float current = 30;
extern float TOP_dir;

float Tiaoshi_Yaw_Speed =0;

float angle_;
float low_passing =0.1f;


// 云台任务
void Gimbal_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	Gimbal.Gimbal_Init();
	/* Infinite loop */
	for (;;)
	{
		
		Gimbal.Behaviour_Mode();
		Gimbal.Feedback_Update();
    Gimbal.Control();

		if(Gimbal.Mode == GIMBAL_NO_MOVE)
		{
			//CAN_Cmd.SendData(&CAN_Cmd.Gimbal, 0, 0,0 ,0);			
			//CAN_Cmd.SendData(&CAN_Cmd.Fric, 0, 0);
			
			
			//CAN_Cmd.DM_MIT_SendData(&CAN_Cmd.Gimbal_DM_Pitch,0,0,0,0,-Gimbal.G_compensation_out);

		}
		else
		{
			
    
		Gimbal.Control_loop();
			
			
		//CAN_Cmd.SendData(&CAN_Cmd.Gimbal,0,0,Gimbal.Trigger.give_current,0);
		CAN_Cmd.SendData(&CAN_Cmd.Fric, Gimbal.Fric1.give_current, Gimbal.Fric2.give_current);
		   
						
		//CAN_Cmd.DM_MIT_SendData(&CAN_Cmd.Gimbal_DM_Pitch,0,0,0,0,-Gimbal.DM_Pitch.tor_set);
		

	

		}
		Gimbal.Stop_TickCount = DWT_GetTimeline_s();
		xQueueSend(Message_Queue, &ID_Data[GimbalData], 0);

		osDelay(GIMBAL_CONTROL_TIME);

	}
	/* USER CODE END StartDefaultTask */
}




// 初始化
void Gimbal_Ctrl::Gimbal_Init(void)
{
	uint8_t i;
	RC_Ptr = get_remote_control_point();
//	DM_Pitch.gimbal_motor_measure= &Message.Gimbal_pitch;
//	DM_Yaw.gimbal_motor_measure= &Message.Gimbal_yaw;

//   Pitch.speed_set=GIMBAL_PITCH_SPEED;
	//CAN_Cmd.start_motor (1);
	Gimbal.Flags .reset_damiao_pitch =false;
	
	Gimbal.Kff = 0.0f;
	
	Gimbal.Kmg=0.0f;//0.45
	
//	Yaw.gimbal_motor_measure = CAN_Cmd.Gimbal.Get_Motor_Measure_Pointer(0);
//	Pitch.gimbal_motor_measure = CAN_Cmd.Gimbal.Get_Motor_Measure_Pointer(1);
	DM_Yaw.gimbal_motor_measure= CAN_Cmd.Gimbal_DM_Yaw.Get_DM_Motor_Measure_Pointer();
	DM_Pitch.gimbal_motor_measure= CAN_Cmd.Gimbal_DM_Pitch.Get_DM_Motor_Measure_Pointer();
	Trigger.gimbal_motor_measure = CAN_Cmd.Gimbal.Get_Motor_Measure_Pointer(2);
	Fric1.gimbal_motor_measure = CAN_Cmd.Fric.Get_Motor_Measure_Pointer(0);
	Fric2.gimbal_motor_measure = CAN_Cmd.Fric.Get_Motor_Measure_Pointer(1);

//	Yaw.visual_offset_angle = Yaw.gimbal_motor_measure->ecd;

	// 手动
		/*达秒pitch*/
	PID.Init(&DM_Pitch.PositinPid, POSITION, DM_PITCH_POSITION_PID_KP, DM_PITCH_POSITION_PID_KI, DM_PITCH_POSITION_PID_KD, DM_PITCH_POSITION_PID_MAX_OUT, DM_PITCH_POSITION_PID_MAX_IOUT,DM_PITCH_POSITION_PID_BAND_I);
	PID.Init(&DM_Pitch.SpeedPid,   POSITION, DM_PITCH_SPEED_PID_KP,    DM_PITCH_SPEED_PID_KI,    DM_PITCH_SPEED_PID_KD,    DM_PITCH_SPEED_PID_MAX_OUT,    DM_PITCH_SPEED_PID_MAX_IOUT,   DM_PITCH_SPEED_PID_BAND_I);
	PID.Init(&DM_Yaw.PositinPid, POSITION, DM_YAW_POSITION_PID_KP, DM_YAW_POSITION_PID_KI, DM_YAW_POSITION_PID_KD, DM_YAW_POSITION_PID_MAX_OUT, DM_YAW_POSITION_PID_MAX_IOUT,DM_YAW_POSITION_PID_BAND_I);
	PID.Init(&DM_Yaw.SpeedPid,   POSITION, DM_YAW_SPEED_PID_KP,    DM_YAW_SPEED_PID_KI,    DM_YAW_SPEED_PID_KD,    DM_YAW_SPEED_PID_MAX_OUT,    DM_YAW_SPEED_PID_MAX_IOUT,   DM_YAW_SPEED_PID_BAND_I);


	// 视觉
  PID.Init(&DM_Yaw.Visual_PositinPid, POSITION, DM_Visual_YAW_POSITION_PID_KP, DM_Visual_YAW_POSITION_PID_KI, DM_Visual_YAW_POSITION_PID_KD, DM_Visual_YAW_POSITION_PID_MAX_OUT, DM_Visual_YAW_POSITION_PID_MAX_IOUT,DM_Visual_YAW_POSITION_PID_BAND_I);
	PID.Init(&DM_Yaw.Visual_SpeedPid,   POSITION, DM_Visual_YAW_SPEED_PID_KP,    DM_Visual_YAW_SPEED_PID_KI,    DM_Visual_YAW_SPEED_PID_KD,    DM_Visual_YAW_SPEED_PID_MAX_OUT,    DM_Visual_YAW_SPEED_PID_MAX_IOUT,   DM_Visual_YAW_SPEED_PID_BAND_I);
	 //导航


  // 摩擦轮
	PID.Init(&Fric1.SpeedPid, POSITION, FRIC1_SPEED_PID_KP, FRIC1_SPEED_PID_KI, FRIC1_SPEED_PID_KD, FRIC1_PID_MAX_OUT, FRIC1_PID_MAX_IOUT, FRIC1_PID_BAND_I);
	PID.Init(&Fric2.SpeedPid, POSITION, FRIC2_SPEED_PID_KP, FRIC2_SPEED_PID_KI, FRIC2_SPEED_PID_KD, FRIC2_PID_MAX_OUT, FRIC2_PID_MAX_IOUT, FRIC2_PID_BAND_I);
	// 拨弹轮
	PID.Init(&Trigger.PositinPid, POSITION, TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD, TRIGGER_ANGLE_PID_MAX_OUT, TRIGGER_ANGLE_PID_MAX_IOUT, TRIGGER_ANGLE_PID_BAND_I);
	PID.Init(&Trigger.SpeedPid, POSITION, TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT, TRIGGER_SPEED_PID_BAND_I); /*  */



	Data.pitch_offset_ecd = GIMBAL_PITCH_OFFSET_RAD;
	Data.pitch_max_angle = GIMBAL_PITCH_MAX_ANGLE;
	Data.pitch_min_angle =GIMBAL_PITCH_MIN_ANGLE;
	Data.Trigger_offset_ecd = Trigger.gimbal_motor_measure->ecd;
	Data.Gear = 0;
	Data.Fric_Gear[0] = FRIC_GEAR_SET_1;
	Data.Fric_Gear[1] = FRIC_GEAR_SET_2;
	Data.Fric_Gear[2] = FRIC_GEAR_SET_3;
	Data.Shoot_Frequency_m[0] = TRIGGER_ONE_S_SHOOT_NUM1;
	Data.Shoot_Frequency_m[1] = TRIGGER_ONE_S_SHOOT_NUM2;
	Data.Shoot_Frequency_m[2] = 20.0f;//TRIGGER_ONE_S_SHOOT_NUM3;
	Data.Loading_open = LOADING_OPEN_DUTY;
	Data.Loading_close = LOADING_CLOSE_DUTY;

//	for(i = 0; i < 3; i++)
//	{
//		switch(Data.Fric_Gear[i])
//		{
//		case 15:
//		Data.Fric_Set[i] = FRIC_SPEED_SET_15;
//		break;
//		case 18:
//		Data.Fric_Set[i] = FRIC_SPEED_SET_18;
//		break;
//		case 30:
//		Data.Fric_Set[i] = FRIC_SPEED_SET_30;
//		break;
//		default:
//		Data.Fric_Set[i] = FRIC_SPEED_SET_15;
//		break;
//		}
//	}
	//CAN_Cmd.DM_Motor_SetZeroT(&CAN_Cmd.Gimbal_DM_Yaw);      //达妙软件标零点
	Data.Fric_Set[2]=5900;//   6600/24m/s左右

	
	Feedback_Update();
	CAN_Cmd.DM_Motor_Enable(&CAN_Cmd.Gimbal_DM_Yaw);
}

// 数据更新
void Gimbal_Ctrl::Feedback_Update(void)
{
	
	Gimbal_DWT_dt = DWT_GetDeltaT(&Gimbal_DWT_Count);
	
	
//	Gimbal_data.Gimbal_yaw= Message.Gyro.Yaw_angle;
//	Gimbal_data.Gimbal_yaw_speed=Message.Gyro.Yaw_speed;	
	//Yaw.speed = Message.Gyro.Yaw_speed * 0.1f;
	//Yaw.angle += Message.Gyro.Yaw_speed*0.001;
	DM_Yaw.angle=Message.Gyro_data.Yaw_angle;
	DM_Yaw.speed=Message.Gyro_data.Yaw_speed * low_passing +DM_Yaw.speed*(1 - low_passing);
	DM_Pitch.angle=-DM_Pitch.gimbal_motor_measure->POS.fdata*180/PI;//Message.Gyro.Pitch_angle;
	DM_Pitch.speed=Message.Gyro_data.Pitch_speed;
	#ifdef motor_6020_pitch
	Pitch.angle = PITCH_MOTOR_REVERSE * motor_relative_ECD_to_angle(Pitch.gimbal_motor_measure->ecd, Data.pitch_offset_ecd);
	Pitch.speed = PITCH_MOTOR_REVERSE * Pitch.gimbal_motor_measure->speed_rpm*6*0.1;
	#endif
	#ifdef damiao_motor_pitch
	
	//记得处理达妙弧度转ecd  
//  DM_Pitch.angle_RAD=Message .Dm_Pitch .Motor_pos;
//	DM_Yaw.angle = 
//		DM_Pitch.angle=Message.Gyro.Pitch
	
	#endif
	Trigger.speed = Trigger.gimbal_motor_measure->speed_rpm;
	Fric1.speed = Fric1.gimbal_motor_measure->speed_rpm;
	Fric2.speed = Fric2.gimbal_motor_measure->speed_rpm;

//	Yaw.visual_send_angle = PITCH_MOTOR_REVERSE * motor_relative_ECD_to_angle(Yaw.gimbal_motor_measure->ecd, Yaw.visual_offset_angle);

	
	Trig.new_angle = motor_relative_ECD_to_angle(Trigger.gimbal_motor_measure->ecd, Data.Trigger_offset_ecd);
	if(Trig.new_angle - Trig.last_angle > 180)
	{
		Trig.cycle--;
	}
	else if(Trig.new_angle - Trig.last_angle < -180)
	{
		Trig.cycle++;
	}
	Trig.last_angle = Trig.new_angle;
	Trigger.angle = Trig.new_angle + Trig.cycle * 360.0f;

	Data.Gear = 0;
//	if(Flags.RC_Flag == false)
//	{
//	//Data.Shoot_Frequency = Data.Shoot_Frequency_m[Data.Gear - 1];
//		Data.Shoot_Frequency = Data.Shoot_Frequency_m[2];//无服务器电脑操控
//	}
//	if(Flags.RC_Flag == true)
//	{
//	Data.Shoot_Frequency = Data.Shoot_Frequency_m[2];
//		
//	}
	
//	暂无火控瞄了就泼水
//////	if(Message.New_VisualR .is_Outpose ==0)
//////	{
//////	Flags.AutoShoot_Flag = AUTOSHOOT;
//////	}
//////	else
//////	{
//////   Flags.AutoShoot_Flag=0;//火控
//////	}
	Flags.AutoShoot_Flag = AUTOSHOOT;
	
	//哨兵枪管热量控制
	if(Message .robo->game_robot_state.shooter_barrel_heat_limit-Message.robo->power_heat_data.shooter_17mm_1_barrel_heat<80)
	{
	   	Data.Shoot_Frequency =Data.Shoot_Frequency_m[2];//Data.Shoot_Frequency =Data.Shoot_Frequency_m[0];
	}
	else
	{
			Data.Shoot_Frequency = Data.Shoot_Frequency_m[2];
	}
		
	
//	
//	if(Mode == GIMBAL_NAV)
//	{
//	 Data.Shoot_Frequency = Data.Shoot_Frequency_m[2];
//	}//哨兵用

	
	
	
	Statistic_Update(xTaskGetTickCount());
}


// 按键和拨杆控制
void Gimbal_Ctrl::Behaviour_Mode(void)
{
	if(Message .robo->game_robot_state.power_management_gimbal_output==0 )
	{
	  Gimbal .Flags .reset_damiao_pitch =false;
	}
	
	if(Gimbal.DM_Yaw.gimbal_motor_measure->state!=1)
	{
		CAN_Cmd.DM_Motor_Enable(&CAN_Cmd.Gimbal_DM_Yaw);
		osDelay(1);
	}
	
//	if(Gimbal.DM_Pitch.gimbal_motor_measure->state!=1)
//	{
//		CAN_Cmd.DM_Motor_Enable(&CAN_Cmd.Gimbal_DM_Pitch);//后续根据达妙反馈进行更改
//		osDelay(1);
//	}
//	
/*作为靶车的时候，不对按键处理，关闭按键*/
/*10月26日*/
/*	if(switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]))
	{
		Flags.RC_Flag = false;//
		
	}
	else
	{
		Flags.RC_Flag = true;
	}
*/
//引入部分逻辑
if(switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT])&&switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Flags.RC_Flag = false;//
	}
	else
	{
		Flags.RC_Flag = true;
	}

	if(Flags.RC_Flag == false)
	{ // 按键控制
		if(RC.read_key(&RC.Key.C, single, true))
		{ // 无力
			Mode = GIMBAL_NO_MOVE;
		}
		if(RC.read_key(&RC.Key.X, single, true) && Mode == GIMBAL_NO_MOVE)
		{ // 启动后为人为控制
			Mode = GIMBAL_Normal;
		}
		// 视觉开关
		RC.read_key(&RC.Press.R, even, &Flags.Visual_Flag);
		// 能量机关开关
		RC.read_key(&RC.Key.E, single, &Flags.Energy_Flag);
		// 装弹开关
		RC.read_key(&RC.Key.R, single, &Flags.Loading_Flag);
		// 摩擦轮开关
		RC.read_key(&RC.Key.Q, single, &Flags.Fric_Flag);
		// 拨弹轮开关
		if(Flags.Fric_Flag == true && Flags.AutoShoot_Flag == true)
		{
			RC.read_key(&RC.Press.L, even, &Flags.Shoot_Flag);
		}
		else if(Flags.Fric_Flag == true && Flags.AutoShoot_Flag == false)
		{
			Flags.Shoot_Flag = RC.read_key(&RC.Press.L, single, true);
		}
		RC.read_key(&RC.Key.ctrl, even, &Flags.Shoot_Reversal_Flag);
	}

	// 拨杆控制
	if(switch_is_down(RC_Ptr->rc.s[CHANNEL_RIGHT]))
	{
		Mode = GIMBAL_NO_MOVE;

	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]))
	{
		Mode = GIMBAL_Normal;
		Flags.Visual_Flag = false;
		Flags.Fric_Flag = false;

	}
	else if(switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT])&&switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
	  Flags.Fric_Flag = true;
		Flags.Shoot_Flag = true;
	}
#if RC_CONTRAL_MODE == 0
	if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Mode = GIMBAL_Normal;
		Flags.Visual_Flag = false;
		Flags.Shoot_Flag = false;
		
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Mode = GIMBAL_Normal;
		
		 Flags.Fric_Flag = false;
		Flags.Visual_Flag = false;
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		
		if(Mode!=GIMBAL_AIM)
		{			
		   Mode = GIMBAL_NAV;
		}
		
		Flags.Fric_Flag = true;//true;//true;
		Flags.Visual_Flag = true;//true;
////	if(Message.New_VisualR.tracking==true&&fire_control==true&&Message.New_VisualR .is_Outpose ==1)//火控
////	{
////	  Flags.Shoot_Flag = true;
////	}

////	else	if(Message.New_VisualR.tracking==true&&Message.New_VisualR .is_Outpose ==0)
////	{
////	  Flags.Shoot_Flag = true;
////	}
////	else 
////	{
////		Flags.Shoot_Flag = false;
////	
////	}


//	if(Message.VisualR.fire==false)
//	 
//	{
//		Flags.Shoot_Flag = false;
//	
//	}
	
	
	}
#elif RC_CONTRAL_MODE == 1
	if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Flags.Visual_Flag = false;
		Flags.Fric_Flag = false;
		Flags.Shoot_Flag = false;
//			Flags.Fric_Flag = true;
//		Flags.Shoot_Flag = true;//测试拨弹轮
		Flags.Loading_Flag = false;
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		Flags.Visual_Flag = true;;
		//Flags.Fric_Flag = true;
		//Flags.Shoot_Flag = false;
		//		Flags.Shoot_Flag = true;
		//Flags.Shoot_Flag = true;
		Flags.Loading_Flag = false;
	}
	else if(switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
			//	Flags.Shoot_Flag = true;
		//Flags.Fric_Flag = true;
		
		
		
		//Flags.Visual_Flag = true;
		//Flags.Fric_Flag = true;
	//	Flags.Shoot_Flag = true;
	//	Flags.Loading_Flag = false;
	}
#endif
	else if(switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
	{
		HAL_NVIC_SystemReset();
	}
	Flag_Behaviour_Control();

}


// 标志位行为设置
void Gimbal_Ctrl::Flag_Behaviour_Control()
{
	if(Message.robo->game_robot_state.robot_id != 0 && Message.robo->game_robot_state.current_HP == 0)
	{//死亡处理
		Flags.Shoot_Flag = false;
		Flags.Visual_Flag = false;
		Flags.Energy_Flag = false;
	  nvflag=true;
		Mode = GIMBAL_NO_MOVE;
		DM_Yaw.angle_set = DM_Yaw.angle;
		DM_Pitch.angle_set = DM_Pitch.angle;
	}

//	if(Flags.Fric_Flag == true && Guard.Return(ChassisData) == true )
//	{

////		if(Flags.RC_Flag == false)
////			{
////				Data.FricSpeedSet = Data.Fric_Set[Data.Gear - 1];
////			}
////		if(Flags.RC_Flag == true)
////			{
////				Data.FricSpeedSet = Data.Fric_Set[1];
////			}
//		Data.FricSpeedSet = Data.Fric_Set[2];

//	}
//	 if(Flags.Fric_Flag == true && (Guard.Return(ChassisData) == false ))
	 if(Flags.Fric_Flag == true )
	{
		Data.FricSpeedSet = Data.Fric_Set[2];
		//Data.FricSpeedSet = Data.Fric_Set[2];//遥控器测摩擦轮//无服务器电脑测试
	//	Data.FricSpeedSet = 7000;
	}
	
	else
	{
		Data.FricSpeedSet = 0.0f;
		Flags.Shoot_Flag = false;
		//Flags.Shoot_Flag = true;//测拨弹
	}
	Fric1.speed_set =  Data.FricSpeedSet;
	Fric2.speed_set = -Data.FricSpeedSet;
	
	
	
	if(Flags.Shoot_Flag == 1)
	{
		if(Flags.AutoShoot_Flag == true)
		{
			Trigger.speed_set = TRIGGER_MOTOR_REVERSE * Data.Shoot_Frequency * 60.0f / TRIGGER_ONCE_SHOOT_NUM * TRIGGER_REDUCTION_RATIO;
		}
		else
		{
			Trigger.angle_set +=TRIGGER_REDUCTION_RATIO * TRIGGER_MOTOR_REVERSE * 360.0f / TRIGGER_ONCE_SHOOT_NUM;

		}
	}
//	aaaa=Flags.Shoot_Flag ;
	else
	{
		if(Flags.AutoShoot_Flag == true)
		{
		
	
		
			//Trigger.angle_set +=TRIGGER_REDUCTION_RATIO * TRIGGER_MOTOR_REVERSE * 360.0f / TRIGGER_ONCE_SHOOT_NUM;

		
			Trigger.speed_set = 0.0f;
		}
	 if(Flags.Fric_Flag == false)
		{
			Trigger.angle_set = Trigger.angle;
		}
	}

//	if(Flags.Shoot_Reversal_Flag == true)
//	{
//		if(Flags.AutoShoot_Flag == true)
//		{
//			Trigger.speed_set = -TRIGGER_MOTOR_REVERSE * Data.Shoot_Frequency * 60.0f / TRIGGER_ONCE_SHOOT_NUM * TRIGGER_REDUCTION_RATIO;
//		}
//		else
//		{
//			Trigger.angle_set -= TRIGGER_REDUCTION_RATIO * TRIGGER_MOTOR_REVERSE * 60.0f / TRIGGER_ONCE_SHOOT_NUM;
//		}
//	}

//	if(Flags.Loading_Flag == true)
//	{
//		pwmWrite(PWM_IO, Data.Loading_open);		
//	}
//	else
//	{
//		pwmWrite(PWM_IO, Data.Loading_close);
//	}

//	if(Flags.Visual_Flag == true && Message.New_VisualR.Goal == true && Mode != GIMBAL_NO_MOVE)//有修改
//	{
//		if(Flags.Energy_Flag == true)
//		{
//			Mode = GIMBAL_ENERGY;
//		}
//		else
//		{
//			Mode = GIMBAL_AIM;
//		}
//	}
	
	if(Flags.Visual_Flag == true &&  Mode != GIMBAL_NO_MOVE&&Message.VisualR.distance!=-1)//有修改
	{

			Mode = GIMBAL_AIM;
	  if(Message.VisualR.fire==true)
		{
		Flags.Shoot_Flag = true;//true
		}
		else
		{
			Flags.Shoot_Flag=false;
			
		
		}
	
		
	  }
	else 
	{
	  Flags.Shoot_Flag = false;
	
	  if(Mode != GIMBAL_NO_MOVE&&Mode !=GIMBAL_Normal)
	   {
			 Mode = GIMBAL_NAV;
		   Flags.Shoot_Flag = false;
	    }
		 
//		if(Mode != GIMBAL_NO_MOVE)
//	   {
//			 Mode = GIMBAL_NAV;
//		   Flags.Shoot_Flag = false;
//	    }
		 
   }
//	if(Flags.Shoot_Flag == true)
//	{
//		if(Flags.AutoShoot_Flag == true)
//		{
//			Trigger.speed_set = TRIGGER_MOTOR_REVERSE * Data.Shoot_Frequen	cy * 60.0f / TRIGGER_ONCE_SHOOT_NUM * TRIGGER_REDUCTION_RATIO;
//		}
//		else
//		{
//			Trigger.angle_set +=TRIGGER_REDUCTION_RATIO * TRIGGER_MOTOR_REVERSE * 360.0f / TRIGGER_ONCE_SHOOT_NUM;

//		}
//	}
//	
//		
//	else
//	{
//		if(Flags.AutoShoot_Flag == true)
//		{
//		
//	
//		
//			//Trigger.angle_set +=TRIGGER_REDUCTION_RATIO * TRIGGER_MOTOR_REVERSE * 360.0f / TRIGGER_ONCE_SHOOT_NUM;

//		
//			Trigger.speed_set = 0.0f;
//		}
//		else if(Flags.Fric_Flag == false)
//		{
//			Trigger.angle_set = Trigger.angle;
//		}
//	}

//	if(Flags.Shoot_Reversal_Flag == true)
//	{
//		if(Flags.AutoShoot_Flag == true)
//		{
//			Trigger.speed_set = -TRIGGER_MOTOR_REVERSE * Data.Shoot_Frequency * 60.0f / TRIGGER_ONCE_SHOOT_NUM * TRIGGER_REDUCTION_RATIO;
//		}
//		else
//		{
//			Trigger.angle_set -= TRIGGER_REDUCTION_RATIO * TRIGGER_MOTOR_REVERSE * 60.0f / TRIGGER_ONCE_SHOOT_NUM;
//		}
//	}

	if(Mode == Last_Mode)
	{
		return;
	}
	else if(Mode != Last_Mode)
	{
		DM_Yaw.feedforward = 0;
		last_target = 0;
		DM_Yaw.angle_set = DM_Yaw.angle;
		DM_Pitch.angle_set = DM_Pitch.angle;
		Trigger.angle_set = Trigger.angle;
		Last_Mode = Mode;
		
		
		nvflag=true;
	}
}

// 摇杆和鼠标输入
void Gimbal_Ctrl::RC_to_Control(fp32 *yaw_set, fp32 *pitch_set)
{
	if(IsInvalid(*yaw_set) || IsInvalid(*pitch_set))
	{
		return;
	}
	// 遥控器原始通道值
	int16_t yaw_channel, pitch_channel;
	fp32 yaw_set_channel, pitch_set_channel;

	if(Flags.RC_Flag == true)
	{
		// 将遥控器的数据处理死区
		rc_deadline_limit(RC_Ptr->rc.ch[YawChannel], yaw_channel, RC_DEADLINE);
		rc_deadline_limit(RC_Ptr->rc.ch[PitchChannel], pitch_channel, RC_DEADLINE);

		yaw_set_channel = -(yaw_channel * Yaw_RC_SEN);
		pitch_set_channel = pitch_channel * Pitch_RC_SEN;

	}
	else if(Flags.RC_Flag == false)
	{
		yaw_set_channel = -(RC_Ptr->mouse.x * Yaw_Mouse_SEN);
		pitch_set_channel =-(RC_Ptr->mouse.y * Pitch_Mouse_SEN);
	}

	*yaw_set = yaw_set_channel;
	*pitch_set = pitch_set_channel;
}

// 云台控制设定
void Gimbal_Ctrl::Behaviour_Control(fp32 *yaw_set, fp32 *pitch_set)
{
	
	/*
	DM_Yaw.VisualR_Error_angle = (Message.VisualR.yaw - DM_Yaw.VisualR_last_yaw);
	if(DM_Yaw.VisualR_Error_angle > 270.0f) DM_Yaw.VisualR_Yaw_cycle--;
	if(DM_Yaw.VisualR_Error_angle < -270.0f) DM_Yaw.VisualR_Yaw_cycle++;
	
	//转换后视觉自瞄Yaw轴angle\
	DM_Yaw.VisualR_cycle_yaw
	DM_Yaw.VisualR_cycle_yaw = Message.VisualR.yaw + DM_Yaw.VisualR_Yaw_cycle * 360.0f;
	
	DM_Yaw.VisualR_last_yaw = Message.VisualR.yaw;
	*/
	
	
	if(Mode == GIMBAL_NO_MOVE)
	{
		*yaw_set = 0;
		*pitch_set = 0;
	}
	else if(Mode == GIMBAL_Normal)
	{
		RC_to_Control(yaw_set, pitch_set);
		

	 }
	else if(Mode == GIMBAL_AIM || Mode == GIMBAL_ENERGY)
	{
		DM_Pitch.PositinPid.max_out = 10;
    
		
			nvflag=true;
		
//		if(Stop_TickCount-last_time2>=1.5f){
//		TOP_dir = -TOP_dir;
//		
//		last_time2 = Stop_TickCount;	
//}
		
		
		process_angle(DM_Yaw.VisualR_LuBo_yaw,Message.Gimbal_IMU_data.Yaw_Angle.int_16 * FP32_MPU_RAD,&Data.VS_yaw_Setangle);
		*yaw_set=Data.VS_yaw_Setangle;
		*pitch_set=Message.VisualR.pitch;
		
		
		
		
		
		//last_target = 0.3*Message.VisualR.yaw + 0.7*last_target;
		
		
	}
	else if(Mode == GIMBAL_NAV)
	{
		
		
		
		
		DM_Pitch.PositinPid.max_out = 2.2;
		
		if(Stop_TickCount-last_time>=0.7f)			
		{
		  sign_=-sign_;
			last_time = Stop_TickCount;
	  }
	 *pitch_set=sign_*20.0f;



//	/*--------------比赛巡逻yaw轴旋转-----------------------------------------*/
	//if(Message.robo->game_robot_state.power_management_gimbal_output==0 )
	//{//死亡处理
  //    *yaw_set=0;
	//}
	//else if(Message.robo->game_robot_state.power_management_gimbal_output==1)
	//{
			if(Message.NAV.Top_state == 1)
				{
					if(Corres.visual.robot_HP != Corres.last_HP  && Mode == GIMBAL_NAV){
					
						*yaw_set=-70;
						Corres.last_HP = Corres.visual.robot_HP;
					}
					else if(Stop_TickCount-last_time2>=1.5f){
					*yaw_set=-45;
					last_time2 = Stop_TickCount;
					
					
					}
				}
			else
			{
				if(Stop_TickCount-last_time2>=1.5f)
				{
					*yaw_set=-45;
					last_time2 = Stop_TickCount;
				
				
				}
				
				
			}
		
		
		
  //}
		
		
//		if(Message.New_VisualR.tracking==true)
//		{
//		*yaw_set=Message.New_VisualR.New_Visual_Yaw;
//		*pitch_set=-(Message.New_VisualR.New_Visual_Pitch-5);
//		}
//		else
//		{
    //   
//			if(Gimbal.Pitch .angle ==	(Data.pitch_max_angle-2))
//			{
//		   	*pitch_set=10;
//			}
//			else if(Gimbal.Pitch .angle ==	(Data.pitch_min_angle-2))
//			{
//			  *pitch_set=-10;
//			
//			}
//		
//		}
		
		
		
//		RC_to_Control(yaw_set, pitch_set);
	}
}

//电机设定量控制
void Gimbal_Ctrl::Control(void)
{
	
	fp32 yaw_set;
	fp32 pitch_set;

	Behaviour_Control(&yaw_set, &pitch_set);

	if(Mode == GIMBAL_NO_MOVE)
	{
		yaw_set = 0;
    pitch_set = 0;
		Fric1.speed_set = 0.0f;
		Fric2.speed_set = 0.0f;
	}
	else if(Mode == GIMBAL_Normal)
	{
		if(Stop_TickCount-last_time2>=0.012f){
		current+=0.5;
		last_time2 = Stop_TickCount;
	}
	if(current>48)
		current = 30;
	
	lll = current;
		
		DM_Yaw.angle_set += yaw_set;
	 //DM_Yaw.angle_set = current;
    DM_Pitch.angle_set += pitch_set;

				
	}
	else if(Mode == GIMBAL_AIM || Mode == GIMBAL_ENERGY)
	{
		
		
		
		DM_Yaw.angle_set = yaw_set;
		DM_Pitch.angle_set = pitch_set;
	}
	else if(Mode == GIMBAL_NAV)
	{	
		DM_Pitch.angle_set= pitch_set;
		DM_Yaw.angle_set+=yaw_set;
		
	}

//	if(Trigger.gimbal_motor_measure->given_current > TRIGGER_BLOCKED_CURRENT && TRIGGER_MOTOR_REVERSE == -1)
//	{ // 拨弹轮防堵转
//		Trigger.angle_set = Trigger.angle_set - TRIGGER_MOTOR_REVERSE * TRIGGER_BLOCKED_ANGLE;
//		Trigger.speed_set = -TRIGGER_MOTOR_REVERSE * TRIGGER_BLOCKED_SPEED;
//	}
//	else if(Trigger.gimbal_motor_measure->given_current < -TRIGGER_BLOCKED_CURRENT && TRIGGER_MOTOR_REVERSE == 1)
//	{ // 拨弹轮防堵转
//		Trigger.angle_set = Trigger.angle_set - TRIGGER_MOTOR_REVERSE * TRIGGER_BLOCKED_ANGLE;
//		Trigger.speed_set = TRIGGER_MOTOR_REVERSE * TRIGGER_BLOCKED_SPEED;
//	}
	DM_Pitch.angle_set = constrain(DM_Pitch.angle_set, Data.pitch_min_angle, Data.pitch_max_angle);
}


// 云台控制PID运算
void Gimbal_Ctrl::Control_loop(void)
{
	fp32 YAW_out = 0;
	fp32 PITCH_out = 0;
  G_compensation_out= Kmg*cos(Gimbal.DM_Pitch.angle*PI/180.f);
	
	
	if(Mode == GIMBAL_Normal)
	{
	
	
		DM_Yaw.feedforward = Kff * ( DM_Yaw.angle_set - last_target );
		
		if(DM_Yaw.feedforward >=5)
		    DM_Yaw.feedforward = 5;
		else if(DM_Yaw.feedforward <=-5)
			 DM_Yaw.feedforward = -5;
		
		last_target = DM_Yaw.angle_set;
		
		
		
//调试部分
		// PID.Calc(&DM_Yaw.PositinPid, DM_Yaw.angle, DM_Yaw.angle_set);
		PID.Calc(&DM_Yaw.PositinPid, DM_Yaw.angle, DM_Yaw.angle_set);
		PID.Calc(&DM_Yaw.SpeedPid,DM_Yaw.speed, DM_Yaw.PositinPid.out);
		// PID.Calc(&DM_Yaw.SpeedPid,DM_Yaw.speed, Tiaoshi_Yaw_Speed);


		PID.Calc(&DM_Pitch.PositinPid, DM_Pitch.angle, DM_Pitch.angle_set);
		PID.Calc(&DM_Pitch.SpeedPid, DM_Pitch.speed,DM_Pitch.PositinPid.out);

		
		YAW_out =DM_Yaw.SpeedPid.out + DM_Yaw.feedforward;
		PITCH_out =forwardfeed_pitch(DM_Pitch.SpeedPid.out)+G_compensation_out;
		
		
	}
	else if(Mode == GIMBAL_AIM)
	{
		
		//if((DM_Yaw.angle_set<0 && DM_Yaw.angle_set<-3.0f)  ||(DM_Yaw.angle_set>0 && DM_Yaw.angle_set>3.0f)){
		/*视觉前馈*/
		DM_Yaw.VisualR_LuBo_yaw = 0.1*Message.VisualR.yaw + 0.9*last_target;
	
		DM_Yaw.feedforward = Kff * ( DM_Yaw.VisualR_LuBo_yaw - last_target );
		
		if(DM_Yaw.feedforward >=8)
		    DM_Yaw.feedforward = 8;
		else if(DM_Yaw.feedforward <=-8)
			 DM_Yaw.feedforward = -8;
		
		last_target = DM_Yaw.VisualR_LuBo_yaw;
	  //}
		//else{
		//	DM_Yaw.feedforward = 0;
		//	
		//}
		
		
		
    PID.Calc(&DM_Yaw.Visual_PositinPid, -DM_Yaw.angle_set, 0);
		PID.Calc(&DM_Yaw.Visual_SpeedPid,DM_Yaw.speed, DM_Yaw.Visual_PositinPid.out);
		
		//PID.Calc(&DM_Yaw.PositinPid, DM_Yaw.angle, DM_Yaw.angle_set);
		//PID.Calc(&DM_Yaw.SpeedPid,DM_Yaw.speed, DM_Yaw.PositinPid.out);
		
		

		PID.Calc(&DM_Pitch.PositinPid, DM_Pitch.angle, DM_Pitch.angle_set);
		PID.Calc(&DM_Pitch.SpeedPid, DM_Pitch.speed,DM_Pitch.PositinPid.out);

		
		YAW_out =DM_Yaw.Visual_SpeedPid.out + DM_Yaw.feedforward;
		PITCH_out = DM_Pitch.SpeedPid.out+G_compensation_out;
		
		

			
	}
	else if(Mode == GIMBAL_ENERGY)
	{
//		PID.Calc(&Yaw.EnergyPositinPid, Yaw.angle, Yaw.angle_set);
//		PID.Calc(&Yaw.EnergySpeedPid, Yaw.speed, Yaw.EnergyPositinPid.out);

////		PID.Calc(&Pitch.EnergyPositinPid, Pitch.angle, Pitch.angle_set);
////		PID.Calc(&Pitch.EnergySpeedPid, Pitch.speed, Pitch.EnergyPositinPid.out);

//		YAW_out = Yaw.EnergySpeedPid.out;
////		PITCH_out = Pitch.EnergySpeedPid.out-Kmg*cos(Gimbal.DM_Pitch.angle*PI/180.f);
	}
	else if(Mode == GIMBAL_NAV)
	{
    PID.Calc(&DM_Yaw.PositinPid, DM_Yaw.angle, DM_Yaw.angle_set);//角度环
		PID.Calc(&DM_Yaw.SpeedPid,DM_Yaw.speed, DM_Yaw.PositinPid.out);//速度环

		 PID.Calc(&DM_Pitch.PositinPid, DM_Pitch.angle, DM_Pitch.angle_set);
		PID.Calc(&DM_Pitch.SpeedPid, DM_Pitch.speed,Tiaoshi_Yaw_Speed);

		
		YAW_out =DM_Yaw.SpeedPid.out;//forwardfeed(DM_Yaw.SpeedPid.out);
		PITCH_out =forwardfeed_pitch(DM_Pitch.SpeedPid.out)+G_compensation_out;
////		PID.Calc(&Yaw.PositinPid, Yaw.angle, Yaw.angle_set);
////		PID.Calc(&Yaw.SpeedPid, Yaw.speed * 5.0f, Yaw.PositinPid.out);

//////		PID.Calc(&Pitch.PositinPid, Pitch.angle, Pitch.angle_set);
//////		PID.Calc(&Pitch.SpeedPid, Pitch.speed, Pitch.PositinPid.out);

//////		YAW_out = Yaw.SpeedPid.out;
////		YAW_out =forwardfeed(Yaw.SpeedPid.out);
////    PITCH_out=1*speed_flag; 
		
	}


	PID.Calc(&Fric1.SpeedPid, Fric1.speed, Fric1.speed_set);
	PID.Calc(&Fric2.SpeedPid, Fric2.speed, Fric2.speed_set);


//	if(Flags.AutoShoot_Flag == false)
//	{
//		PID.Calc(&Trigger.PositinPid, Trigger.angle, Trigger.angle_set);
//		PID.Calc(&Trigger.SpeedPid, Trigger.speed, Trigger.PositinPid.out);
//	}
	 if(Flags.AutoShoot_Flag == true)
	{
		PID.Calc(&Trigger.SpeedPid, Trigger.speed, Trigger.speed_set);
	}

	DM_Yaw.tor_set = -YAW_out;   /*齿轮传动，力矩方向相方*/
	DM_Pitch.tor_set = PITCH_out;
	Fric1.give_current = Fric1.SpeedPid.out;
	Fric2.give_current = Fric2.SpeedPid.out;
	Trigger.give_current = Trigger.SpeedPid.out;
}

// //规整ANGLE后转化成角度ANGLE，范围±180

//fp32 Gimbal_Ctrl::Gyro_relative_angle_to_angle(fp32 angle, fp32 offset_angle)
//{

//	fp32 relative_ecd = angle - offset_angle;
//	if (relative_ecd > 180){
//		relative_ecd -= 360;
//	}
//	else if (relative_ecd <-180 )
//	{
//		relative_ecd += 360;
//	}
//		number[3]=relative_ecd;
//	return relative_ecd;
//}

// 规整ECD后转化成角度DEG，范围±180
fp32 Gimbal_Ctrl::motor_relative_ECD_to_angle(uint16_t angle, uint16_t offset_ecd)
{
	fp32 relative_angle;
	int32_t relative_ecd = angle - offset_ecd;
	if (relative_ecd > Half_ecd_range)
	{
		relative_ecd -= ecd_range;
	}
	else if (relative_ecd < -Half_ecd_range)
	{
		relative_ecd += ecd_range;
	}
	relative_angle = relative_ecd * ECD_TO_DEG;
	return relative_angle;
}

void rc_key_v_fresh_Gimbal(RC_ctrl_t *RC)
{
	Gimbal.RC.rc_key_v_set(RC);
}

Gimbal_Ctrl *get_gimbal_ctrl_pointer(void)
{
	return &Gimbal;
}

/*
采样周期：T=0.001秒(代码一毫秒运行一次所以这里取)
转动惯量：J=1
摩擦系数： f=1
角速度/力矩：G(s)=1/(s+1)
前馈环节：Gf(s)=s+1;
输出：out=in'+in=(in-last_in)/T+in

*/


float last_in=0;
float T=1.0f;
float Gimbal_Ctrl::forwardfeed(float in)
{
	float out;
	out=(in-last_in)/T+in;
	last_in=in;
	return out;

}

float last_in_pitch=0;
float Gimbal_Ctrl::forwardfeed_pitch(float in)
{
	float out;
	out=(in-last_in_pitch)/T+in;
	last_in_pitch=in;
	return out;

}

void  Gimbal_Ctrl::process_angle(float visual_angle,float yaw_gyro,float *aim_angle) {
    if(visual_angle-yaw_gyro >= 270)
      *aim_angle=(visual_angle-yaw_gyro-360);
		
		else if(visual_angle-yaw_gyro<=-270)
      *aim_angle= (visual_angle-yaw_gyro+360);
		
		else *aim_angle=visual_angle-yaw_gyro;
   
}


	

