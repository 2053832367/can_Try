#include "Message_Task.h"
#include "tasks.h"
#include "algorithm_SolveTrajectory.h"
Message_Ctrl Message;
uint8_t arrrr[31];
uint8_t leneeee;


void Message_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	Message.Init();
  /* Infinite loop */
  for(;;)
  {		
    if(xQueueReceive(Message_Queue, &ID_Data[MessageData], portMAX_DELAY))
		{
			Guard.Feed(ID_Data[MessageData].Data_ID);
			Guard.Feed(MessageData);
			Message.Statistic_Update(xTaskGetTickCount());
		}
		if(RC_data_is_error(Message.RC_Ptr))
	  {
				slove_RC_lost();
		}
  }
  /* USER CODE END StartDefaultTask */
}

void CAN1_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t CAN1_Rx_Data;
  /* Infinite loop */
  for(;;)
  {		
		if(xQueueReceive(CAN1_Rx_Queue, &CAN1_Rx_Data, portMAX_DELAY))
		{
			Message.CAN1_Process((CanRxMsg *)CAN1_Rx_Data.Data_Ptr);
			Guard.Feed(CanData1);
		}
  }
  /* USER CODE END StartDefaultTask */
}

void CAN2_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t CAN2_Rx_Data;
  /* Infinite loop */
  for(;;)
  {		
    if(xQueueReceive(CAN2_Rx_Queue, &CAN2_Rx_Data, portMAX_DELAY))
		{
			Message.CAN2_Process((CanRxMsg *)CAN2_Rx_Data.Data_Ptr);
			Guard.Feed(CanData2);
		}
  }
  /* USER CODE END StartDefaultTask */
}

void CAN3_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t CAN3_Rx_Data;
  /* Infinite loop */
  for(;;)
  {		
    if(xQueueReceive(CAN3_Rx_Queue, &CAN3_Rx_Data, portMAX_DELAY))
		{
			Message.CAN3_Process((CanRxMsg *)CAN3_Rx_Data.Data_Ptr);
			Guard.Feed(CanData3);
		}
  }
  /* USER CODE END StartDefaultTask */
}

void Serial_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t Serial_Rx_Data;
  /* Infinite loop */
  for(;;)
  {		
    if(xQueueReceive(Serial_Rx_Queue, &Serial_Rx_Data, portMAX_DELAY))
		{
			switch(Serial_Rx_Data.Data_ID)
			{
			case SerialData3:
			Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial3_Ctrl);
			break;
			case SerialData10:
			Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial10_Ctrl);
			break;
			case SerialData5:
			Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial5_Ctrl);
			break;
			case SerialData7:
			Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial7_Ctrl);
			break;
			case SerialData1:
			Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial1_Ctrl);
			break;
			default:
			break;
			}
			Guard.Feed(Serial_Rx_Data.Data_ID);
		}
  }
  /* USER CODE END StartDefaultTask */
}

void Referee_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t Referee_Rx_Data;
  /* Infinite loop */
  for(;;)
  {		
		if(xQueueReceive(Referee_Rx_Queue, &Referee_Rx_Data.Data_Ptr, portMAX_DELAY))
		{
			referee_data_solve( &(((uint8_t *)Referee_Rx_Data.Data_Ptr)[1]) );
			//Message.FricSpeed_pid();
			Guard.Feed(RefereeData);
		}
  }
  /* USER CODE END StartDefaultTask */
}

void DR16_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t DR16_Rx_Data;	
	//remote control data 
	//遥控器控制变量
	
  /* Infinite loop */
  for(;;)
  {		
		if(xQueueReceive(DR16_Rx_Queue, &DR16_Rx_Data.Data_Ptr, portMAX_DELAY))
		{
			sbus_to_rc(&( ((uint8_t *)(DR16_Rx_Data.Data_Ptr))[1]) ,Message.RC_Ptr);
			rc_key_v_fresh_Gimbal(Message.RC_Ptr);
			rc_key_v_fresh_Chassis(Message.RC_Ptr);
			Guard.Feed(RCData);
		if(RC_data_is_error(Message.RC_Ptr))
			{
				slove_data_error();
			}
		}
  }
  /* USER CODE END StartDefaultTask */
}

void Message_Ctrl::Init()
{	
	Message.RC_Ptr = &RC_ctrl;
	CAN_ALL_Init();
	Prefence_Init();
	Serial_ALL_Init();
	robo = get_robo_data_Point();
}

void Message_Ctrl::Serialx_Hook(uint8_t *Rx_Message, Serialctrl *Serialx_Ctrl)
{
	if(Serialx_Ctrl == &VISUAL_SERIAL)
	{
		Visual_Serial_Hook(Rx_Message);
	}
	if(Serialx_Ctrl == &JUDGE_SERIAL)
	{
		xQueueSend(Referee_Rx_Queue, &Rx_Message, 0);
	}
//	if(Serialx_Ctrl == &NAV_SERIAL)
//	{
//		NAV_Serial_Hook(Rx_Message);
//	}
	if(Serialx_Ctrl == &DR16_SERIAL)
	{
		xQueueSend(DR16_Rx_Queue, &Rx_Message, 0);
	}
}

void Message_Ctrl::CAN1_Process(CanRxMsg *Rx_Message)
{
	CanRxMsg Rx_Data;
	uint8_t dect=0;
  uint8_t Len=0;
  while(dect!=0xA5)
  {
			dect=CAN1_Ctrl.read();
			Len=CAN1_Ctrl.available();
			if(Len <= 12)
			return;
  }
  Len=CAN1_Ctrl.available();
  if(Len <= 12)
    return;
  dect=CAN1_Ctrl.read();
  if(dect!=0xA6)
    return;
  if(Len <= 11)
    return;
	for(uint8_t x=0;x<4;x++)
	{
					Rx_Data.StdId.u8[x]=CAN1_Ctrl.read();
	}
	for(uint8_t x=0;x<8;x++)
	{
					Rx_Data.Data[x]=CAN1_Ctrl.read();
	}
	switch(Rx_Data.StdId.u32)
	{

//		case CAN_DJI_Motor1_ID:
//		case CAN_DJI_Motor2_ID:
//		case CAN_DJI_Motor3_ID:

		#ifdef useHero
		case CAN_DJI_Motor3_ID:
		#endif
//	{
//		static uint8_t i = 0;
//		//处理电机ID号
//		i = Rx_Data.StdId.u32 - CAN_DJI_Motor6_ID;
//		//处理电机数据宏函数o
//		MA_get_motor_measure(CAN_Cmd.Gimbal.GetData(i), Rx_Data.Data);
//		break;
//	}
			case CAN_DJI_Motor7_ID:				
		{
			static uint8_t i = 0;
			//处理电机ID号
			i = Rx_Data.StdId.u32 - CAN_DJI_Motor5_ID;
			//处理电机数据宏函数
			MA_get_motor_measure(CAN_Cmd.Gimbal.GetData(i), Rx_Data.Data);
			break;
		}
	
			case DM_GimbalR_ID:
		{
				Get_DM_Motor_Measure(CAN_Cmd.Gimbal_DM_Yaw.GetData(),Rx_Data.Data,DM_6220_P_MIN,DM_6220_P_MAX,DM_6220_V_MIN,DM_6220_V_MAX,DM_6220_T_MIN,DM_6220_T_MAX);
				
				break;
		}
			case DM_GimbalR_Pitch_ID:
		{
				Get_DM_Motor_Measure(CAN_Cmd.Gimbal_DM_Pitch.GetData(),Rx_Data.Data,DM_4310_P_MIN,DM_4310_P_MAX,DM_4310_V_MIN,DM_4310_V_MAX,DM_4310_T_MIN,DM_4310_T_MAX);
				
				break;
		}


		default:
		{
			break;
		}
	}
}

void Message_Ctrl::CAN2_Process(CanRxMsg *Rx_Message)
{
	CanRxMsg Rx_Data;
	uint8_t dect=0;
  uint8_t Len=0;
  while(dect!=0xA5)
  {
				dect=CAN2_Ctrl.read();
				Len=CAN2_Ctrl.available();
				if(Len <= 12)
				return;
  }
  Len=CAN2_Ctrl.available();
  if(Len <= 12)
    return;
  dect=CAN2_Ctrl.read();
  if(dect!=0xA6)
    return;
  if(Len <= 11)
    return;
	for(uint8_t x=0;x<4;x++)
	{
					Rx_Data.StdId.u8[x]=CAN2_Ctrl.read();
	}
	for(uint8_t x=0;x<8;x++)
	{
					Rx_Data.Data[x]=CAN2_Ctrl.read();
	}
	switch(Rx_Data.StdId.u32)
	{
			case CAN_DJI_Motor1_ID:
			case CAN_DJI_Motor2_ID:
			case CAN_DJI_Motor3_ID:
			case CAN_DJI_Motor4_ID://四个麦轮
		{
			static uint8_t i = 0;
			//处理电机ID号
			i = Rx_Data.StdId.u32 - CAN_DJI_Motor1_ID;
			//处理电机数据宏函数
			MA_get_motor_measure(CAN_Cmd.Chassis.GetData(i), Rx_Data.Data);
			break;
		}

		//case CAN_DJI_Motor5_ID:
		case CAN_DJI_Motor7_ID://yaw和拨弹盘
		{
			static uint8_t i = 0;
			//处理电机ID号
			i = Rx_Data.StdId.u32 - CAN_DJI_Motor5_ID;
			//处理电机数据宏函数
			MA_get_motor_measure(CAN_Cmd.Gimbal.GetData(i), Rx_Data.Data);
			break;
		}
		case CAN_CAP_GET_ID://超级电容
		{
			SuperCapR.situation = (uint8_t)(Rx_Data.Data[0]);
			SuperCapR.mode = (uint8_t)(Rx_Data.Data[1]);
			SuperCapR.power = (float)((uint16_t)((Rx_Data.Data[2]) | (Rx_Data.Data[3]) << 8)) * 0.1f;
			SuperCapR.energy = (uint8_t)(Rx_Data.Data[4]);
			SuperCapR.power_limit = (uint8_t)(Rx_Data.Data[5]);
			SuperCapR.errorcode = (uint8_t)(Rx_Data.Data[6]);
		//SuperCapR.enable = (uint8_t)((Rx_Message)->Data.Data[7]);
		//SuperCapR.enable = (uint8_t)((Rx_Message)->Data.Data[8]);
			Guard.Feed(SupercapData);
			break;
		}

					case DM_GimbalR_ID:
		{
				Get_DM_Motor_Measure(CAN_Cmd.Gimbal_DM_Yaw.GetData(),Rx_Data.Data,DM_6220_P_MIN,DM_6220_P_MAX,DM_6220_V_MIN,DM_6220_V_MAX,DM_6220_T_MIN,DM_6220_T_MAX);
				
				break;
		}
		
		default:
		{
//			Gyro_CAN_Hook(&Rx_Data.StdId.u32 , Rx_Data.Data);
			break;
		}
	}
}
void Message_Ctrl::CAN3_Process(CanRxMsg *Rx_Message)
{
	CanRxMsg Rx_Data;
	uint8_t dect=0;
  uint8_t Len=0;
  while(dect!=0xA5)
  {
			dect=CAN3_Ctrl.read();
			Len=CAN3_Ctrl.available();
			if(Len <= 12)
			return;
  }
  Len=CAN3_Ctrl.available();
  if(Len <= 12)
    return;
  dect=CAN3_Ctrl.read();
  if(dect!=0xA6)
    return;
  if(Len <= 11)
    return;
	
	for(uint8_t x=0;x<4;x++)
	{
					Rx_Data.StdId.u8[x]=CAN3_Ctrl.read();
	}
	for(uint8_t x=0;x<8;x++)
	{
					Rx_Data.Data[x]=CAN3_Ctrl.read();
	}
	
	
	
	
	switch(Rx_Data.StdId.u32)
	{
						/*调试临时启用*/
		case CAN_DJI_Motor5_ID:
				{
			static uint8_t i = 0;
			//处理电机ID号
			i = Rx_Data.StdId.u32 - CAN_DJI_Motor5_ID;
			//处理电机数据宏函数
			MA_get_motor_measure(CAN_Cmd.Chassis.GetData(i), Rx_Data.Data);
			break;
		}
//		 case CAN_MPU_TO_Z:    
//        {
//            CAN_MPU_R_Z.AngleZ = ((int16_t)((Rx_Data.Data[0])<< 8|(Rx_Data.Data[1])))*FP32_MPU_RAD;
//            CAN_MPU_R_Z.Speed_Z= ((int16_t)((Rx_Data.Data[2])<< 8|(Rx_Data.Data[3])))*BMI088_GYRO_2000_SEN;
//            CAN_MPU_R_Z.Acce_Z = ((int16_t)((Rx_Data.Data[4])<< 8|(Rx_Data.Data[5])))*BMI088_ACCEL_6G_SEN;
//            CAN_MPU_R_Z.Acce_Y = ((int16_t)((Rx_Data.Data[6])<< 8|(Rx_Data.Data[7])))*BMI088_ACCEL_6G_SEN;
//						Gyro.Error_angle = (CAN_MPU_R_Z.AngleZ - Gyro.Last_angle);
//						if(Gyro.Error_angle > 270.0f) Gyro.Yaw_cycle--;
//						if(Gyro.Error_angle < -270.0f) Gyro.Yaw_cycle++;
//						Gyro.Yaw_real_angle=CAN_MPU_R_Z.AngleZ;
//						Gyro.Yaw_angle = CAN_MPU_R_Z.AngleZ+ Gyro.Yaw_cycle * 360.0f;
//						Gyro.Yaw_speed=CAN_MPU_R_Z.Speed_Z;
//						Gyro.Last_angle=CAN_MPU_R_Z.AngleZ;
//            break;
//        }
//     case CAN_MPU_TO_X_Y:
//        {
//            CAN_MPU_R_XY.AngleX =((int16_t)((Rx_Data.Data[0])<< 8|(Rx_Data.Data[1])))*FP32_MPU_RAD;
//            CAN_MPU_R_XY.Speed_X=((int16_t)((Rx_Data.Data[2])<< 8|(Rx_Data.Data[3])))*BMI088_GYRO_2000_SEN;
//            CAN_MPU_R_XY.AngleY =((int16_t)((Rx_Data.Data[4])<< 8|(Rx_Data.Data[5])))*FP32_MPU_RAD;
//            CAN_MPU_R_XY.Speed_Y=((int16_t)((Rx_Data.Data[6])<< 8|(Rx_Data.Data[7])))*BMI088_GYRO_2000_SEN;
//						Gyro.Pitch_angle= CAN_MPU_R_XY.AngleY ;
//						Gyro.Pitch_speed= CAN_MPU_R_XY.Speed_Y;
//            break;

//        }
case 0x315 :
        {
            Gimbal_IMU_data.Pitch_Angle.uint_8[0] = Rx_Data.Data[0];
            Gimbal_IMU_data.Pitch_Angle.uint_8[1] = Rx_Data.Data[1];
            Gimbal_IMU_data.Pitch_Gyro.uint_8[0] =  Rx_Data.Data[2];
            Gimbal_IMU_data.Pitch_Gyro.uint_8[1] =  Rx_Data.Data[3];
            Gimbal_IMU_data.Yaw_Angle.uint_8[0] =   Rx_Data.Data[4];
            Gimbal_IMU_data.Yaw_Angle.uint_8[1] =   Rx_Data.Data[5];
            Gimbal_IMU_data.Yaw_Gyro.uint_8[0] =    Rx_Data.Data[6];
            Gimbal_IMU_data.Yaw_Gyro.uint_8[1] =    Rx_Data.Data[7];
            
            Gyro_data.Error_angle = Gimbal_IMU_data.Yaw_Angle.int_16 * FP32_MPU_RAD - Gyro_data.Last_YAW_angle;
            if(Gyro_data.Error_angle > 270.0f)
            { 
                  Gyro_data.Yaw_cycle--;
            }
            if(Gyro_data.Error_angle < -270.0f)
            {
                Gyro_data.Yaw_cycle++;
            }
         
            Gyro_data.Yaw_angle = Gimbal_IMU_data.Yaw_Angle.int_16 * FP32_MPU_RAD + Gyro_data.Yaw_cycle * 360.0f;
              Gyro_data.Yaw_speed = Gimbal_IMU_data.Yaw_Gyro.int_16 * BMI088_GYRO_2000_SEN;
          
            
            Gyro_data.Last_YAW_angle = Gimbal_IMU_data.Yaw_Angle.int_16 * FP32_MPU_RAD;
            
        }
		
		
		
		

		case CAN_DJI_Motor1_ID:
		case CAN_DJI_Motor2_ID:
	{
		static uint8_t i = 0;
		//处理电机ID号
		i = Rx_Data.StdId.u32 - CAN_DJI_Motor1_ID;
		//处理电机数据宏函数
		MA_get_motor_measure(CAN_Cmd.Fric.GetData(i), Rx_Data.Data);
		break;
	}

//	case 0x401:
//	{
//			Gyro.DM_data.DM_Picth =((int16_t) ((Rx_Data.Data[0])<< 8 | (Rx_Data.Data[1]) ) ) / 100.0f;
//			Gyro.DM_data.DM_Roll  =((int16_t) ((Rx_Data.Data[2])<< 8  | (Rx_Data.Data[3]) ) )/ 100.0f;
//			Gyro.DM_data.DM_Yaw   =((int16_t) ((Rx_Data.Data[4])<< 8  | (Rx_Data.Data[5]) ) )/ 100.0f;
//			Gyro.DM_data.DM_SpeedZ=((int16_t) ((Rx_Data.Data[6])<< 8  | (Rx_Data.Data[7]) ) )/ 100.0f;
//		
//		Gyro.DM_data.DM_Error_angle = Gyro.DM_data.DM_Yaw - Gyro.DM_data.Last_YAW_angle;
//	   if(Gyro.DM_data.DM_Error_angle > 270.0f) 
//		  Gyro.DM_data.DM_Yaw_cycle--;
//	   if(Gyro.DM_data.DM_Error_angle < -270.0f)
//		  Gyro.DM_data.DM_Yaw_cycle++;
//	  Gyro.DM_data.DM_Yaw_angle = -Gyro.DM_data.DM_Yaw  - Gyro.DM_data.DM_Yaw_cycle * 360.0f;
//	  Gyro.DM_data.DM_Yaw_speed = -Gyro.DM_data.DM_SpeedZ;

//	  Gyro.DM_data.Last_YAW_angle =Gyro.DM_data.DM_Yaw;
//	
//		 break;
//	}

#ifdef useSteering
	case CAN_DJI_Motor5_ID:
	case CAN_DJI_Motor6_ID:
	case CAN_DJI_Motor7_ID:
	case CAN_DJI_Motor8_ID:
	{
		static uint8_t i = 0;
		//处理电机ID号
		i = Rx_Data.StdId.u32 - CAN_DJI_Motor5_ID;
		//处理电机数据宏函数
		MA_get_motor_measure(CAN_Cmd.Steer.GetData(i), Rx_Data.Data);
		break;
	}
#endif
	default:
	{
//		Gyro_CAN_Hook(&Rx_Data.StdId.u32 , Rx_Data.Data);
//		break;
	}
	}
}

//void Message_Ctrl::NAV_Serial_Hook(uint8_t *Rx_Message)
//{
//	
//	uint8_t len = Rx_Message[0];
//	
//	for(uint8_t i=0 ;i<4;i++)
//	{
//		linear_x.I [i]=Rx_Message[2+i];
//		linear_y.I [i]=Rx_Message[6+i];
//		angular_z.I[i]=Rx_Message[10+i];
//		
//	
//	}	
//	NAV.Top_state =Rx_Message[14];
//	NAV.linear_x =linear_x.F;
//	NAV.linear_y =linear_y.F;
//	NAV.angular_z=angular_z.F;
//}

void Message_Ctrl::FricSpeed_pid()
{
		uint8_t Bullet_Speed_Set;
//			if(Message.robo->shoot_data.initial_speed != bullet_speed_last)
//			{
//				bullet_speed_last = Message.robo->shoot_data.initial_speed;
//				#ifdef useHero
//				if(Message.robo->game_robot_state.shooter_barrel_heat_limit != 0)
//				{
//					Bullet_Speed_Set = Message.robo->game_robot_state.shooter_barrel_heat_limit;
//				}
//				#endif
//				#ifdef useInfantry
//				if(Message.robo->power_heat_data.shooter_17mm_1_barrel_heat != 0)
//				{
//					Bullet_Speed_Set = Message.robo->game_robot_state.shooter_id1_17mm_speed_limit;
//				}
//				#endif
//				else
//				{
//					Bullet_Speed_Set = Gimbal.Data.Fric_Gear[Gimbal.Data.Gear - 1];
//				}
//				//待改成pid,无固定周期,没想法
//				if(Message.robo->shoot_data.initial_speed - Bullet_Speed_Set > -(Bullet_Speed_Set*0.05f))
//				{
//					Gimbal.Data.Fric_Set[Gimbal.Data.Gear - 1] -= 40;
//				}
//				else if(Message.robo->shoot_data.initial_speed - Bullet_Speed_Set > -(Bullet_Speed_Set*0.08f))
//				{
//					Gimbal.Data.Fric_Set[Gimbal.Data.Gear - 1] -= 20;
//				}
//				else if(Message.robo->shoot_data.initial_speed - Bullet_Speed_Set < -(Bullet_Speed_Set*0.15f))
//				{
//					Gimbal.Data.Fric_Set[Gimbal.Data.Gear - 1] += 80;
//				}
//				else if(Message.robo->shoot_data.initial_speed - Bullet_Speed_Set < -(Bullet_Speed_Set*0.08f))
//				{
//					Gimbal.Data.Fric_Set[Gimbal.Data.Gear - 1] += 20;
//				}
//			}
}

void Message_Ctrl::Visual_Serial_Hook(uint8_t *Rx_Message)
{
uint8_t len = Rx_Message[0];

//	for(uint8_t i=0 ;i<4;i++)
//	{
//		pitch.I [i]=Rx_Message[3+i];
//		yaw.I [i]=Rx_Message[7+i];
//		distance.I[i]=Rx_Message[11+i];
//		
//	
//	}
//	
//	
//	VisualR.fire =Rx_Message[2];
//	VisualR.pitch =pitch.F;
//	VisualR.yaw =yaw.F;
//	VisualR.check_byte =Rx_Message[14];
//	VisualR.distance=distance.F;
	
	//视觉包为32字节，其中第倒数第二个字节为辨识：0xA5视觉包   0x5A导航包
	leneeee=len;
	if(Rx_Message[1]==0xA5)
	{
			for(uint8_t i=0 ;i<4;i++)
		{	
				pitch.I [i]=Rx_Message[3+i];
				yaw.I [i]=Rx_Message[7+i];
				distance.I[i]=Rx_Message[11+i];
		
		
		}
	
	
			VisualR.fire =Rx_Message[2];
			VisualR.pitch =pitch.F;
			VisualR.yaw =yaw.F;
			VisualR.distance=distance.F;
	
		for(uint8_t i=0 ;i<4;i++)
		{
			linear_x.I [i]=Rx_Message[16+i];
			linear_y.I [i]=Rx_Message[20+i];
			angular_z.I[i]=Rx_Message[24+i];
		  
		}
		NAV.Top_state =Rx_Message[15];
		NAV.Follow_Gimbal_state = Rx_Message[28];
		NAV.linear_x =linear_x.F;
		NAV.linear_y =linear_y.F;
		NAV.angular_z=angular_z.F;
	}
	
//if(Rx_Message[15]==2)
//{	
//	
//	for(uint8_t i=0 ;i<4;i++)
//	{
//		pitch.I [i]=Rx_Message[3+i];
//		yaw.I [i]=Rx_Message[7+i];
//		distance.I[i]=Rx_Message[11+i];
//		
//	
//	}
//	
//	
//	VisualR.fire =Rx_Message[2];
//	VisualR.pitch =pitch.F;
//	VisualR.yaw =yaw.F;
//	
//	VisualR.distance=distance.F;
//}

//	
//	if(Rx_Message[15]==1)
//	{	
//		for(uint8_t i=0 ;i<4;i++)
//		{
//			linear_x.I [i]=Rx_Message[3+i];
//			linear_y.I [i]=Rx_Message[7+i];
//			angular_z.I[i]=Rx_Message[11+i];
//			
//		
//		}
//		NAV.Top_state =Rx_Message[2];
//		NAV.linear_x =linear_x.F;
//		NAV.linear_y =linear_y.F;
//		NAV.angular_z=angular_z.F;
//	}
for(uint8_t i=1 ;i<17;i++)
{
	arrrr[i]=Rx_Message[i];
}

}

void Message_Ctrl::Gyro_CAN_Hook(uint32_t *Rx_Message ,uint8_t *Rx_Data)
{


}

RC_ctrl_t *get_remote_control_point(void)
{
    return &RC_ctrl;
}

Message_Ctrl *get_message_ctrl_pointer(void)
{
	return &Message;
}





float Message_Ctrl::Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if(time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2; //计算速度

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300 /*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0; //时间过长则认为速度不变
	}

	return S->processed_speed; //计算出的速度
}

//统计按键 按下次数：eg:  按下-松开  按下-松开  2次
//key_num==1代表有键盘按下
//key_num==0代表键盘松开
void rc_key_c::sum_key_count(int16_t key_num, count_num_key *temp_count)
{
	if(key_num == 1 && temp_count->key_flag == 0)
	{
		temp_count->key_flag = 1;
	}
	if(temp_count->key_flag == 1 && key_num == 0)
	{
		temp_count->count++;
		temp_count->key_flag = 0;
	}
}

void rc_key_c::clear_key_count(count_num_key *temp_count)
{
	temp_count->count = 0;
	temp_count->key_flag = 0;
}
//按键单点赋值
bool rc_key_c::read_key_single(count_num_key *temp_count, bool *temp_bool)
{
	if((temp_count->count >= 1) && *temp_bool == 0)
	{
		temp_count->count = 0;
		*temp_bool = true;
	}
	else if((temp_count->count >= 1) && *temp_bool == 1)
	{
		temp_count->count = 0;
		*temp_bool = false;
	}
	return *temp_bool;
}
//按键单点
bool rc_key_c::read_key_single(count_num_key *temp_count)
{
	if(temp_count->count >= 1)
	{
		temp_count->count = 0;
		return true;
	}
	else
	{
		temp_count->count = 0;
		return false;
	}
}
//按键长按赋值q
bool rc_key_c::read_key_even(count_num_key *temp_count, bool *temp_bool)
{
	if(temp_count->key_flag == 1)
	{
		*temp_bool = true;
	}
	else if(temp_count->key_flag == 0)
	{
		*temp_bool = false;
	}
	return *temp_bool;
}
//按键长按
bool rc_key_c::read_key_even(count_num_key *temp_count)
{
	if(temp_count->key_flag == 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

uint8_t rc_key_c::read_key(count_num_key *temp_count, key_count_e mode, bool clear)
{
	uint8_t result;
	if(clear == true)
	{
		if(mode == single)
		{
			result = read_key_single(temp_count);
		}
		else if(mode == even)
		{
			result = read_key_even(temp_count);
		}
	}
	else
	{
		if(mode == single)
		{
			result = temp_count->count;
		}
		else if(mode == even)
		{
			result = temp_count->key_flag;
		}
	}
	return result;
}

bool rc_key_c::read_key(count_num_key *temp_count, key_count_e mode, bool *temp_bool)
{
	if(mode == single)
	{
		read_key_single(temp_count, temp_bool);
	}
	else if(mode == even)
	{
		read_key_even(temp_count, temp_bool);
	}
	return *temp_bool;
}

//更新按键
void rc_key_c::rc_key_v_set(RC_ctrl_t *RC)
{
	count_num_key *p = &Key.W;
	for(uint8_t i = 0; i < 16; i++)
	{
		if(RC->key.v & ((uint16_t)1 << i))
		{
			sum_key_count(1, (p + i));
		}
		else
		{
			sum_key_count(0, (p + i));
		}
	}
	//鼠标
	if(RC->mouse.press_l == 1)
	{
		sum_key_count(1, &Press.L);
	}
	else
	{
		sum_key_count(0, &Press.L);
	}
	if(RC->mouse.press_r == 1)
	{
		sum_key_count(1, &Press.R);
	}
	else
	{
		sum_key_count(0, &Press.R);
	}
}



















