#include "Correspond_Task.h"
#include "algorithm_SolveTrajectory.h"
#include "chassis_power_control.h"
#include "protocol_judgement.h"
#include "app_preference.h"
#include "tasks.h"
#include "Robot_Task.h"

#include "arm_math.h"
#include <vector>
float last_shoot_speed;
sentry_cmd_t sentry_cmd_struct;
union F Gimbal_Union;
/*发送数据*/
float ch[15];
float set_bo[25];
Correspondence_ctrl Corres;
extern float lll;
uint16_t X=10,Y=0;



void Correspond_Task(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	Corres.Corres_Init();
  /* Infinite loop */
  for(;;)
  {		
		set_bo[1] = Message.robo->shoot_data.launching_frequency;
		//set_bo[2] = Gimbal.DM_Yaw.angle_set;
		//set_bo[3] = Message.VisualR.yaw;
		//set_bo[4] = Gimbal.DM_Yaw.VisualR_LuBo_yaw;
		//set_bo[5] = Gimbal.DM_Yaw.speed;
		//set_bo[6] = Gimbal.DM_Yaw.PositinPid.out;
		//set_bo[7] = Gimbal.DM_Yaw.feedforward;
		
		//set_bo[4] = Gimbal.DM_Yaw.Visual_SpeedPid.out;
		//set_bo[6] = Gimbal.DM_Yaw.VisualR_LuBo_yaw;
		//set_bo[8] = Gimbal.DM_Yaw.tor_set;
	
		
	/*	
  set_bo[5] = Gimbal.DM_Yaw.PositinPid.P_out;
  set_bo[2]=Gimbal.DM_Yaw.PositinPid.out;
  set_bo[3]= Gimbal.DM_Yaw.feedforward;
  set_bo[4]= Gimbal.DM_Yaw.SpeedPid.out;
 */
    vofa_justfloat_output(set_bo,2,&Serial3_Ctrl);
		
		
		Corres.Corres_Feedback();
		Corres.Corres_Send();
		
		xQueueSend(Message_Queue, &ID_Data[CorrespondenceData], 0);
    osDelay(Correspondence_Task_Control_Time);
  }
  /* USER CODE END StartDefaultTask */
}

void Correspondence_ctrl::Corres_Init(void)
{
	WS2812_INIT();
	visual.header=0xff;
	visual.mode=0;
	visual.check_byte=1;
	visual.pitch=Gimbal.DM_Pitch.angle;
	visual.roll=0;
	visual.shoot_speed = Message.robo->shoot_data.initial_speed;
	visual.yaw= Message.Gimbal_IMU_data.Yaw_Angle.int_16 * FP32_MPU_RAD;
	visual.game_sate = Message.robo ->game_status.game_progress; //Chassis.Flags.game_sate_Flag;
	visual.projectile = Message.robo->projectile.projectile_allowance_17mm;
	visual.robot_HP = 400;
	last_HP = 400;
	visual.Tail=0x0d;
	visual.outpost_HP = 1000;
	Data.Shoot_Unoin.F = 14.0f;
	
	
	
	


}
void Correspondence_ctrl::Corres_Send(void)
{
	uint8_t j;
	
	j=sizeof(Visual_New_Posture_Data_t);
	
	if(Rate_Do_Execute(5))
	{//10MS一次
		VISUAL_SERIAL.sendData(&visual.header , sizeof(Visual_Send_Data_t));		
	} 
	JUDGE_SERIAL.sendData(&sentry_cmd_struct.Sentry_Msg_head , sizeof(sentry_cmd_struct));
	
	
	
	
}

void Correspondence_ctrl::Corres_Feedback(void)
{
	
	switch(Message.robo->game_robot_state.robot_id)
	{  
		case 7:   //id:7  红方哨兵
			visual.mode = 1;
		  visual.robot_HP = Message.robo->game_robot_HP.red_7_robot_HP;
		  visual.outpost_HP = Message.robo->game_robot_HP.blue_outpost_HP;
			break;
		case 107: //id:107  蓝方哨兵
			visual.mode = 0;
		 visual.robot_HP = Message.robo->game_robot_HP.blue_7_robot_HP;
		 visual.outpost_HP = Message.robo->game_robot_HP.red_outpost_HP;
			break;
		default:
			break;
	}
	//数据更新
	
	
	
	sentry_cmd_struct.Sentry_Msg_head .SOF =0xA5;
		sentry_cmd_struct.Sentry_Msg_head .DataLength=10;
		sentry_cmd_struct.Sentry_Msg_head.Seq +=1;
    Append_CRC8_Check_Sum((uint8_t *)&sentry_cmd_struct.Sentry_Msg_head ,sizeof(sentry_cmd_struct.Sentry_Msg_head));		
		sentry_cmd_struct.CmdID=0x0301;
		
		sentry_cmd_struct.Sentry_header_id.sender_ID=Message.robo->game_robot_state.robot_id;
		sentry_cmd_struct.Sentry_header_id.receiver_ID=0x8080;
		sentry_cmd_struct.Sentry_header_id.data_cmd_id =0x0120;
	  sentry_cmd_struct.sentry_cmd=1;
	Append_CRC16_Check_Sum((uint8_t *)&sentry_cmd_struct ,sizeof(sentry_cmd_struct));
	
	  /*----*/
//	  if(Message.robo->projectile.projectile_allowance_17mm == 0)
//	  {
//		 sentry_cmd_struct.sentry_cmd=  ((X+Y)<< 2 | 1);
//		 Y++;
//		}
//		else
//		{
//			sentry_cmd_struct.sentry_cmd=1;
//			
//		}
		/*----*/
		
		
	
	

	
	
 if(Message.robo->game_robot_state.power_management_chassis_output == 0 || Message.robo->power_heat_data.chassis_power < 1
		|| Message.robo->game_robot_state.power_management_gimbal_output == 0)
	{
		SuperCapS.enable = 0x00;
	}
 else if((Message.SuperCapR.situation == CAP_CLOSE || Message.SuperCapR.situation == CAP_OPEN) && Message.robo->game_robot_state.robot_level >= 1)
	{
		SuperCapS.enable = 0xff;
	}
	else
	{
		SuperCapS.enable = 0x00;
	}
	if(Chassis.Mode==CHASSIS_NO_MOVE)
	{
		SuperCapS.enable = 0x00;
	}

	SuperCapS.mode = 0xFF;
	
	SuperCapS.power = (uint8_t)Message.robo->power_heat_data.chassis_power;
	SuperCapS.power_limit = (uint8_t)Message.robo->game_robot_state.chassis_power_limit;
	
	visual.game_sate = Message.robo ->game_status.game_progress;
	visual.projectile = Message.robo->projectile.projectile_allowance_17mm;
  
	last_shoot_speed = Message.robo->shoot_data.initial_speed;
	if(Message.robo->shoot_data.initial_speed == 0)
	{
		visual.shoot_speed = last_shoot_speed;
	}
	else
	{		
		visual.shoot_speed = Message.robo->shoot_data.initial_speed;
	}
	
	
	
	
	
	Statistic_Update(xTaskGetTickCount());
}
