#include "dev_serial.h"
#include "app_preference.h"
#include "protocol_judgement.h"
#include "protocol_crc.h"
#include <stdio.h>
#include "string.h"
#include "arm_math.h"

//裁判系统串口

//static tFrame			                  tframe;
static tMsg_head    judgedatahead;
//static ext_client_custom_graphic_single_t  ext_client_custom_graphic_single;
//static draw_data_struct_t		draw_data_struct;
//static judge_type_t                   judge_type;
extern void client_info_update(void);
//extern void Rfid_status(ext_rfid_status_t rfid_status,Rfid_Status * RF);
judge_type_t judge_type;
//Rfid_Status  RFID_Status; 

int level_group[10]={1,2,3,4,5,6,7,8,9,10};
int HP_group[10]={150,175,200,225,250,275,300,325,350,400};
int power_group1[10]={60,65,70,75,80,85,90,95,100,100};
int power_group2[10]={45,50,55,60,65,70,75,80,90,100};



//裁判系统相关
void referee_data_solve(uint8_t *Rx_Message)
{
	static uint16_t start_pos = 0, next_start_pos = 0;
	while (1)
	{
		memcpy(&judgedatahead.SOF, &Rx_Message[start_pos], FrameHeader_Len);
		/*先校验头帧0xA5 然后crc8校验帧头 再crc16位校验整包*/
		if ((judgedatahead.SOF == (uint16_t)JudgeFrameHeader) \
			&& (1 == Verify_CRC8_Check_Sum(&Rx_Message[start_pos], FrameHeader_Len)) \
			&& (1 == Verify_CRC16_Check_Sum(&Rx_Message[start_pos], judgedatahead.DataLength + FrameHeader_Len + 4)))//数据位长度+帧头长度+命令码长度+校验码长度
		{
			memcpy(&judge_type.rxCmdId, (&Rx_Message[start_pos] + 5), sizeof(judge_type.rxCmdId));
			Rx_Message[start_pos]++;//每处理完一次就在帧头加一防止再次处理这帧数据
			next_start_pos = start_pos + 9 + judgedatahead.DataLength;//9为 5位帧头 2位数据长度 2校验位
			switch (judge_type.rxCmdId)
			{
				case CmdID_0x0001:
				{
					memcpy(&judge_type.game_status, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
					break;
				}
				case CmdID_0x0003:
				{
					memcpy(&judge_type.game_robot_HP, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
					break;
				}
				case CmdID_0x0201://机器人状态数据，10Hz发送；
				{
					memcpy(&judge_type.game_robot_state, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
					break;
				}
				case CmdID_0x0202://实时功率热量数据，10Hz发送；
				{
					memcpy(&judge_type.power_heat_data, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
					break;
				}
				case CmdID_0x0203://读取机器人位置信息
				{
					memcpy(&judge_type.game_robot_pos, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
					break;
				}	
//				case CmdID_0x0204://机器人增益和底盘能量数据，固定以 3Hz 频率发送
//				{
//					memcpy(&judge_type.buff_t, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
//					break;
//				}  
				case CmdID_0x0207://实时射击数据，弹丸发射后发送；
				{
					memcpy(&judge_type.shoot_data, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
					break;
				}
				case CmdID_0x0208://实时射击数据，弹丸发射后发送；
				{
					memcpy(&judge_type.projectile, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
					break;
				}
//				case CmdID_0x0209://实时射击数据，弹丸发射后发送；
//				{
//					memcpy(&judge_type.rfid_status_t, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
//					break;
//				}
				case CmdID_0x0301:
				{
					memcpy(&judge_type.userinfo, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
					break;
				}
			default:{
					break;
				}
			}

//			Rfid_status(judge_type.rfid_status_t,&RFID_Status);
			start_pos = next_start_pos;
		}
		else
		{
			start_pos = 0;
			break;
		}
		/**如果头指针越界了退出循环**/
		if (start_pos > JUDGERX_BUF_NUM)
		{
			start_pos = 0;
			break;
		}
//		for(int i=0;i<10;i++)
//		{
//			if(level_group[i]==judge_type.game_robot_state.robot_level)
//			{
//				if(judge_type.game_robot_state.maximum_HP==HP_group[i])
//				{
//					judge_type.game_robot_state.chassis_power_limit = power_group1[i];
//					break;
//				}
//				else
//				{
//					judge_type.game_robot_state.chassis_power_limit = power_group2[i];
//					break;
//				}
//			}
//		}	
	}
}

const judge_type_t *get_robo_data_Point(void)
{
	return &judge_type;
}

void Usart_SendBuff(uint8_t *buf, uint16_t len)
{
	if (len > 512)
	{
		return;
	}
	JUDGE_SERIAL.sendData(buf, len);
}
//void Rfid_status(ext_rfid_status_t rfid_status,Rfid_Status * RF)
//{
//	int i =0;
//	for(i=0;i<=23;i++){
//		RF->Rfid_Status[i]=rfid_status.rfid_status & int(pow(2.0f,i));
//	}
//}
