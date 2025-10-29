#ifndef Message_TASK_H
#define Message_TASK_H

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "app_preference.h"
#include "dev_can.h"
#include "app_serial.h"
#include "protocol_dbus.h"
#include "protocol_judgement.h"
#include "drivers_statistic.h"
#include "app_motor.h"


#ifdef __cplusplus
extern "C" {
#endif

	void Message_Task(void *pvParameters);
	void CAN1_Rx_Task(void *pvParameters);
	void CAN2_Rx_Task(void *pvParameters);
	void CAN3_Rx_Task(void *pvParameters);
	void Serial_Rx_Task(void *pvParameters);
	void Referee_Rx_Task(void *pvParameters);
	void DR16_Rx_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif


extern QueueHandle_t Message_Queue;   		//消息队列句柄
extern QueueHandle_t Message_Queue;
extern QueueHandle_t CAN1_Rx_Queue;
extern QueueHandle_t CAN2_Rx_Queue;
extern QueueHandle_t CAN3_Rx_Queue;
extern QueueHandle_t Serial_Rx_Queue;
extern QueueHandle_t Referee_Rx_Queue;
extern QueueHandle_t DR16_Rx_Queue;

#define CAP_CHECK 0x00//检查状态
#define CAP_CLOSE 0x0f//关闭状态
#define CAP_OPEN 0xf0//开启状态
#define CAP_ERROR 0xff//错误状态
#define CAP_MODE1 0x00//关闭补偿
#define CAP_MODE2 0xff//开启补偿

#define VISUAL_TARGET_MAX 200


struct Sentry_msg_cmd_t
{
	uint32_t Rwc_flag:1;//读秒完成是否复活标志位bit0
	uint32_t Buyback_flag:1;//买活标志位bit1
  uint32_t efp_flag:11;//非远程兑弹bit2-bit12
	uint32_t rep_flag:4;//远程兑弹bit13-bit16
	uint32_t Rbpr_flag:4;//远程兑换血包bit17-bit20
	uint32_t buff:12;	//剩下十二位数据保留
};

union F
{
	uint8_t I[4];
	fp32 F;
};

union int16_t_uint8_t
{
    uint8_t uint_8[2];
    int16_t int_16;
};

union Sentry_u32
{
	Sentry_msg_cmd_t Sentry_Cmd;
  uint32_t Sentry_cmd_data;
};



#define sampleFreq	512.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//卡尔曼滤波
#define Yaw_Q_angle  0.1f		//角度数据置信度，角度噪声的协方差  0.001f
#define Yaw_Q_gyro   0.3f		//角速度数据置信度，角速度噪声的协方差  0.003f
#define Yaw_R_angle  0.05f			//加速度计测量噪声的协方差  0.5f
static float PP[2][2] = { { 1, 0 },{ 0, 1 } };//过程协方差矩阵P，初始值为单位阵

struct C_gyro_data_t
{
  int16_t Yaw_angle;
	int16_t  Yaw_speed;
};



struct gyro_receive_data_t
{
	fp32 AngleP;//pitch
	fp32 AngleR;//roll
	fp32 AngleY;//yaw

	int16_t SpeedX;
	int16_t SpeedY;
	int16_t SpeedZ;

	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;

	int16_t MagX;
	int16_t MagY;
	int16_t MagZ;
};




struct Damiao_motor_Data
{
   float Motor_pos;
   float Motor_vel;
	
};










struct gyro_calc_data_t
{
	float twoKp;
	float twoKi;
	float q0, q1, q2, q3;

	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, hz, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float integralFBx, integralFBy, integralFBz;
	gyro_calc_data_t(void)
	{
		q0 = 1.0f;
		twoKp = twoKpDef;
		twoKi = twoKiDef;
	}
};

typedef struct //视觉目标速度测量
{
	int delay_cnt; //计算相邻两帧目标不变持续时间,用来判断速度是否为0
	int freq;
	int last_time;         //上次受到目标角度的时间
	float last_position;   //上个目标角度
	float speed;           //速度
	float last_speed;      //上次速度
	float processed_speed; //速度计算结果
} speed_calc_data_t;

struct gyro_angle_data_t
{
	fp32 AngleP_Calc;
	fp32 AngleR_Calc;
	fp32 AngleY_Calc;

	fp32 AngleP_Speed;
	fp32 AngleR_Speed;
	fp32 AngleY_Speed;

	fp32 AngleP_Acc;
	fp32 AngleR_Acc;
	fp32 AngleY_Acc;

	fp32 AngleP_Mag;
	fp32 AngleR_Mag;
	fp32 AngleY_Mag;
};
struct DM_receive_data_t
{
	fp32 DM_Picth;
	fp32 DM_Roll;
	fp32 DM_Yaw;
	fp32 DM_SpeedZ;
	
	fp32 DM_Error_angle;
	fp32 DM_Yaw_cycle;
	fp32 Last_YAW_angle;
	fp32 DM_Yaw_angle;
	fp32 DM_Yaw_speed;
	
};
struct gyro_data_t
{
	gyro_receive_data_t data;
	gyro_calc_data_t calc;
	gyro_angle_data_t angle;
	
	DM_receive_data_t DM_data;
	


	fp32 Yaw_angle;
	fp32 Yaw_real_angle;
	fp32 Yaw_angle_offset;
	fp32 Yaw_speed;
	int32_t Yaw_cycle;
	fp32 Last_angle;
	fp32 Error_angle;
	fp32 Pitch_angle;
	fp32 Pitch_speed;

	uint32_t time[4];
	uint32_t last_time[4];
	uint32_t differ_time[4];

	Statistic Gyro_Acc_fps;
	Statistic Gyro_Speed_fps;
	Statistic Gyro_Mag_fps;
	Statistic Gyro_Angle_fps;
};

struct Visual_Receive_Data_t
{
		bool fire;
		fp32 pitch;
		fp32 yaw;
		fp32 distance;
		uint8_t check_byte;
		
};
struct NAV_Receive_Data_t
{
		
		fp32 linear_x;
		fp32 linear_y;
		fp32 angular_z;
		uint8_t Top_state;
	  uint8_t Follow_Gimbal_state;
	  uint16_t checksum;
	  
		
};


struct CAN_MPU_Data_R_Z
{
   float AngleZ;
   float Speed_Z;
   float Acce_Z;
   float Acce_Y;
};

struct CAN_MPU_Data_R_XY
{
   float AngleX;
   float Speed_X;

   float AngleY;
   float Speed_Y;    
};




//新can
struct Can_Gimbal_IMU_Data_t
{
    int16_t_uint8_t Pitch_Angle;
    int16_t_uint8_t Pitch_Gyro;
    int16_t_uint8_t Yaw_Angle;
    int16_t_uint8_t Yaw_Gyro;
    

};
struct Gyro_RX_data_t
{
    fp32 Picth;
    fp32 Roll;
    fp32 Yaw;
    fp32 SpeedZ;
    fp32 SpeedX;
    fp32 SpeedY;
    fp32 Acce_Z;
    fp32 Acce_Y;
    
    fp32 Error_angle;
    fp32 Yaw_cycle;
    fp32 Last_YAW_angle;
    fp32 Yaw_angle;
    fp32 Yaw_speed;
    fp32 Pitch_angle;
    fp32 Pitch_speed;    
    fp32 Roll_angle;
    fp32 Roll_speed;        
};


struct supercap_Receive_Data_t
{
    uint8_t situation;  //0x00自检 0x0f关闭 0xf0开启 0xff错误
    uint8_t mode;       //0x00模式1 0xff模式2
    float power;
    uint8_t energy;
    uint8_t power_limit;
    uint8_t errorcode;

    uint8_t *ptr;
};

//union int16_float
//{
//	float fdata;
//	int Idata;
//};

//struct Odem_Receive_Data_t
//{
//    Odem_Receive_Data_t():Header(0x5A)
//    {}
//    uint8_t Header;
//    float l_x;
//    float l_y;
//    float a_z;
//		uint8_t navigating;
//	  uint8_t CRC16_L;
//	  uint8_t CRC16_H;

//};

//typedef struct
//{
//	uint8_t ID;//ID[0]
//	uint8_t state;//ERR[0]
//	int16_float POS;//位置[1],[2]
//	int16_float VEl;//速度[3][4]
//	int16_float Torque;//扭矩信息[4][5]
//	float temperature_MOS;//驱动上 MOS 的平均温度[6]
//	float temperature_Rotro;//电机内部线圈的平均温度[7]
//}Torque_Motor_measure_t;
class Message_Ctrl:public Statistic
{
public: 
	
  union Sentry_u32 Sentry_Cmd_data;

	NAV_Receive_Data_t         NAV;
	Visual_Receive_Data_t      VisualR;
//	Torque_Motor_measure_t     Pitch_Motor_measure_t;
    Damiao_motor_Data Dm_Pitch;
	supercap_Receive_Data_t    SuperCapR;
  //Odem_Receive_Data_t OdemR;
//	CAN_MPU_Data_R_Z CAN_MPU_R_Z;
//	CAN_MPU_Data_R_XY CAN_MPU_R_XY;

	Can_Gimbal_IMU_Data_t Gimbal_IMU_data;
	Gyro_RX_data_t  Gyro_data;
	

	DM_Motor_measure_t    Gimbal_yaw;
	DM_Motor_measure_t		Gimbal_pitch;

	const judge_type_t *robo;

//	gyro_data_t Gyro;

	RC_ctrl_t *RC_Ptr;


	union F yaw;
	union F pitch;
	union F distance;
	
	union F linear_x;
	union F linear_y;
	union F angular_z;
	



	void Hook();
	void Init();
	//void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	//void ToEulerAngles(fp32 q1, fp32 q2, fp32 q3, fp32 q4);
	void CAN1_Process(CanRxMsg *Rx_Message);
	void CAN2_Process(CanRxMsg *Rx_Message);
	void CAN3_Process(CanRxMsg *Rx_Message);
	void Serialx_Hook(uint8_t *Rx_Message, Serialctrl *Serialx_Ctrl);
	void Visual_Serial_Hook(uint8_t *Rx_Message);
	void NAV_Serial_Hook(uint8_t *Rx_Message);
	void FricSpeed_pid();
	void Gyro_CAN_Hook(uint32_t *Rx_Message ,uint8_t *Rx_Data);
	float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);
private:
	speed_calc_data_t Visual_Yaw_Speed;
	speed_calc_data_t Visual_Pitch_Speed;

	fp32 bullet_speed_last;


};

Message_Ctrl *get_message_ctrl_pointer(void);
RC_ctrl_t *get_remote_control_point(void);

typedef struct
{
	uint8_t	key_flag;
	uint8_t  count;//次数
	uint8_t  last_count;
}count_num_key;

typedef enum
{
	single = 0,
	even,
	count,
}key_count_e;

struct rc_key_v_t
{
	//键盘
	count_num_key W;
	count_num_key S;
	count_num_key A;
	count_num_key D;
	count_num_key shift;
	count_num_key ctrl;
	count_num_key Q;
	count_num_key E;
	count_num_key R;
	count_num_key F;
	count_num_key G;
	count_num_key Z;
	count_num_key X;
	count_num_key C;
	count_num_key V;
	count_num_key B;
};

struct rc_press_t
{
	//鼠标
	count_num_key L;
	count_num_key R;
};

class rc_key_c
{
public:
	rc_key_v_t Key;
	rc_press_t Press;

	void rc_key_v_set(RC_ctrl_t *RC);
	uint8_t read_key(count_num_key *temp_count, key_count_e mode, bool clear);
	bool read_key(count_num_key *temp_count, key_count_e mode, bool *temp_bool);
	void clear_key_count(count_num_key *temp_count);
	bool read_key_single(count_num_key *temp_count);
	bool read_key_single(count_num_key *temp_count, bool *temp_bool);
	bool read_key_even(count_num_key *temp_count);
	bool read_key_even(count_num_key *temp_count, bool *temp_bool);
	void sum_key_count(int16_t key_num, count_num_key *temp_count);
private:

};

extern struct Torque_Motor_Ctrl Damiao_Pitch;
#endif
