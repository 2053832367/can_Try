#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H

#include "dev_system.h"
#include "dev_can.h"

//#define USE_PWM_CONTROL_FRIC

#ifdef USE_PWM_CONTROL_FRIC
#define LEFT_FRIC_PWM_PIN          PA2
#define RIGHT_FRIC_PWM_PIN         PA1
#define FRIC_MOTOR_STOP_DUTY_CYCLE 1000
#endif


//DM_6220 力矩电机的输出幅度
#define DM_6220_P_MIN  -3.141593f
#define DM_6220_P_MAX   3.141593f
#define DM_6220_V_MIN -45.0f
#define DM_6220_V_MAX  45.0f
#define DM_6220_T_MIN -10.0f
#define DM_6220_T_MAX  10.0f
#define DM_6220_KP_MIN 0.0f
#define DM_6220_KP_MAX 500.0f
#define DM_6220_KD_MIN 0.0f
#define DM_6220_KD_MAX 5.0f

#define DM_4310_P_MIN -12.5f
#define DM_4310_P_MAX  12.5f
#define DM_4310_V_MIN -30.0f
#define DM_4310_V_MAX  30.0f
#define DM_4310_T_MIN -10.0f
#define DM_4310_T_MAX  10.0f
#define DM_4310_KP_MIN 0.0f
#define DM_4310_KP_MAX 500.0f
#define DM_4310_KD_MIN 0.0f
#define DM_4310_KD_MAX 5.0f


#define ERR_ID      0xFB
#define START_ID    0xFC
#define LOCK_ID     0xFD
#define SETPOINT_ID 0xFE


//达秒电机数据读取
#define Get_DM_Motor_Measure(ptr,Data,P_Min,P_Max,V_Min,V_Max,T_Min,T_Max)    \
{                                                       										  \
	(ptr)->ID            = (Data[0])&0x0F;                              			  \
	(ptr)->state         = (Data[0])>>4;         			                          \
	(ptr)->POS.Idata     = (Data[1] <<8)    | Data[2];     						          \
	(ptr)->VEl.Idata     = (Data[3] <<4)    |(Data[4]>>4); 								      \
	(ptr)->Torque.Idata  =((Data[4]&0xF)<<8)| Data[5];                          \
	(ptr)->POS.fdata     = uint_to_float((ptr)->POS.Idata,   P_Min,P_Max,16);   \
	(ptr)->VEl.fdata     = uint_to_float((ptr)->VEl.Idata,   V_Min,V_Max,12);   \
	(ptr)->Torque.fdata  = uint_to_float((ptr)->Torque.Idata,T_Min,T_Max,12);   \
	(ptr)->temperature_MOS=   (float)(Data[6]);																  \
	(ptr)->temperature_Rotro= (float)(Data[7]);																  \
}



//大疆电机数据读取
#define get_motor_measure(ptr, rx_message)                                                \
{                                                                                        \
	(ptr)->last_ecd = (ptr)->ecd;                                                         \
	(ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);          \
	(ptr)->speed_rpm = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
	(ptr)->given_current = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
	(ptr)->temperate = (rx_message)->Data[6];                                             \
}

//大疆电机数据读取
#define MA_get_motor_measure(ptr, Data)                                                \
{                                                                                         \
	(ptr)->last_ecd = (ptr)->ecd;                                                         \
	(ptr)->ecd = (uint16_t)(Data[0] << 8 | Data[1]);          \
	(ptr)->speed_rpm = (int16_t)(Data[2] << 8 | Data[3]);     \
	(ptr)->given_current = (int16_t)(Data[4] << 8 | Data[5]); \
	(ptr)->temperate = Data[6];                                             \
}

/*
除6020外，id2控制第一组电机，id1控制第二组电机(电机实际id和反馈id同步)
6020，id1控制第二组电机(电机实际id为1234，反馈id为5678)，id3控制第三组电机(电机实际id为567，反馈id为9 10 11)
*/
typedef enum
{
	//控制id	1，2，3
	CAN_DJI_Motor_Group1_ID = 0x1ff,
	CAN_DJI_Motor_Group2_ID = 0x200,
	CAN_DJI_Motor_Group3_ID = 0x2ff,
	//电机组1
	CAN_DJI_Motor1_ID = 0x201,
	CAN_DJI_Motor2_ID = 0x202,
	CAN_DJI_Motor3_ID = 0x203,
	CAN_DJI_Motor4_ID = 0x204,
	//电机组2
	CAN_DJI_Motor5_ID = 0x205,
	CAN_DJI_Motor6_ID = 0x206,
	CAN_DJI_Motor7_ID = 0x207,
	CAN_DJI_Motor8_ID = 0x208,
	//电机组3
	CAN_DJI_Motor9_ID = 0x209,
	CAN_DJI_Motor10_ID = 0x210,
	CAN_DJI_Motor11_ID = 0x211,
	CAN_SEND1_ID = 0x01,      //达妙电机发送ID
	CAN_JOINT1_ID = 0x00,     //达妙电机反馈ID

	CAN_CAP_GET_ID = 0x301,
	CAN_CAP_SENT_ID = 0x311,
	CAN_MPU_TO_Z=0x315,
	CAN_MPU_TO_X_Y=0x316,
	
	
	/*云台发送组*/
	DM_GimbalS_ID = 0x06,//0x70,
	DM_GimbalS_Pitch_ID =0x07, //0x71,
	/*云台接收组*/
	DM_GimbalR_ID = 0x014,//0x80,
	DM_GimbalR_Pitch_ID=0x15,//0x81,
	
} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
	uint8_t cnt;
	uint16_t offset;
	uint16_t angle_ecd;
	uint8_t mescnt;
} motor_measure_t;
//6020云台电机数据结构体
typedef struct
{
	int fdbPosition;        //电机的编码器反馈值
	int last_fdbPosition;   //电机上次的编码器反馈值
	int bias_position;      //机器人初始状态电机位置环设定值
	int real_current;         //实际电流
	int given_current;       //给定电流
	int round;              //电机转过的圈数
	int real_position;      //过零处理后的电机转子位置
	int last_real_position;//上次过零处理后的电机转子位置	
}gimbal_measure_t;


typedef struct
{
	float		P_MIN; 
	float		P_MAX; 
	float		V_MIN; 
	float		V_MAX; 
	float		T_MIN;
	float		T_MAX;
	float		KP_MIN;
	float		KP_MAX;
	float		KD_MIN; 
	float		KD_MAX; 
}DM_DATA_LIMIT;

typedef  union  
{
	float fdata;
	int   Idata;
}int16_float;


typedef struct
{
	uint8_t ID;             //ID[0]
	uint8_t state;          //ERR[0]
	int16_float POS;        //位置[1],[2]
	int16_float VEl;        //速度[3][4]
	int16_float Torque;     //扭矩信息[4][5]
	float temperature_MOS;  //驱动上 MOS 的平均温度[6]
	float temperature_Rotro;//电机内部线圈的平均温度[7]
}DM_Motor_measure_t;





class Motor_CAN_Ctrl
{
public:
	Motor_CAN_Ctrl(FDCAN_HandleTypeDef *CANx_, uint32_t StdID_, uint8_t Num_)
	{
		this->CANx = CANx_;
		this->StdID = StdID_;
		this->Num = Num_;
		Motor_Measure = new motor_measure_t[Num_];
	}
	const motor_measure_t *Get_Motor_Measure_Pointer(uint8_t i)
	{
		return &Motor_Measure[i];
	}
	void GetData(uint32_t &StdID_, uint8_t &Num_)
	{
		StdID_ = this->StdID;
		Num_ = this->Num;
	}
	motor_measure_t *GetData(uint8_t i)
	{
		return &Motor_Measure[(i & 3)];
	}
	FDCAN_HandleTypeDef *CANx;
private:
	motor_measure_t *Motor_Measure;
	uint16_t StdID;
	uint8_t Num;
};


class DM_Motor_CAN_Ctrl
{
public:
	DM_Motor_CAN_Ctrl(FDCAN_HandleTypeDef *CANx_,uint32_t StdID_,float P_MIN,float	P_MAX,float	V_MIN,float	V_MAX,float	T_MIN,float	T_MAX,float	KP_MIN,float	KP_MAX,float	KD_MIN, float	KD_MAX)
	{
		this->CANx  = CANx_;
		this->StdID = StdID_;
		LIMIT.P_MIN=P_MIN;
		LIMIT.P_MAX=P_MAX;
		LIMIT.V_MIN=V_MIN;
		LIMIT.V_MAX=V_MAX;
		LIMIT.T_MIN=T_MIN;
		LIMIT.T_MAX=T_MAX;
		LIMIT.KP_MIN=KP_MIN;
		LIMIT.KP_MAX=KP_MAX;
		LIMIT.KD_MIN=KD_MIN;
		LIMIT.KD_MAX=KD_MAX;
		DM_Motor_Measure = new DM_Motor_measure_t;
	}
	const DM_Motor_measure_t *Get_DM_Motor_Measure_Pointer(void)
	{
		return DM_Motor_Measure;
	}
	void ID_GetData(uint32_t &StdID_)
	{
		StdID_ = this->StdID;
	}
	DM_Motor_measure_t *GetData(void)
	{
		return DM_Motor_Measure;
	}
	
	FDCAN_HandleTypeDef *CANx;
	DM_DATA_LIMIT LIMIT;
	
private:
	DM_Motor_measure_t  *DM_Motor_Measure;
	uint16_t StdID;
};



/* 注意！！！
 * 一个1kHz的数据包占据占据11%的负载，控制报文和反馈报文个数的总和不超过9，
 * 1个控制控制报文可以控制4个电机，4个电机只需要占据（1+4）
 * useInfantry+useSteering时为舵轮步兵，can3为8
 * useHero+useSteering时为舵轮英雄，can3为9刚好满载
 */

class CAN_Ctrl
{
public:
	CAN_Ctrl()
		:Gimbal (&hfdcan1, CAN_DJI_Motor_Group1_ID, 4),//yaw,c610
		 Chassis(&hfdcan3, CAN_DJI_Motor_Group1_ID, 4),//麦轮*hfdcan2 ,临时调试改成了1，记得改成2
		 //Bottom (&hfdcan2, CAN_DJI_Motor_Group1_ID, 3),//yaw,super,拨弹盘hfdcan2
		#ifdef useHero
	 	 Fric  (&hfdcan1,   CAN_DJI_Motor_Group2_ID,3),//摩擦轮*3
		 Fric02(&hfdcan1,   CAN_DJI_Motor_Group1_ID,2)//摩擦轮*2
		#endif
		#ifdef useInfantry
		Fric(&hfdcan3, CAN_DJI_Motor_Group2_ID, 2),//摩擦轮*2+(4号陀螺仪用can接受)
		#endif
		Gimbal_DM_Yaw(&hfdcan2,DM_GimbalS_ID,DM_6220_P_MIN,DM_6220_P_MAX,DM_6220_V_MIN,DM_6220_V_MAX,DM_6220_T_MIN,DM_6220_T_MAX,DM_6220_KP_MIN,DM_6220_KP_MAX,DM_6220_KD_MIN,DM_6220_KD_MAX),
		Gimbal_DM_Pitch(&hfdcan1,DM_GimbalS_Pitch_ID,DM_4310_P_MIN,DM_4310_P_MAX,DM_4310_V_MIN,DM_4310_V_MAX,DM_4310_T_MIN,DM_4310_T_MAX,DM_4310_KP_MIN,DM_4310_KP_MAX,DM_4310_KD_MIN,DM_4310_KD_MAX)
		#ifdef useSteering
		,Steer(&hfdcan3, CAN_DJI_Motor_Group1_ID, 4)//舵轮*4
		#endif
	{}
	Motor_CAN_Ctrl Gimbal;
	Motor_CAN_Ctrl Chassis;
//	Motor_CAN_Ctrl Bottom;
	Motor_CAN_Ctrl Fric;
	DM_Motor_CAN_Ctrl  Gimbal_DM_Yaw;
	DM_Motor_CAN_Ctrl  Gimbal_DM_Pitch;


//	Motor_CAN_Ctrl Fric02;

	#ifdef useSteering
	Motor_CAN_Ctrl Steer;
	#endif
	void start_motor(uint16_t id);
	void SendData(FDCAN_HandleTypeDef *CANx, uint32_t StdID, const void *buf, uint8_t len);
	void SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2, int16_t Motor3, int16_t Motor4);
	void SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2, int16_t Motor3);
	void SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1, int16_t Motor2);
	void SendData(Motor_CAN_Ctrl *Motor, int16_t Motor1);
	
	/*达秒电机函数*/
	void DM_MIT_SendData(DM_Motor_CAN_Ctrl *DM_Motor,float P_des, float V_des,float KP_des,float KD_des,float T_ff);
	void SendData(FDCAN_HandleTypeDef *CANx,uint16_t id,uint8_t frame);
	void DM_Motor_SetZeroT(DM_Motor_CAN_Ctrl    *DM_Motor);
	void DM_Motor_clear_error(DM_Motor_CAN_Ctrl *DM_Motor);
	void DM_Motor_Enable(DM_Motor_CAN_Ctrl      *DM_Motor);
	void DM_Motor_UnEnable(DM_Motor_CAN_Ctrl    *DM_Motor);
  /*达秒电机函数*/
private:
	void SendData(CANctrl *CANx_Ctrl, uint32_t StdID, const void *buf, uint8_t len);



};

extern CAN_Ctrl CAN_Cmd;
extern void CAN1_Send(uint8_t *Rx_Message);
extern void CAN2_Send(uint8_t *Rx_Message);
extern void CAN3_Send(uint8_t *Rx_Message);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);




#endif /* __APP_MOTOR_H */
