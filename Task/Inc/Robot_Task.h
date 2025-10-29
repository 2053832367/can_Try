#ifndef Robot_TASK_H
#define Robot_TASK_H

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "app_preference.h"

#include "app_motor.h"
#include "Message_Task.h"
#include "Guard_Task.h"

#include "dev_system.h"

#include "protocol_dbus.h"

#include "algorithm_pid.h"
#include "algorithm_user_lib.h"
#include "chassis_power_control.h"


#include "drivers_statistic.h"

#include "tim.h"
extern "C++"
{
#include "algorithm_matrix.hpp"
}

#ifdef __cplusplus
extern "C" {
#endif
	
		void Gimbal_Task(void *pvParameters);
    void Chassis_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

//云台-------------------------------------------------------------------------

// 任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
// 云台控制周期
#define GIMBAL_CONTROL_TIME   1

// yaw,pitch控制通道以及状态开关通道
#define YawChannel    0
#define PitchChannel  1
// 选择云台状态 开关通道号
#define CHANNEL_LEFT  1
#define CHANNEL_RIGHT 0
// 遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADLINE  20

// 堵转时反转
#define TRIGGER_BLOCKED_ANGLE 1.0f
#define TRIGGER_BLOCKED_SPEED 10.0f
// 电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range      8191
// 电机编码值转化成角度值
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad   0.000766990394f //      2*  PI  /8192
#endif

//处理是否Pitch反装
#if ((GIMBAL_PITCH_MAX_ECD > GIMBAL_PITCH_MIN_ECD)&&(GIMBAL_PITCH_MAX_ECD - GIMBAL_PITCH_MIN_ECD < 4095))||\
		((GIMBAL_PITCH_MIN_ECD > GIMBAL_PITCH_MAX_ECD)&&(GIMBAL_PITCH_MIN_ECD - GIMBAL_PITCH_MAX_ECD > 4095))
#define PITCH_MOTOR_REVERSE (1)
#else
#define PITCH_MOTOR_REVERSE (-1)
#endif

typedef struct
{
    const motor_measure_t     *gimbal_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 angle;
    fp32 angle_error;
    fp32 angle_set;
	  fp32 speed_set;
	  fp32 angle_RAD;
		fp32 visual_offset_angle;
	  fp32 visual_send_angle;

	    fp32 give_speed;

    int16_t give_current;
    int16_t Prospect_current;
    int16_t Feedforward_current;
    fp32 Feedforward_KP;
	
    sPidTypeDef SpeedPid;
    sPidTypeDef PositinPid;
    sPidTypeDef FollowSpeedPid;
    sPidTypeDef FollowPositinPid;
    sPidTypeDef EnergyPositinPid;
    sPidTypeDef EnergySpeedPid;

} Gimbal_Main_Motor_t; // 云台主要电机数据

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 angle_set;
    int16_t give_current;

    sPidTypeDef SpeedPid;
    sPidTypeDef PositinPid;

} Gimbal_Minor_Motor_t; // 云台次要电机数据

// 拨弹轮电机结构体
struct trigger_motor_t
{
    fp32 new_angle;
    fp32 last_angle;
    int32_t cycle;
};

typedef struct
{
    bool RC_Flag;
    bool Visual_Flag;
    bool Energy_Flag;
    bool Loading_Flag;
    bool Fric_Flag;
    bool Shoot_Flag;
		bool AutoShoot_Flag;
    bool Shoot_Reversal_Flag;
	  bool RC_Shoot_Flag;
		bool Mirror_Flag;
		bool FC_Flag;
		bool ZM_Flag;
	  bool reset_damiao_pitch;
} Gimbal_Ctrl_Flags_t; // 云台控制标志位

typedef enum
{
    GIMBAL_NO_MOVE = 0x00,
    GIMBAL_Normal,
    GIMBAL_AIM,
    GIMBAL_ENERGY,
		GIMBAl_FRIC_CONTROL,
	  GIMBAL_NAV,
} gimbal_mode_e;

struct Gimbal_Data_t
{		 

  	bool Angle_mode;
		bool Time_mode;
    uint16_t pitch_offset_ecd;
	  fp32 Visual_offset_ecd;
	
    fp32 pitch_max_angle;
    fp32 pitch_min_angle;

    uint16_t Trigger_offset_ecd;

    uint8_t Gear;
		uint8_t Frame_Period;
	
    uint8_t Fric_Gear[3];
    fp32 FricSpeedSet;
		fp32 Last_FricSpeedSet;
		fp32 FricSpeed;
    fp32 Fric_Set[5];

		uint16_t Now_bullet_speed_limit;
		uint16_t Last_bullet_speed_limit;
		uint16_t Now_shooter_barrel_heat;
		uint8_t  Shoot_Num;
    uint8_t Shoot_Frequency;
    uint8_t Shoot_Frequency_m[3];

    uint16_t Loading_open;
    uint16_t Loading_close;
		
		uint16_t Trigger_once_num;
		uint16_t timer_set;
		
		fp32  Mirror_open;
		fp32  Mirror_close;
		fp32  Image_Hanging_Shot;
		fp32  Image_Normal;
		
		fp32  VS_yaw_Setangle;
};

typedef struct
{
  const DM_Motor_measure_t *gimbal_motor_measure;
	
	fp32 accel;
	fp32 speed;
	fp32 Last_angle;
	fp32 angle;
	fp32 angle_set;
	float vs_angle_set;
	float pos_set;
	float vel_set;
	float tor_set;
	float kp_set;
	float kd_set;
	float R;
	
	float VisualR_last_yaw;
	float VisualR_Error_angle;
	float VisualR_LuBo_yaw;     //左右无限数轴 [负无穷，正无穷]
	
	int VisualR_Yaw_cycle;
	
	
	sPidTypeDef SpeedPid;
  sPidTypeDef PositinPid;
	
	sPidTypeDef Visual_SpeedPid;
	sPidTypeDef Visual_PositinPid;
	
	float feedforward;

} Gimbal_DM_Motor_t; // 达秒电机数据

typedef struct
{
	float Gimbal_yaw;
	float Gimbal_yaw_speed;
	float Gimbal_pitch;
	
}Gimbal_Main_t;


class Gimbal_Ctrl: public Statistic, public ValidData
{
public:
    rc_key_c RC;
    const RC_ctrl_t *RC_Ptr;    
		


    uint32_t Gimbal_DWT_Count;
	  float Gimbal_DWT_dt;
		fp32 Last_Stop_TickCount;
    float Stop_TickCount;

		
		ramp_function_source_t  speed_ramp;
//    Gimbal_Main_Motor_t Yaw;
//    Gimbal_Main_Motor_t Pitch;
		Gimbal_Main_t Gimbal_data;

    float Kmg,G_compensation_out;

		Gimbal_DM_Motor_t   DM_Pitch; 
		Gimbal_DM_Motor_t		DM_Yaw;

    Gimbal_Minor_Motor_t Trigger;
		Gimbal_Minor_Motor_t Fric1;
    Gimbal_Minor_Motor_t Fric2;
		Gimbal_Minor_Motor_t Fric3;
		Gimbal_Minor_Motor_t Fric4;
		Gimbal_Minor_Motor_t Fric5;
    
		float Kff;     // 前馈增益
		float last_target;  // 上一次目标角度（用于前馈计算）
		

    Gimbal_Data_t Data;
    Gimbal_Ctrl_Flags_t Flags;
    gimbal_mode_e Mode;
    gimbal_mode_e Last_Mode;

    void Gimbal_Init(void);
    void Feedback_Update(void);
    void Control(void);
    void Behaviour_Mode(void);
    void Control_loop(void);

    fp32 motor_relative_ECD_to_angle(uint16_t ECD, uint16_t offset_ecd);
		fp32 Gyro_relative_angle_to_angle(fp32 angle, fp32 offset_angle);
		
		float forwardfeed(float in);
		float forwardfeed_pitch(float in);
		void process_angle(float visual_angle,float yaw_gyro,float *aim_angle);
		float normalize_angle_diff(float current, float target);
		float adjust_target_angle(float current, float target);
		
private:
    trigger_motor_t Trig;

    void RC_to_Control(fp32 *yaw_set, fp32 *pitch_set);
    void Behaviour_Control(fp32 *yaw_set, fp32 *pitch_set);
    void Flag_Behaviour_Control(void);
};


Gimbal_Ctrl *get_gimbal_ctrl_pointer(void);

extern void rc_key_v_fresh_Gimbal(RC_ctrl_t *RC);

//底盘-------------------------------------------------------------------------

extern QueueHandle_t Message_Queue;

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 3
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 1

//选择底盘状态 开关通道号
#define CHANNEL_LEFT  1
#define CHANNEL_RIGHT  0

typedef struct
{
	const motor_measure_t *chassis_motor_measure;
	fp32 accel;
	fp32 speed;
	fp32 speed_set;
	int16_t give_current;

} Chassis_Motor_t;//底盘接收编码器数据


#ifdef useSteering
struct Steering_Data_t
{
	fp32 angle;
	fp32 angle_last;
	int8_t angle_round;

	fp32 angle_set;
	fp32 angle_set_last;
	int8_t angle_set_round;
};

typedef struct
{
	const motor_measure_t *chassis_motor_measure;
	Steering_Data_t data;

	fp32 accel;
	fp32 speed;
	fp32 speed_set;

	uint32_t offset_ecd;

	fp32 angle_real;
	fp32 angle_set_real;

	int16_t give_current;
} Chassis_Steering_t;//底盘接收编码器数据
#endif

typedef struct
{
	bool Spin_Flag;
	bool RC_Flag;
	bool Visual_Flag;
	bool Energy_Flag;
	bool Looding_Flag;
	bool Fric_Flag;
	bool Shoot_Flag;
	bool Shoot_Reversal_Flag;
	bool Speed_Up_Flag;
	bool Velocity_Clac_Flag;
	bool game_sate_Flag;
		
	
} Chassis_Ctrl_Flags_t;//底盘控制标志位

typedef struct
{
	bool RC_Flag;
	bool Gimbal_Flag;
}
Chassis_Error_Flags_t;
typedef enum
{
	CHASSIS_NO_MOVE = 0,
	CHASSIS_FOLLOW_YAW,//跟随云台
	CHASSIS_FOLLOW_YAW_LIMIT,//有限制跟随云台
	CHASSIS_NO_FOLLOW_YAW,//不跟随云台
	CHASSIS_LITTLE_TOP,//小陀螺
	CHASSIS_VISION,
	CHASSIS_NAV,
	CHASSIS_LITTLE_TOP_HIGH,//小陀螺高功率模式,又可调速度模式
	CHASSIS_LITTLE_TOP_HIGH_FOLLOW,//设定值为TOP+HIGH切换时的值,跟随云台移动
} chassis_mode_e;//底盘工作状态

typedef enum
{
	STEERING_STOP,//舵轮停止旋转
	STEERING_NORMAL,//运动模式
	STEERING_FOLLOW_GIMBAL,//舵轮跟随云台
	STEERING_FOLLOW_CHASSIS,//舵轮跟随底盘
	STEERING_VECTOR_NO_FOLLOW,//舵轮保持上次设定角度
	STEERING_LIMIT,//舵轮限制旋转范围
	STEERING_LIMIT_UPDATE,//舵轮限制旋转范围
	STEERING_LITTLE_TOP,//底盘小陀螺状态下的舵轮
} chassis_steering_mode_e;//舵轮工作状态

typedef struct
{
	fp32 vx;      //底盘速度 前进方向 前为正，单位 m/s
	fp32 vy;      //底盘速度 左右方向 左为正  单位 m/s
	fp32 wz;      //底盘旋转角速度，逆时针为正 单位 rad/s
	fp32 vx_set;  //底盘设定速度 前进方向 前为正，单位 m/s
	fp32 vy_set;  //底盘设定速度 左右方向 左为正，单位 m/s
	fp32 wz_set;  //底盘设定旋转角速度，逆时针为正 单位 rad/s

	fp32 vx_max_speed;  //前进方向最大速度 单位m/s
	fp32 vx_min_speed;  //前进方向最小速度 单位m/s
	fp32 vy_max_speed;  //左右方向最大速度 单位m/s
	fp32 vy_min_speed;  //左右方向最小速度 单位m/s

	uint8_t Gear;  //等级
	fp32 Speed;
	fp32 Speed_Set;
	fp32 Vx_Set_Last;
	fp32 Vy_Set_Last;
	fp32 Speed_Set_Last;
	Matrix<3, 4> Speed_Set_m;//底盘速度
} Chassis_Velocity_t;

class Chassis_Ctrl:public Statistic, public ValidData
{
public:
	rc_key_c RC;
	const RC_ctrl_t *RC_Ptr;

	int16_t chassis_relative_ECD;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
	fp32 chassis_relative_RAD;
  fp32         Power_Set_KP;
	Chassis_Motor_t Motor[4];

	sPidTypeDef  Velocity_Pid;
	sPidTypeDef  Speed_Pid[4];
	sPidTypeDef  Follow_Gimbal_Pid;
	sPidTypeDef  Power_buffer_Pid;
	sPidTypeDef  chassis_setangle;
	sPidTypeDef  chassis_setangle_gyro; 

  first_order_filter_type_t Filter_vx;
  first_order_filter_type_t Filter_vy;
  first_order_filter_type_t Filter_vw;


	PowerClass           Power_Ctrl;
	Chassis_Velocity_t   Velocity;
	Chassis_Ctrl_Flags_t Flags;
	chassis_mode_e Mode;
	chassis_mode_e Last_Mode;

	void Chassis_Init(void);
	void Feedback_Update(void);
	void Control(void);
	void Behaviour_Mode(void);
	void Control_loop(void);
#ifdef useSteering
	Chassis_Steering_t Steering[4];
	chassis_steering_mode_e Steering_Mode;

	sPidTypeDef  steering_Speed_Pid[4];
	sPidTypeDef  steering_Angle_Pid[4];
#endif
private:
	void RC_to_Control(fp32 *vx_set, fp32 *vy_set);
	void Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set);
	void Vector_to_Wheel_Speed(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);
	void Flag_Behaviour_Control(void);
#ifdef  useSteering
	void Steering_Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);
	void Steering_Round_Calc(void);
	void Steering_Mode_Control(void);
#endif
};

extern void Chassis_Power_Limit(Chassis_Velocity_t  *Velocity,Message_Ctrl *Message);

extern void rc_key_v_fresh_Chassis(RC_ctrl_t *RC);
extern fp32 motor_ecd_to_relative_ecd(fp32 angle, fp32 offset_ecd);
extern Chassis_Ctrl *get_chassis_ctrl_pointer(void);

extern void System_Reset(void);

#endif /* __Robot_TASK_H */
