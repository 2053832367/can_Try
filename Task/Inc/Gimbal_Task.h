#ifndef Chassis_TASK_H
#define Chassis_TASK_H

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

#include "drivers_statistic.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void Gimbal_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

// 任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
// 云台控制周期
#define GIMBAL_CONTROL_TIME 1

// yaw,pitch控制通道以及状态开关通道
#define YawChannel 0
#define PitchChannel 1
// 选择云台状态 开关通道号
#define CHANNEL_LEFT 1
#define CHANNEL_RIGHT 0
// 遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADLINE 20

// 堵转时反转
#define TRIGGER_BLOCKED_ANGLE 1.0f
#define TRIGGER_BLOCKED_SPEED 10.0f
// 电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range 8191
// 电机编码值转化成角度值
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
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
    const motor_measure_t *gimbal_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 angle;
    fp32 angle_error;
    fp32 angle_set;
    int16_t give_current;

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
    bool Shoot_Reversal_Flag;
    bool Autoshoot_Flag;
} Gimbal_Ctrl_Flags_t; // 云台控制标志位

typedef enum
{
    GIMBAL_NO_MOVE = 0x00,
    GIMBAL_Normal,
    GIMBAL_AIM,
    GIMBAL_ENERGY,
} gimbal_mode_e;

struct Gimbal_Data_t
{
    uint16_t pitch_offset_ecd;
    fp32 pitch_max_angle;
    fp32 pitch_min_angle;

    uint16_t Trigger_offset_ecd;

    uint8_t Gear;

    uint8_t Fric_Gear[3];
    fp32 FricSpeedSet;
    fp32 Fric_Set[3];

		uint16_t Last_bullet_speed_limit;
	
    uint8_t Shoot_Frequency;
    uint8_t Shoot_Frequency_m[3];

    uint16_t Loading_open;
    uint16_t Loading_close;
};

class Gimbal_Ctrl: public Statistic, public ValidData
{
public:
    rc_key_c RC;
    const RC_ctrl_t *RC_Ptr;

    Gimbal_Main_Motor_t Yaw;
    Gimbal_Main_Motor_t Pitch;
    Gimbal_Minor_Motor_t Trigger;
    Gimbal_Minor_Motor_t Fric1;
    Gimbal_Minor_Motor_t Fric2;

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

private:
    trigger_motor_t Trig;

    void RC_to_Control(fp32 *yaw_set, fp32 *pitch_set);
    void Behaviour_Control(fp32 *yaw_set, fp32 *pitch_set);
    void Flag_Behaviour_Control(void);
};


Gimbal_Ctrl *get_gimbal_ctrl_pointer(void);

extern void rc_key_v_fresh(RC_ctrl_t *RC);
extern void System_Reset(void);
#endif /* __GIMBAL_TASK_H */

