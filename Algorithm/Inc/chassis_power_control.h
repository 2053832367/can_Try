
#ifndef _CHASSIS_POWER_CONTROL_H
#define _CHASSIS_POWER_CONTROL_H

#include "arm_math.h"
#include "algorithm_pid.h"

#define TOQUE_COEFFICIENT 1.99688994e-6f
/*
k1调，使其车堵转，使裁判系统功率达到功率上限附近，超功率时加大k1.没达到时则减小
k2调，使其车小陀螺，使裁判系统功率达到功率上限附近，超功率时加大k2.没达到时则减小

调试原理：公式-P=k1*F*F+b*V*F+k2*V*V+C,(二元一次方程)
因为堵转时速度基本很小，功率主要为力矩决定，所以k1影响大。调k1
小陀螺时速度主导，则调k2
*/
#define POWER_K1 			  1.82e-07f
#define POWER_K2			  1.92e-07f  //1.453e-07f
#define POWER_CONSTANT  1.21f
#define CAP_POWER_OPEN  70
#define CAP_POWER_CLOSE 5

typedef struct
{
 float K1;
 float K2;


 float constant;//,常数
	
 float Cap_power_open;
 float Cap_power_close;

}Chassis_Power_K;
typedef struct
{
	 float power_buffer_set;//,缓冲能量pid目标值
	 float power_buffer_out;//,缓冲能量pid输出限制
		
	 uint16_t grade_power_limit;//,此时等级的功率限制
	 float Max_input_power;//，最大功率限制
	 float Chassis_Max_power;//，最大功率限制
	 float Chassis_judge_power;
}Chassis_Power_limit;
typedef struct
{
	float chassis_speed_rpm[4];//，速度反馈
	float initial_total_power;//,计算得到的总功率
	float initial_give_power[4];//，两个轮子的功率
	float scaled_give_power[4];//,缩放后的功率
	float scaled_total_power;//,缩放后的总功率
	float send_current_value[4];//，计算得到的发送电流值
} Chassis_Power_calc;
class PowerClass
{
private:

public:
	
    Chassis_Power_K     Power_K;
    Chassis_Power_limit Power_limit;
    Chassis_Power_calc  Power_calc;

    float power_buffer_set;
    float cap_state;
		void  Power_Feedback_Update();
		void  Power_Calc();
    PowerClass();
};


#endif
