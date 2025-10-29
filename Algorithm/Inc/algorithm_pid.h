#ifndef __PID_H
#define __PID_H

#include "dev_system.h"

#ifdef __cplusplus
extern "C" {
#endif
	
#define useFreeRTOS
//	
//#ifndef useFreeRTOS
//#define osDelay( ms )   delay_ms( ms )
//#define Delay_ms( ms )  delay_ms( ms )
//#define Delay_us( us )  delay_us( us )
//#else
//#define osDelay( ms )   vTaskDelay( pdMS_TO_TICKS( ms ) )
//#define Delay_ms( ms )  delay_xms( ms )
//#define Delay_us( us )  delay_xus( us )
//#endif
	
#ifdef __cplusplus
}
#endif

//#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

#define LimitBand(input, max, min)   \
    {                                \
        if (input > max)             \
        {                            \
            input = max;             \
        }                            \
        else if (input < min)        \
        {                            \
            input = min;             \
        }                            \
    }

#define TIMERPERCYCLE TIMERPER_1MS

#define TIMERPER_1US	1000000	
#define TIMERPER_1MS	1000

/**********论域宏定义**********/
#define NB                -0.3f
#define NM                -0.2f
#define NS                -0.1f
#define ZO                 0.0f
#define PS                 0.1f
#define PM                 0.2f
#define PB                 0.3f

/***********Kp规则表***********/
static const float Kp_Rules_Table[7][7] =
{
  PB, PB, PM, PM, PS, ZO, ZO,
  PB, PB, PM, PS, PS, ZO, NS,
  PM, PM, PM, PS, ZO, NS, NS,
  PM, PM, PS, ZO, NS, NM, NM,
  PS, PS, ZO, NS, NS, NM, NM,
  PS, ZO, NS, NM, NM, NM, NB,
  ZO, ZO, NM, NM, NM, NB, NB
};

/***********Ki规则表***********/
static const float Ki_Rules_Table[7][7] =
{
  NB, NB, NM, NM, NS, ZO, ZO,
  NB, NB, NM, NS, NS, ZO, ZO,
  NB, NM, NS, NS, ZO, PS, PS,
  NM, NM, NS, ZO, PS, PM, PM,
  NM, NS, ZO, PS, PS, PM, PB,
  ZO, ZO, PS, PS, PM, PB, PB,
  ZO, ZO, PS, PM, PM, PB, PB
};

/***********Kd规则表***********/
static const float Kd_Rules_Table[7][7] =
{
  PS, NS, NB, NB, NB, NM, PS,
  PS, NS, NB, NM, NM, NS, ZO,
  ZO, NS, NM, NM, NS, NS, ZO,
  ZO, NS, NS, NS, NS, NS, ZO,
  ZO, ZO, ZO, ZO, ZO, ZO, ZO,
  PB, NS, PS, PS, PS, PS, PB,
  PB, PM, PM, PM, PS, PS, PB
};

typedef enum
{
    INIT = 0x00,
    POSITION,//位置式
    DELTA//增量式
}PidMode;

typedef struct
{
    uint32_t sampleTime;
    uint32_t nowTime;
    uint32_t lastTime;
    uint32_t passTime;

}PidTimerDef;

typedef struct
{
    bool trapezoidalintegral;  //梯形积分开关
    bool processVariable;      //微分先行开关
    bool fuzzyController;      //模糊PID开关
    bool disturbanceRejection; //DR_PID开关
}PidChangerTypeDef;

typedef struct
{
    float errorRate;

    float deltaKp;
    float deltaKi;
    float deltaKd;

    uint8_t IndexE[2];
    uint8_t IndexER[2];
    uint8_t MembershipE[2];
    uint8_t MembershipER[2];
}FuzzyPidTypeDef;

class sPidTypeDef
{
public:
    sPidTypeDef(PidMode mode_, fp32 Kp_, fp32 Ki_, fp32 Kd_, fp32 max_out_ = 30000, fp32 max_Iout_ = 3000, fp32 band_I_ = 3000)
        :mode(mode_), Kp(Kp_), Ki(Ki_), Kd(Kd_), max_out(max_out_), max_Iout(max_Iout_), band_I(band_I_)
    {}
    sPidTypeDef() {};
    PidMode mode;

    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_Iout; //最大积分输出

    fp32 band_I;
    fp32 set;
    fp32 ref;

    fp32 out;
    fp32 out_single;
    fp32 P_out;
    fp32 I_out;
    fp32 D_out;

    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

    ~sPidTypeDef() {}
};

class cPidTypeDef:public sPidTypeDef
{
public:
    cPidTypeDef() {}
    cPidTypeDef(PidMode mode_, fp32 Kp_, fp32 Ki_, fp32 Kd_, fp32 max_out_ = 30000, fp32 max_Iout_ = 3000, fp32 band_I_ = 3000\
        , float dead_band_ = 0, float pvCoefficient_ = 1, float filter_D_ = 0, float minInterval_ = 0, float maxInterval_ = 0, float Cycle_ = 0.001)\
        : sPidTypeDef(mode_, Kp_, Ki_, Kd_, max_out_, max_Iout_, band_I_)\
        , dead_band(dead_band_), pvCoefficient(pvCoefficient_), filter_D(filter_D_), minInterval(minInterval_), maxInterval(maxInterval_), Cycle(Cycle_)\
    {}
    cPidTypeDef(float dead_band_ = 0, float pvCoefficient_ = 1, float filter_D_ = 0, float minInterval_ = 0, float maxInterval_ = 0, float Cycle_ = 0.001)\
        : dead_band(dead_band_), pvCoefficient(pvCoefficient_), filter_D(filter_D_), minInterval(minInterval_), maxInterval(maxInterval_), Cycle(Cycle_)\
    {}

    float dead_band;
    float pvCoefficient;  //微分先行滤波系数
    float filter_D;       //越小效果越好，但系统灵敏度会下降。反之，效果变差，灵敏度提高。0为关闭惯性一阶惯性系统。
    float minInterval;    //变速积分区间
    float maxInterval;

    float Cycle;

    PidChangerTypeDef PidChanger;
    FuzzyPidTypeDef fuzzyPID;

    float lastRef;
    float perRef;
    float wc;       //启用DR_PID时，wc决定闭环响应速度，kp决定抗扰性能，因此先调wc，再调kp
    //速度环wc > 1, 角度环wc < 1。若系统超调，可以通过在输入端增加低通滤波器来消除超调。
#ifndef useFreeRTOS
    PidTimerDef pidCycle;
#endif
};

typedef enum
{
    SimplePID,
    ComplexPID,
}UsingPID_e;

void PIDChangerInit(cPidTypeDef *pid, bool tlBool, bool pvBool, bool fcBool, bool drBool);

class PID_Ctrl
{
public:
    void Init(sPidTypeDef *pid, PidMode mode, fp32 Kp, fp32 Ki, fp32 Kd, fp32 max_out, fp32 max_Iout, fp32 band_I);
    void Init(cPidTypeDef *pid, PidMode mode, fp32 Kp, fp32 Ki, fp32 Kd, fp32 max_out, fp32 max_Iout, fp32 band_I,
        float dead_band, float pvCoefficient, float filter_D, float minInterval, float maxInterval, float Cycle);
    fp32 Calc(sPidTypeDef *pid, fp32 ref, fp32 set);
    void Clear(sPidTypeDef *pid);
    fp32 Calc(cPidTypeDef *pid, fp32 ref, fp32 set);
    void Clear(cPidTypeDef *pid);
};

extern PID_Ctrl PID;

#endif

//测试稳定后删除,复杂pid待测试
// typedef struct
// {
//     uint8_t mode;
//     //PID 三参数
//     fp32 Kp;
//     fp32 Ki;
//     fp32 Kd;

//     fp32 max_out;  //最大输出
//     fp32 max_Iout; //最大积分输出

//     fp32 set;
//     fp32 fdb;

//     fp32 out;
//     fp32 Pout;
//     fp32 Iout;
//     fp32 Dout;
//     fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
//     fp32 error[3]; //误差项 0最新 1上一次 2上上次

// } PidTypeDef;

// typedef struct PidTypeDef
// {
//     PidMode mode;

//     float Kp;
//     float Ki;
//     float Kd;
//     float wc;         //启用DR_PID时，wc决定闭环响应速度，kp决定抗扰性能，因此先调wc，再调kp
//     //速度环wc > 1, 角度环wc < 1。若系统超调，可以通过在输入端增加低通滤波器来消除超调。
//     float set;
//     float ref;
//     float lastRef;
//     float perRef;
//     float error[3];
//     float Dbuf[3];

//     float out;
//     float P_out;
//     float I_out;
//     float D_out;

//     float max_Iout;
//     float max_out;
//     float band_I;
//     float dead_band;
//     float filter_D;       //越小效果越好，但系统灵敏度会下降。反之，效果变差，灵敏度提高。0为关闭惯性一阶惯性系统。
//     float pvCoefficient;  //微分先行滤波系数
//     float minInterval;    //变速积分区间
//     float maxInterval;

//     PidChangerTypeDef PidChanger;
//     FuzzyPidTypeDef fuzzyPID;

//     void (*vParmaInitFun)(struct PidTypeDef *pid, const float PID_Coefficient[3], float max_Iout,
//         float max_out, float band_I, float dead_band, float pvCoefficient,
//         float filter_D, float	minInterval, float maxInterval, float Cycle);
//     void (*vChangerInitFun)(struct PidTypeDef *pid, bool tlBool, bool pvBool, bool fcBool, bool drBool);
//     void (*vClearFun)(struct PidTypeDef *pid);
//     float (*fCalcFun)(struct PidTypeDef *pid, float set, float ref, PidMode mode);
//     PidMode(*pmGetFun)(struct PidTypeDef *pid);
//     PidChangerTypeDef(*pcGetFun)(struct PidTypeDef *pid);

// #ifndef useFreeRTOS
//     PidTimerDef pidCycle;
// #endif

// }PidTypeDef;
