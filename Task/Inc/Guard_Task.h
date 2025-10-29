#ifndef Guard_TASK_H
#define Guard_TASK_H

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "app_preference.h"
#include "dev_can.h"
//#include "iwdg.h"

#ifdef __cplusplus
extern "C" {
#endif

	void Guard_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#define GUARD_TOTAL_NUM ID_e_count

//extern IWDG_HandleTypeDef hiwdg1;

struct Error_Flags_t
{
	bool Gimbal;
	bool Visual;
	bool Judge;
};

struct SG_Data_t
{
	ID_e Name;//名称
	bool Enable;//使能开关
	bool Start;//初始化等待完成标志位
	bool Close;//任务异常次数过多关闭标志位(触发回调超过200ms关闭该id警戒任务)
	bool Porper;//任务正常标志位
	uint32_t Time;//计数器
	uint32_t StartValue;//初始化等待时间
	uint32_t MaxValue;//最大超时值
	uint32_t CloseValue;//触发过多关闭超时值
	uint32_t DiffValue;//时间差
	void (*errcallback)(uint8_t id);//异常回调函数
	void (*closecallback)(uint8_t id);//关闭回调函数
};

class Guard_Ctrl
{
public:
	void Start(void);
	void Scan(void);
	void Feed(ID_e Name);
	bool Return(ID_e Name);
private:
	void Init(ID_e Name, uint32_t StartValue, uint32_t MaxValue, void(*errcb)(uint8_t id));
	void Init(ID_e Name, uint32_t StartValue, uint32_t MaxValue, void(*errcb)(uint8_t id), bool Close);
	void Init(ID_e Name, uint32_t StartValue, uint32_t MaxValue, void(*errcb)(uint8_t id), bool Close, uint32_t CloseValue, void(*closecb)(uint8_t id));
	void Guard_Enable(void);

	SG_Data_t SG_Structure[GUARD_TOTAL_NUM];
};
extern Guard_Ctrl Guard;

extern void IWDG_Feed(void);  //喂狗函数
void Error_Enable(uint8_t name);
void System_RESET(uint8_t id);
void Guard_Return(uint8_t id);
void Close_Enable(uint8_t id);

Guard_Ctrl *get_guard_ctrl_pointer();
extern ID_e Guard_ID;

#endif

