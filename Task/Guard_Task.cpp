#include "Guard_Task.h"

Guard_Ctrl Guard;
Error_Flags_t Error_Flag;

void Guard_Task(void *pvParameters)
{
   /* USER CODE BEGIN StartDefaultTask */
	Guard.Start();
	osDelay(300);
  /* Infinite loop */
  for(;;)
  {		
	Guard.Scan();
    IWDG_Feed();
    osDelay(2);
  }
  /* USER CODE END StartDefaultTask */
}

//警戒任务开始
void Guard_Ctrl::Start(void)
{
    // Init(CanData1, 1000 ,100, &System_RESET);
    // Init(CanData2, 1000, 100, &System_RESET);
    // Init(ChassisData, 50000, 500, &Error_Enable, true ,30000, &Close_Enable);
    // Init(RC_Data, 1000, 200, &System_RESET);
    // Init(GimbalData, 1000, 100, &System_RESET);
    // Init(CorrespondenceData, 1000 ,100, &System_RESET);
}
//警戒任务初始化
void Guard_Ctrl::Init(ID_e Name, uint32_t StartValue, uint32_t MaxValue, void(*errcb)(uint8_t id), bool Close, uint32_t CloseValue, void(*closecb)(uint8_t id))
{
    SG_Structure[Name].Enable = false;//默认关闭
    SG_Structure[Name].Start = true;//默认打开
    SG_Structure[Name].Time = 0;
    SG_Structure[Name].DiffValue = 0;
    SG_Structure[Name].Name = Name;
    SG_Structure[Name].Porper = false;
    SG_Structure[Name].StartValue = StartValue;
    SG_Structure[Name].MaxValue = MaxValue;
    SG_Structure[Name].Close = Close;
    SG_Structure[Name].CloseValue = CloseValue;
    if(errcb == NULL)
    {
        SG_Structure[Name].errcallback = &Guard_Return;
    }
    else
    {
        SG_Structure[Name].errcallback = errcb;
    }
    if(closecb == NULL)
    {
        SG_Structure[Name].closecallback = &Guard_Return;
    }
    else
    {
        SG_Structure[Name].closecallback = closecb;
    }
}

void Guard_Ctrl::Init(ID_e Name, uint32_t StartValue, uint32_t MaxValue, void(*errcb)(uint8_t id))
{
    Init(Name, StartValue, MaxValue, errcb, false, 0, &Guard_Return);
}
void Guard_Ctrl::Init(ID_e Name, uint32_t StartValue, uint32_t MaxValue, void(*errcb)(uint8_t id), bool Close)
{//默认200ms
    Init(Name, StartValue, MaxValue, errcb, Close, 200, &Guard_Return);
}
//警戒任务扫描
void Guard_Ctrl::Scan(void)
{
    uint8_t i;
    for(i = 0;i < GUARD_TOTAL_NUM;i++)
    {
        if(SG_Structure[i].Start == true && (SG_Structure[i].StartValue != 0))
        {//初始化检测
            SG_Structure[i].DiffValue = xTaskGetTickCount() - SG_Structure[i].Time;
            if(SG_Structure[i].DiffValue > SG_Structure[i].StartValue)
            {//超时执行回调
                SG_Structure[i].errcallback(i);
                if(((int32_t)(SG_Structure[i].DiffValue - SG_Structure[i].StartValue) > SG_Structure[i].CloseValue) && (SG_Structure[i].Close == true))
                {//超时后等待关闭
                    SG_Structure[i].Start = false;
                    SG_Structure[i].closecallback(i);
                }
            }
        }
        else if((SG_Structure[i].Enable == true) && (SG_Structure[i].MaxValue != 0))
        {//运行检测
            SG_Structure[i].DiffValue = xTaskGetTickCount() - SG_Structure[i].Time;
            if(SG_Structure[i].DiffValue > SG_Structure[i].MaxValue)
            {//超时执行回调
                SG_Structure[i].errcallback(i);
                if(((int32_t)(SG_Structure[i].DiffValue - SG_Structure[i].MaxValue) > SG_Structure[i].CloseValue) && (SG_Structure[i].Close == true))
                {//超时后等待关闭
                    SG_Structure[i].Enable = false;
                    SG_Structure[i].closecallback(i);
                }
            }
            else
            {//运行则执行关闭回调
                if(SG_Structure[i].Close == true)
                {
                    SG_Structure[i].closecallback(i);
                }
            }
        }
    }

		if((hfdcan1.Instance->CCCR&0x01) == 1)
		{
			hfdcan1.Instance->CCCR &= ~(1 << 0);
		}
		if((hfdcan2.Instance->CCCR&0x01) == 1)
		{
			hfdcan2.Instance->CCCR &= ~(1 << 0);
		}
		if((hfdcan3.Instance->CCCR&0x01) == 1)
		{
			hfdcan3.Instance->CCCR &= ~(1 << 0);
		}
}
//警戒任务喂狗
void Guard_Ctrl::Feed(ID_e Name)
{
    if(Name == FaultData)
    {
        return;
    }
    //若正常运行或重连则打开运行超时检测关闭初始化检测和错误标志
    Guard.SG_Structure[Name].Enable = true;
    Guard.SG_Structure[Name].Start = false;
    Guard.SG_Structure[Name].Porper = true;
    Guard.SG_Structure[Name].Time = xTaskGetTickCount();
}
//错误处理函数
void Error_Enable(uint8_t id)
{
    switch(id)
    {
    case ChassisData:
    {
//        pwmWrite(PH6, 1000);
    }
    break;
    default:
    break;
    }
}
//关闭处理函数
void Close_Enable(uint8_t id)
{
    switch(id)
    {
    case ChassisData:
    {
//        pwmWrite(PH6, 0);
    }
    break;
    default:
    break;
    }
}
//警戒任务使能(feed中的使能只针对已运行任务)
void Guard_Ctrl::Guard_Enable(void)
{
    uint8_t i;
    for(i = 0;i < GUARD_TOTAL_NUM;i++)
    {
        Guard.SG_Structure[i].Enable = 1;
    }
}
//警戒任务默认回调函数
void Guard_Return(uint8_t id)
{
    return;
}

//返回任务是否正常工作
bool Guard_Ctrl::Return(ID_e Name)
{
    return Guard.SG_Structure[Name].Porper;
}

Guard_Ctrl *get_guard_ctrl_pointer()
{
    return &Guard;
}

void IWDG_Feed(void)
{
//		HAL_IWDG_Refresh(&hiwdg1);
}

//void System_RESET(uint8_t id)
//{
//    SCB->AIRCR = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
//        (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
//        SCB_AIRCR_SYSRESETREQ_Msk);
//}
