#ifndef __TASKS_H
#define __TASKS_H

#include "Robot_Task.h"
#include "Message_Task.h"
#include "Guard_Task.h"
#include "Correspond_Task.h"
#include "UIDraw_Task.h"

//#include "app_led.h"
//#include "app_motor.h"
//#include "app_power_ctrl.h"
//#include "app_serial.h"
//#include "app_preference.h"

extern Chassis_Ctrl  Chassis;
extern Gimbal_Ctrl   Gimbal;
extern Message_Ctrl  Message;
extern Guard_Ctrl    Guard;
extern UI_Draw_Ctrl  UIDraw;
extern Correspondence_ctrl Corres;

#endif
