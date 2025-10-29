#ifndef __RTOSsystem_TASK_H
#define __RTOSsystem_TASK_H

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif

	
void RTOSsystem_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif	

#endif /* __RTOSsystem_TASK_H */
