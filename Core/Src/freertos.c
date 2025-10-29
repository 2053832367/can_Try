/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "app_preference.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RTOSsystem_task */
osThreadId_t RTOSsystem_taskHandle;
const osThreadAttr_t RTOSsystem_task_attributes = {
  .name = "RTOSsystem_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for Gimbal_task256 */
osThreadId_t Gimbal_task256Handle;
const osThreadAttr_t Gimbal_task256_attributes = {
  .name = "Gimbal_task256",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow5,
};
/* Definitions for Guard_task */
osThreadId_t Guard_taskHandle;
const osThreadAttr_t Guard_task_attributes = {
  .name = "Guard_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow6,
};
/* Definitions for Correspond_task */
osThreadId_t Correspond_taskHandle;
const osThreadAttr_t Correspond_task_attributes = {
  .name = "Correspond_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow4,
};
/* Definitions for Message_task */
osThreadId_t Message_taskHandle;
const osThreadAttr_t Message_task_attributes = {
  .name = "Message_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow5,
};
/* Definitions for CAN1_Rx_task */
osThreadId_t CAN1_Rx_taskHandle;
const osThreadAttr_t CAN1_Rx_task_attributes = {
  .name = "CAN1_Rx_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for CAN2_Rx_task */
osThreadId_t CAN2_Rx_taskHandle;
const osThreadAttr_t CAN2_Rx_task_attributes = {
  .name = "CAN2_Rx_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for Serial_Rx_task */
osThreadId_t Serial_Rx_taskHandle;
const osThreadAttr_t Serial_Rx_task_attributes = {
  .name = "Serial_Rx_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for DR16_Rx_task */
osThreadId_t DR16_Rx_taskHandle;
const osThreadAttr_t DR16_Rx_task_attributes = {
  .name = "DR16_Rx_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for CAN3_Rx_task */
osThreadId_t CAN3_Rx_taskHandle;
const osThreadAttr_t CAN3_Rx_task_attributes = {
  .name = "CAN3_Rx_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for Chassis_task */
osThreadId_t Chassis_taskHandle;
const osThreadAttr_t Chassis_task_attributes = {
  .name = "Chassis_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow5,
};
/* Definitions for UIDraw_task */
osThreadId_t UIDraw_taskHandle;
const osThreadAttr_t UIDraw_task_attributes = {
  .name = "UIDraw_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};
/* Definitions for Referee_Rx_task */
osThreadId_t Referee_Rx_taskHandle;
const osThreadAttr_t Referee_Rx_task_attributes = {
  .name = "Referee_Rx_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//��Ϣ���о��
QueueHandle_t Message_Queue;
QueueHandle_t CAN1_Rx_Queue;
QueueHandle_t CAN2_Rx_Queue;
QueueHandle_t CAN3_Rx_Queue;
QueueHandle_t Serial_Rx_Queue;
QueueHandle_t Referee_Rx_Queue;
QueueHandle_t DR16_Rx_Queue;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void RTOSsystem_Task(void *argument);
extern void Gimbal_Task(void *argument);
extern void Guard_Task(void *argument);
extern void Correspond_Task(void *argument);
extern void Message_Task(void *argument);
extern void CAN1_Rx_Task(void *argument);
extern void CAN2_Rx_Task(void *argument);
extern void Serial_Rx_Task(void *argument);
extern void DR16_Rx_Task(void *argument);
extern void CAN3_Rx_Task(void *argument);
extern void Chassis_Task(void *argument);
extern void UIDraw_Task(void *argument);
extern void Referee_Rx_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	Message_Queue = xQueueCreate(8, sizeof(ID_Data_t));
	CAN1_Rx_Queue = xQueueCreate(8, sizeof(ID_Data_t));
	CAN2_Rx_Queue = xQueueCreate(8, sizeof(ID_Data_t));
	CAN3_Rx_Queue = xQueueCreate(8, sizeof(ID_Data_t));
	Serial_Rx_Queue = xQueueCreate(4, sizeof(ID_Data_t));
	Referee_Rx_Queue = xQueueCreate(2, sizeof(ID_Data_t));
	DR16_Rx_Queue = xQueueCreate(2, sizeof(ID_Data_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of RTOSsystem_task */
  RTOSsystem_taskHandle = osThreadNew(RTOSsystem_Task, NULL, &RTOSsystem_task_attributes);

  /* creation of Gimbal_task256 */
  Gimbal_task256Handle = osThreadNew(Gimbal_Task, NULL, &Gimbal_task256_attributes);

  /* creation of Guard_task */
  Guard_taskHandle = osThreadNew(Guard_Task, NULL, &Guard_task_attributes);

  /* creation of Correspond_task */
  Correspond_taskHandle = osThreadNew(Correspond_Task, NULL, &Correspond_task_attributes);

  /* creation of Message_task */
  Message_taskHandle = osThreadNew(Message_Task, NULL, &Message_task_attributes);

  /* creation of CAN1_Rx_task */
  CAN1_Rx_taskHandle = osThreadNew(CAN1_Rx_Task, NULL, &CAN1_Rx_task_attributes);

  /* creation of CAN2_Rx_task */
  CAN2_Rx_taskHandle = osThreadNew(CAN2_Rx_Task, NULL, &CAN2_Rx_task_attributes);

  /* creation of Serial_Rx_task */
  Serial_Rx_taskHandle = osThreadNew(Serial_Rx_Task, NULL, &Serial_Rx_task_attributes);

  /* creation of DR16_Rx_task */
  DR16_Rx_taskHandle = osThreadNew(DR16_Rx_Task, NULL, &DR16_Rx_task_attributes);

  /* creation of CAN3_Rx_task */
  CAN3_Rx_taskHandle = osThreadNew(CAN3_Rx_Task, NULL, &CAN3_Rx_task_attributes);

  /* creation of Chassis_task */
  Chassis_taskHandle = osThreadNew(Chassis_Task, NULL, &Chassis_task_attributes);

  /* creation of UIDraw_task */
  UIDraw_taskHandle = osThreadNew(UIDraw_Task, NULL, &UIDraw_task_attributes);

  /* creation of Referee_Rx_task */
  Referee_Rx_taskHandle = osThreadNew(Referee_Rx_Task, NULL, &Referee_Rx_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

