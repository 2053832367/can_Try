#include "RTOSsystem_Task.h"

void RTOSsystem_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {		
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}
