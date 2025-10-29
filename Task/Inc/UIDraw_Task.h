#ifndef UIDraw_TASK_H
#define UIDraw_TASK_H

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "app_preference.h"
#include "protocol_ui.h"
#include "drivers_statistic.h"

#ifdef __cplusplus
extern "C" {
#endif

    void UIDraw_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

extern QueueHandle_t Message_Queue;


typedef enum
{
    UIDefult,
    UIADD,
    UIChange,
}UI_Mode_e;


class UI_Draw_Ctrl :public Statistic, UIGraph
{
public:
	  uint64_t Cnt;
    UI_Mode_e Mode;

    void Init();
    void Feedback_Update();
    void UI_ADD();
    void UI_Change();
private:
    void UI_Sent(uint8_t *ptr, uint8_t Len);
    void UI_Char_Sent(uint8_t *ptr);

    uint8_t Seq;
    uint8_t Len;
    uint8_t GraphData[105];

//    ext_client_custom_graphic_single_tt ext_client_custom_graphic_single;
//    ext_client_custom_character_tt ext_client_custom_character;
};

UI_Draw_Ctrl *get_UI_Draw_Ctrl_Pointer();
#endif

