#ifndef __APP_SERIAL_H
#define __APP_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include <stdint.h>	
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "app_preference.h"
#include "dev_serial.h"

	
#ifdef __cplusplus
}
#endif

#define SERIAL1 USART1
#define SERIAL3 USART3
#define SERIAL5 UART5
#define SERIAL7 UART7
#define SERIAL10 USART10

typedef enum
{
    game_robot_state_id_ = 0x01,
    game_robot_HP_id_ = 0x02,
		game_status_id_ = 0x03,

    visual_posture_id = 0x01,
    game_robot_state_id = 0x02,
    visual_mode_id = 0x04,
    game_robot_HP_id = 0x08,
    chassis_data_id = 0x51,
}serial_msg_mode_e;

struct Serial_Data_t
{
    uint8_t Header;
    uint8_t Tail;
    uint8_t Lenth0;
    uint8_t Lenth1;
    uint8_t Lenth2;
    uint8_t Lenth3;
		uint8_t buffer_size;
    uint8_t Len;
    uint8_t Temp;
		uint8_t Mode;
    uint8_t **Data;
    Serial_Data_t(uint8_t Header_, uint8_t Tail_, uint8_t Lenth0_,uint8_t Lenth1_,uint8_t Lenth2_,uint8_t Lenth3_, uint8_t buffer_size_, uint8_t Mode_)
        :Header(Header_), Tail(Tail_), Lenth0(Lenth0_),Lenth1(Lenth1_),Lenth2(Lenth2_),Lenth3(Lenth3_), buffer_size(buffer_size_), Mode(Mode_)
    {
        Data = new uint8_t*[2];
				Data[0] = new uint8_t[buffer_size_];
				Data[1] = new uint8_t[buffer_size_];
    };
};
//达妙板子需要更改串口
class Serial_Ctrl
{
public:
//达妙板子需要更改串口
    Serial_Ctrl()
        :Serial1(Serial1_Data_Header, Serial1_Data_Tail, Serial1_Data_Lenth0, Serial1_Data_Lenth1, Serial1_Data_Lenth2, Serial1_Data_Lenth3, Serial1_Buffer_Size, Serial1_Mode),
         Serial3(Serial3_Data_Header, Serial3_Data_Tail, Serial3_Data_Lenth0, Serial3_Data_Lenth1, Serial3_Data_Lenth2, Serial3_Data_Lenth3, Serial3_Buffer_Size, Serial3_Mode),
         Serial5(Serial5_Data_Header, Serial5_Data_Tail, Serial5_Data_Lenth0, Serial5_Data_Lenth1, Serial5_Data_Lenth2, Serial5_Data_Lenth3, Serial5_Buffer_Size, Serial5_Mode),
         Serial7(Serial7_Data_Header, Serial7_Data_Tail, Serial7_Data_Lenth0, Serial7_Data_Lenth1, Serial7_Data_Lenth2, Serial7_Data_Lenth3, Serial7_Buffer_Size, Serial7_Mode),
         Serial10(Serial10_Data_Header, Serial10_Data_Tail, Serial10_Data_Lenth0, Serial10_Data_Lenth1, Serial10_Data_Lenth2, Serial10_Data_Lenth3, Serial10_Buffer_Size, Serial10_Mode)
    {}

    void Hook(USART_TypeDef *SERIAL, bool mode);
    void Handle(Serialctrl *Serial, Serial_Data_t *Usart, bool mode);
    void Send_to_Message(Serialctrl *SerialCtrl , bool Memory);

    uint8_t Get_Data(Serial_Data_t *Serial, uint8_t *buf);

    ~Serial_Ctrl() {}

    Serial_Data_t Serial1;
    Serial_Data_t Serial3;
    Serial_Data_t Serial5;
    Serial_Data_t Serial7;
    Serial_Data_t Serial10;

private:
    void *buf;
};
extern Serial_Ctrl Serial_Cmd;

extern void Serial_ALL_Init();
#endif
