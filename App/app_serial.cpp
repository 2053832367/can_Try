#include "app_serial.h"
#include "Message_Task.h"
#include "app_preference.h"
#include "drivers_dma.h"

Serial_Ctrl Serial_Cmd;
uint8_t judge1=0,judgedatalen=0;
uint8_t judgedata[200];

void Serial1_Hook(bool mode)
{
    Serial_Cmd.Hook(SERIAL1, mode);
}

void Serial3_Hook(bool mode)
{
    Serial_Cmd.Hook(SERIAL3, mode);
}

void Serial4_Hook(bool mode)
{
    Serial_Cmd.Hook(SERIAL4, mode);
}

void Serial7_Hook(bool mode)
{
    Serial_Cmd.Hook(SERIAL7, mode);
}

void Serial8_Hook(bool mode)
{
    Serial_Cmd.Hook(SERIAL8, mode);
}

void Serial_ALL_Init(void)
{
#if ( Serial1_Mode == Serial_NORMAL_Mode )
    HAL_UART_Receive_IT(&huart1,&Serial1_Ctrl.receive_RXNE,1);   // 重新使能接收中断
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  //idle interrupt
#endif
#if ( Serial1_Mode == Serial_DMA_Mode )
    MA_UART_Receive_DMA_Init(&huart1, &hdma_usart1_rx, (uint8_t *)&(Serial_Cmd.Serial1.Data[0][1]), (uint8_t *)&(Serial_Cmd.Serial1.Data[1][1]), Serial1_Buffer_Size);
#endif
	
#if ( Serial3_Mode == Serial_NORMAL_Mode )
		HAL_UART_Receive_IT(&huart3,&Serial3_Ctrl.receive_RXNE,1);   // 重新使能接收中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);  //idle interrupt
#endif
#if ( Serial3_Mode == Serial_DMA_Mode )
    MA_UART_Receive_DMA_Init(&huart3, &hdma_usart3_rx, (uint8_t *)&(Serial_Cmd.Serial3.Data[0][1]), (uint8_t *)&(Serial_Cmd.Serial3.Data[1][1]), Serial3_Buffer_Size);
#endif
	
#if ( Serial4_Mode == Serial_NORMAL_Mode )
		HAL_UART_Receive_IT(&huart4,&Serial4_Ctrl.receive_RXNE,1);   // 重新使能接收中断
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);  //idle interrupt
#endif
#if ( Serial4_Mode == Serial_DMA_Mode )
    MA_UART_Receive_DMA_Init(&huart4, &hdma_uart4_rx, (uint8_t *)&(Serial_Cmd.Serial4.Data[0][1]), (uint8_t *)&(Serial_Cmd.Serial4.Data[1][1]), Serial4_Buffer_Size);
#endif

#if ( Serial7_Mode == Serial_NORMAL_Mode )
		HAL_UART_Receive_IT(&huart7,&Serial7_Ctrl.receive_RXNE,1);   // 重新使能接收中断
    __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);  //idle interrupt
#endif
#if ( Serial7_Mode == Serial_DMA_Mode )
    MA_UART_Receive_DMA_Init(&huart7, &hdma_uart7_rx, (uint8_t *)&(Serial_Cmd.Serial7.Data[0][1]), (uint8_t *)&(Serial_Cmd.Serial7.Data[1][1]), Serial7_Buffer_Size);
#endif

#if ( Serial8_Mode == Serial_NORMAL_Mode )
		HAL_UART_Receive_IT(&huart8,&Serial8_Ctrl.receive_RXNE,1);   // 重新使能接收中断
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);  //idle interrupt
#endif
#if ( Serial8_Mode == Serial_DMA_Mode )
    MA_UART_Receive_DMA_Init(&huart8, &hdma_uart8_rx, (uint8_t *)&(Serial_Cmd.Serial8.Data[0][1]), (uint8_t *)&(Serial_Cmd.Serial8.Data[1][1]), Serial8_Buffer_Size);
#endif
	
    Serial1_Ctrl.attachInterrupt(Serial1_Hook);
    Serial3_Ctrl.attachInterrupt(Serial3_Hook);
    Serial4_Ctrl.attachInterrupt(Serial4_Hook);
    Serial7_Ctrl.attachInterrupt(Serial7_Hook);
		Serial8_Ctrl.attachInterrupt(Serial8_Hook);
}

void Serial_Ctrl::Hook(USART_TypeDef *SERIAL, bool mode)
{
    if (SERIAL == SERIAL1)
    {
        Handle(&Serial1_Ctrl, &Serial1, mode);
    }
    if (SERIAL == SERIAL3)
    {
        Handle(&Serial3_Ctrl, &Serial3, mode);
    }
    if (SERIAL == SERIAL4)
    {
        Handle(&Serial4_Ctrl, &Serial4, mode);
    }
    if (SERIAL == SERIAL7)
    {
        Handle(&Serial7_Ctrl, &Serial7, mode);
    }
		if (SERIAL == SERIAL8)
    {
        Handle(&Serial8_Ctrl, &Serial8, mode);
    }
}

void Serial_Ctrl::Handle(Serialctrl *SerialCtrl, Serial_Data_t *Serial, bool mode)
{
		if (Serial->Mode == Serial_NORMAL_Mode)
		{
			if(mode == 0)
			{
        Serial->Temp = SerialCtrl->peek();
//				if(Serial == &Serial7)
//        {
//            if(Serial->Temp == 0xA5)
//						{
//							if(judge1==0)
//							{
//								judge1=1;
//								judgedata[judgedatalen] = SerialCtrl->read();
//								judgedatalen++;
//							}
//							else if(judge1==1)
//							{
//								Serial->Data[0][0] = judgedatalen;
//								for(uint8_t i = 0; i < judgedatalen; i++)
//								{
//									Serial->Data[0][i + 1] = judgedata[i];
//								}
//								Send_to_Message(SerialCtrl,0);
//								judgedatalen=0;
//								memset(judgedata,0,sizeof(judgedata));
//								judgedata[judgedatalen] = SerialCtrl->read();
//								judgedatalen++;
//							}
//						}
//						else
//						{
//							judgedata[judgedatalen] = SerialCtrl->read();
//							judgedatalen++;
//						}
//            return;
//        }
				if(Serial->Header != NULL && Serial->Temp != Serial->Header)
        {
            SerialCtrl->read();
            return;
        }
									if(Serial->Len == Serial->buffer_size - 1)
				{
					for(uint8_t i = 0; i < Serial->Len; i++)
					{
            SerialCtrl->read();
					}
				}
			}
			if (mode == 1)
			{		
				Serial->Len = SerialCtrl->available();
				if((Serial->Len == Serial->Lenth0 ||Serial->Len == Serial->Lenth1 ||Serial->Len == Serial->Lenth2 ||Serial->Len == Serial->Lenth3) && (Serial->Len != NULL))
        {
					Serial->Data[0][0] = Serial->Len;
					for(uint8_t i = 0; i < Serial->Len; i++)
					{
            Serial->Data[0][i + 1] = SerialCtrl->read();
					}
					if(Serial->Tail != NULL && Serial->Data[0][Serial->Len] != Serial->Tail)
					{
            Serial->Data[0][0] = 0;
					}
			 	Serial->Len = SerialCtrl->available();
					if(Serial->Data[0][0] != 0)
					{
            Send_to_Message(SerialCtrl,0);
					}
				}
				else if(Serial->Lenth0 == NULL ||Serial->Lenth1 == NULL ||Serial->Lenth2 == NULL ||Serial->Lenth3 == NULL)
        {
					Serial->Data[0][0] = Serial->Len;
					if( SerialCtrl->peek() == 0xA5 )
					{
						for(uint8_t i = 0; i < Serial->Len; i++)
						{
							Serial->Data[0][i + 1] = SerialCtrl->read();
						}
						if(Serial->Data[0][0] != 0)
						{
							Send_to_Message(SerialCtrl,0);
						}
					}
//		Serial->Len = SerialCtrl->available();
				}
				else
				{
					for(uint8_t i = 0; i < Serial->Len; i++)
					{
            SerialCtrl->read();
					}
				}
				if(Serial->Len == Serial->buffer_size - 1)
				{
					for(uint8_t i = 0; i < Serial->Len; i++)
					{
            SerialCtrl->read();
					}
				}
			}
		}
    if (Serial->Mode == Serial_DMA_Mode)
    {
			if (mode == 1)
			{
        bool Memory;
        /* Current memory buffer used is Memory 0 */
        // disable DMA
        // 失效DMA
        __HAL_DMA_DISABLE(SerialCtrl->hdma_usart_rx);

        // get receive data length, length = set_data_length - remain_length
        // 获取接收数据长度,长度 = 设定长度 - 剩余长度
        Serial->Len = Serial->buffer_size - ((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->NDTR;

        // reset set_data_lenght
        // 重新设定数据长度
        ((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->NDTR = Serial->buffer_size;

        if ((((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->CR & DMA_SxCR_CT) == RESET)
        {
            // set memory buffer 1
            // 设定缓冲区1
            ((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->CR |= DMA_SxCR_CT;
            Memory = 0;
        }
        else
        {
            // set memory buffer 0
            // 设定缓冲区0
            ((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->CR &= ~(DMA_SxCR_CT);
            Memory = 1;
        }
				
				
		// enable DMA
        // 使能DMA
        __HAL_DMA_ENABLE(SerialCtrl->hdma_usart_rx);
				
				
        if ((Serial->Len == Serial->Lenth0 ||Serial->Len == Serial->Lenth1 ||Serial->Len == Serial->Lenth2 ||Serial->Len == Serial->Lenth3) && (Serial->Len != NULL))
        {
            Serial->Data[Memory][0] = Serial->Len;
						if(Serial->Header != NULL && Serial->Header != Serial->Data[Memory][1])
						{
								Serial->Len = 0;
								return;
						}
						if(Serial->Tail != NULL && Serial->Tail != Serial->Data[Memory][Serial->Len])
						{
								Serial->Len = 0;
								return;
						}
            if (Serial->Len != 0)
            {
                Send_to_Message(SerialCtrl, Memory);
            }
        }
			}
							if(Serial->Len == Serial->buffer_size - 1)
				{
					for(uint8_t i = 0; i < Serial->Len; i++)
					{
            SerialCtrl->read();
					}
				}
    }
}


uint8_t Serial_Ctrl::Get_Data(Serial_Data_t *Serial, uint8_t *buf)
{
    if (Serial->Len == 0)
    {
        return 0;
    }
    buf = Serial->Data[0];
    return Serial->Len;
}
//达妙移植改串口
void Serial_Ctrl::Send_to_Message(Serialctrl *SerialCtrl, bool Memory)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (SerialCtrl == &Serial1_Ctrl)
    {
        ID_Data[SerialData1].Data_Ptr = Serial1.Data[Memory];
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData1], &xHigherPriorityTaskWoken);
    }
    if (SerialCtrl == &Serial3_Ctrl)
    {
        ID_Data[SerialData3].Data_Ptr = Serial3.Data[Memory];
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData3], &xHigherPriorityTaskWoken);
    }
    if (SerialCtrl == &Serial4_Ctrl)
    {
        ID_Data[SerialData4].Data_Ptr = Serial4.Data[Memory];
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData4], &xHigherPriorityTaskWoken);
    }
    if (SerialCtrl == &Serial7_Ctrl)
    {
        ID_Data[SerialData7].Data_Ptr = Serial7.Data[Memory];
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData7], &xHigherPriorityTaskWoken);
    }
		if (SerialCtrl == &Serial8_Ctrl)
    {
        ID_Data[SerialData8].Data_Ptr = Serial8.Data[Memory];
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData8], &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
