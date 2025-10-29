#include "dev_serial.h"
#include "app_preference.h"

Serialctrl Serial1_Ctrl(&huart1, &hdma_usart1_rx, Serial1_Buffer_Size, Serial1_Mode);
Serialctrl Serial3_Ctrl(&huart3, &hdma_usart3_rx, Serial3_Buffer_Size, Serial3_Mode);
Serialctrl Serial4_Ctrl(&huart4, &hdma_uart4_rx ,Serial4_Buffer_Size, Serial4_Mode);
Serialctrl Serial7_Ctrl(&huart7, &hdma_uart7_rx ,Serial7_Buffer_Size, Serial7_Mode);
Serialctrl Serial8_Ctrl(&huart8, &hdma_uart8_rx ,Serial8_Buffer_Size, Serial8_Mode);

Serialctrl::Serialctrl(UART_HandleTypeDef *_huartx, DMA_HandleTypeDef * hdma_usart_rx , uint32_t BufferSize, uint8_t Serial_Mode)
{
    this->huartx = _huartx;
		this->hdma_usart_rx = hdma_usart_rx;
		this->Serial_Mode = Serial_Mode;
    USART_Function = 0;
		if(this->Serial_Mode == Serial_NORMAL_Mode)
		{
    newBuffer(&_rx_buffer, BufferSize);
		}
}

void Serialctrl::attachInterrupt(USART_CallbackFunction_t Function)
{
    USART_Function = Function;
}

void Serialctrl::IRQHandler_RXNE(uint8_t c)
{
        Buffer_Write(&_rx_buffer, c);
        if(USART_Function)
        {
            USART_Function(0);
        }
}

void Serialctrl::IRQHandler_IDLE(void)
{
        if(USART_Function)
        {
            USART_Function(1);
        }
}

void Serialctrl::sendData(uint8_t ch)
{
//    USART_SendData(this->USARTx, ch);
//    while(USART_GetFlagStatus(this->USARTx, USART_FLAG_TXE) == RESET);
//	HAL_UART_Transmit(this->huartx, &ch, 1, 99);
//	while(!( __HAL_UART_GET_FLAG(this->huartx,UART_FLAG_TC)==SET)){osDelay(1);};
	while(!( __HAL_UART_GET_FLAG(this->huartx,UART_FLAG_TC)==SET)){ osDelay(1); };
		HAL_UART_Transmit_DMA(this->huartx, &ch, 1);
}

void Serialctrl::sendData(const void *str)
{
    unsigned int k = 0;
    do
    {
        sendData(*((uint8_t *)str + k));
        k++;
    } while(*((uint8_t *)str + k) != '\0');
//    while(USART_GetFlagStatus(this->USARTx, USART_FLAG_TC) == RESET) {}
}

void Serialctrl::sendData(const void *buf, uint8_t len)
{
    uint8_t *ch = (uint8_t *)buf;
//    while(len--)
//    {
//        sendData(*ch++);
//    }
		while(!( __HAL_UART_GET_FLAG(this->huartx,UART_FLAG_TC)==SET)){osDelay(1);};
		HAL_UART_Transmit_DMA(this->huartx, ch, len);
}

int Serialctrl::available(void)
{
    return ((unsigned int)(_rx_buffer.buf_size + _rx_buffer.pw - _rx_buffer.pr)) % _rx_buffer.buf_size;
}

uint8_t Serialctrl::read(void)
{
    uint8_t c = 0;
    Buffer_Read(&_rx_buffer, &c);
    return c;
}

int Serialctrl::peek(void)
{
    if(_rx_buffer.pr == _rx_buffer.pw)
    {
        return -1;
    }
    else
    {
        return _rx_buffer.fifo[_rx_buffer.pr];
    }
}

void Serialctrl::flush(void)
{
    _rx_buffer.pr = _rx_buffer.pw;
}

extern "C"{

//串口接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1){
        
		Serial1_Ctrl.IRQHandler_RXNE(Serial1_Ctrl.receive_RXNE);
    HAL_UART_Receive_IT(&huart1, &Serial1_Ctrl.receive_RXNE, 1);    // 继续使能RX中断
        
    }
		if(huart->Instance == USART3){
        
		Serial3_Ctrl.IRQHandler_RXNE(Serial3_Ctrl.receive_RXNE);
    HAL_UART_Receive_IT(&huart3, &Serial3_Ctrl.receive_RXNE, 1);    // 继续使能RX中断
        
    }
		if(huart->Instance == UART4){
        
		Serial4_Ctrl.IRQHandler_RXNE(Serial4_Ctrl.receive_RXNE);
    HAL_UART_Receive_IT(&huart4, &Serial4_Ctrl.receive_RXNE, 1);    // 继续使能RX中断
        
    }if(huart->Instance == UART7){
        
		Serial7_Ctrl.IRQHandler_RXNE(Serial7_Ctrl.receive_RXNE);
    HAL_UART_Receive_IT(&huart7, &Serial7_Ctrl.receive_RXNE, 1);    // 继续使能RX中断
        
    }if(huart->Instance == UART8){
        
		Serial8_Ctrl.IRQHandler_RXNE(Serial8_Ctrl.receive_RXNE);
    HAL_UART_Receive_IT(&huart8, &Serial8_Ctrl.receive_RXNE, 1);    // 继续使能RX中断
        
    }
}
//串口接收空闲中断回调函数，这个是从hal里面新加的
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1){  
        
//    Serial1_Ctrl.receive_IDLE = huart1.Instance->RDR;
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
//		__HAL_DMA_DISABLE(&hdma_usart1_rx);
		Serial1_Ctrl.IRQHandler_IDLE(); 
//		__HAL_DMA_ENABLE(&hdma_usart1_rx);
    }
		if(huart->Instance == USART3){  
        
//    Serial1_Ctrl.receive_IDLE = huart1.Instance->RDR;
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
//		__HAL_DMA_DISABLE(&hdma_usart2_rx);
		Serial3_Ctrl.IRQHandler_IDLE(); 
//		__HAL_DMA_ENABLE(&hdma_usart2_rx);
    }
		if(huart->Instance == UART4){  
        
//    Serial1_Ctrl.receive_IDLE = huart1.Instance->RDR;
		__HAL_UART_CLEAR_IDLEFLAG(&huart4);
//		__HAL_DMA_DISABLE(&hdma_usart2_rx);
		Serial4_Ctrl.IRQHandler_IDLE(); 
//		__HAL_DMA_ENABLE(&hdma_usart2_rx);
    }
		if(huart->Instance == UART7){  
        
//    Serial1_Ctrl.receive_IDLE = huart1.Instance->RDR;
		__HAL_UART_CLEAR_IDLEFLAG(&huart7);
//		__HAL_DMA_DISABLE(&hdma_usart2_rx);
		Serial7_Ctrl.IRQHandler_IDLE(); 
//		__HAL_DMA_ENABLE(&hdma_usart2_rx);
    }
		if(huart->Instance == UART8){  
        
//    Serial1_Ctrl.receive_IDLE = huart1.Instance->RDR;
		__HAL_UART_CLEAR_IDLEFLAG(&huart8);
//		__HAL_DMA_DISABLE(&hdma_usart2_rx);
		Serial8_Ctrl.IRQHandler_IDLE(); 
//		__HAL_DMA_ENABLE(&hdma_usart2_rx);
    }
}

//串口接收错误中断
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(HAL_UART_GetError(huart) & HAL_UART_ERROR_PE){		/*!< Parity error            */
		//奇偶校验错误
		__HAL_UART_CLEAR_PEFLAG(huart);
	}else if(HAL_UART_GetError(huart) & HAL_UART_ERROR_NE){ /*!< Noise error             */
		//噪声错误
		__HAL_UART_CLEAR_NEFLAG(huart);
	}else if(HAL_UART_GetError(huart) & HAL_UART_ERROR_FE){ /*!< Frame error             */
		//帧格式错误
		__HAL_UART_CLEAR_FEFLAG(huart);
	}else if(HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE){ /*!< Overrun error           */
		//数据太多串口来不及接收错误
		__HAL_UART_CLEAR_OREFLAG(huart);
	}
    //当这个串口发生了错误，一定要在重新使能接收中断
    if(huart ->Instance == USART1){
		HAL_UART_Receive_IT(&huart1, &Serial1_Ctrl.receive_RXNE, 1);
	}
		if(huart ->Instance == USART3){
		HAL_UART_Receive_IT(&huart3, &Serial3_Ctrl.receive_RXNE, 1);
	}
		if(huart ->Instance == UART4){
		HAL_UART_Receive_IT(&huart4, &Serial4_Ctrl.receive_RXNE, 1);
	}
		if(huart ->Instance == UART7){
		HAL_UART_Receive_IT(&huart7, &Serial7_Ctrl.receive_RXNE, 1);
	}
		if(huart ->Instance == UART8){
		HAL_UART_Receive_IT(&huart8, &Serial8_Ctrl.receive_RXNE, 1);
	}
    //其他串口......
}

}
