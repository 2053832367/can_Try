/******************************************************************************	 
							OLED 	1.3寸oled驱动程序
              GND  	电源地
              VCC  	接5V或3.3v电源
              D0   	接PD6（SCLK）
              D1   	接PD7（SDIN）
              RES  	接PD4
              DC   	接PD5
              CS   	接PD3               
							用途  修改用于串行模式下界面
							时间 	2021/8/16  
							版本 	v1.2
							移植  Rod、niether
******************************************************************************/
#ifndef __OLED_H
#define __OLED_H			 

#include "main.h"
#include "stdlib.h"	 

//OLED模式设置
//0:4线串行模式
#define OLED_MODE 	0
#define XLeve_began 0x00  //X轴起始位置
#define XLevelL			0x00  //低列地址命令
#define XLevelH			0x10  //高列地址命令
#define Max_Column	128	  //最大列值  //0~（Max_Column-1） //128	或132
#define Max_Row			64		//最大行值  //0~（Max_Row-1）
// Max_Column*Max_Row 分辨率
#define	Brightness	0xFF  //亮度


#define OLED_CMD  	0	//写命令
#define OLED_DATA 	1	//写数据

//-----------------OLED端口定义----------------  					   
#define OLED_CS_Clr()  HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_RESET)//CS
#define OLED_CS_Set()  HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_SET)

#define OLED_RST_Clr() HAL_GPIO_WritePin(RES_GPIO_Port,RES_Pin,GPIO_PIN_RESET)//RES
#define OLED_RST_Set() HAL_GPIO_WritePin(RES_GPIO_Port,RES_Pin,GPIO_PIN_SET)

#define OLED_DC_Clr() HAL_GPIO_WritePin(DC_GPIO_Port,DC_Pin,GPIO_PIN_RESET)//DC
#define OLED_DC_Set() HAL_GPIO_WritePin(DC_GPIO_Port,DC_Pin,GPIO_PIN_SET)

//使用4线串行接口时使用 
#define OLED_SCLK_Clr() HAL_GPIO_WritePin(D0_SCL_GPIO_Port,D0_SCL_Pin,GPIO_PIN_RESET)//CLK
#define OLED_SCLK_Set() HAL_GPIO_WritePin(D0_SCL_GPIO_Port,D0_SCL_Pin,GPIO_PIN_SET)

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(D1_SDA_GPIO_Port,D1_SDA_Pin,GPIO_PIN_RESET)//DIN
#define OLED_SDIN_Set() HAL_GPIO_WritePin(D1_SDA_GPIO_Port,D1_SDA_Pin,GPIO_PIN_SET)

//OLED控制用函数
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y, uint8_t *p,uint8_t size);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowHZ(uint8_t x,uint8_t y, uint8_t *p);

extern char OLED[200];;

#endif  
	 



