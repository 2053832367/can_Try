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
							版本 	v1.3
							移植  Rod、niether
              补充  1. V1.3引入汉字字库
                    2. ss1306的0.91寸屏幕Y轴同样七页（64位），但每8bit的数据中偶数位不显示
******************************************************************************/
#include "stdlib.h"
#include "string.h"
#include "app_oled.h"
#include "app_oledfont.h"
#include "main.h"
//#include "spi.h"

char OLED[200];

//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 			   
/**
  * @brief  向SSD1306写入一个字节
  *         
  * @param  dat:写入的数据，OLED_DATA = 1
  *         cmd:写入的命令，OLED_CMD  = 0
	*
  * @note   时钟上升沿写入数据
  */
void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{ 
  uint8_t i;
		  
	if(cmd)
	  OLED_DC_Set();
	else 
	  OLED_DC_Clr();	
  
  
//  HAL_SPI_Transmit(&hspi1,&dat,1,200);//硬件spi
  
  
	OLED_CS_Clr();//片选使能 //软件spi
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else 
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;   
	}				 		  
	OLED_CS_Set();//片选失能
  
  
	OLED_DC_Set();//默认下次写数据   	  
} 
/**
  * @brief  设置位置
  *         
  * @param  x:写入的x坐标
  *         y:写入的y页坐标
  */
void OLED_Set_Pos(uint8_t x, uint8_t y) 
{ 
  x+=XLeve_began;
	OLED_WR_Byte(0xb0+y,OLED_CMD);// 写y起始位置
	OLED_WR_Byte(((x&0xf0)>>4)|XLevelH,OLED_CMD);//对X的高四位
	OLED_WR_Byte((x&0x0f)|XLevelL,OLED_CMD); //对X的低四位
}   

//开启OLED显示    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}

//关闭OLED显示     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   		

//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void)
{  
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte(((XLeve_began&0xf0)>>4)|XLevelH,OLED_CMD); //设置显示位置—列低地址
		OLED_WR_Byte((XLeve_began&0x0f)|XLevelL,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<Max_Column;n++)OLED_WR_Byte(0,OLED_DATA); 
	} //更新显示
}


//在指定位置显示一个字符,包括部分字符
//x:0~（Max_Column-1）
//y:0-7				 
//size:选择字体 16/8/汉字
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size)
{      	
	uint8_t c=0,i=0,a=0,uh[8]={0},ul[8]={0};	
		c=chr-' ';//得到偏移后的值			
		if(x>Max_Column-1)
      {
        x=0;
        if(size==16)
         y+=2;
        else
         y++;
      }
		if(size ==16)
			{
			for(a=0;a<8;a++)
        {
          for(i=7;i>=4;i--)
          {
            uh[a]=uh[a]+((F8X16[c*16+a]>>i)&0x01);
            ul[a]=ul[a]+((F8X16[c*16+a]>>(i-4))&0x01);
            if(i==4)
            {
              ul[a]=ul[a]<<1;
               uh[a]=uh[a]<<1;
            }
            else
            {
               ul[a]=ul[a]<<2;
               uh[a]=uh[a]<<2;
            }
           
          }
        }
        
        OLED_Set_Pos(x,y);
        
        for(i=0;i<8;i++)
        {
          OLED_WR_Byte(ul[i],OLED_DATA);
        }
			OLED_Set_Pos(x,y+1);
			  for(i=0;i<8;i++)
        {
          OLED_WR_Byte(uh[i],OLED_DATA);
        }
        
      memset(uh,0,sizeof(uh));
      memset(ul,0,sizeof(ul));
        for(a=0;a<8;a++)
        {
        for(i=7;i>=4;i--)
          {
            uh[a]=uh[a]+((F8X16[c*16+a+8]>>i)&0x01);
            ul[a]=ul[a]+((F8X16[c*16+a+8]>>(i-4))&0x01);
            if(i==4)
            {
              ul[a]=ul[a]<<1;
               uh[a]=uh[a]<<1;
            }
            else
            {
               ul[a]=ul[a]<<2;
               uh[a]=uh[a]<<2;
            }
           
          }
        }
          OLED_Set_Pos(x,y+2);
        
        for(i=0;i<8;i++)
        {
          OLED_WR_Byte(ul[i],OLED_DATA);
        }
			OLED_Set_Pos(x,y+3);
			  for(i=0;i<8;i++)
        {
          OLED_WR_Byte(uh[i],OLED_DATA);
        }
        
			}
      
			if(size ==8)
			{	
        for(a=0;a<6;a++)
        {
          for(i=7;i>=4;i--)
          {
            uh[a]=uh[a]+((F6x8[c][a]>>i)&0x01);
            ul[a]=ul[a]+((F6x8[c][a]>>(i-4))&0x01);
            if(i==4)
            {
              ul[a]=ul[a]<<1;
               uh[a]=uh[a]<<1;
            }
            else
            {
               ul[a]=ul[a]<<2;
               uh[a]=uh[a]<<2;
            }
          }
          
        }
				OLED_Set_Pos(x,y);
        
        for(i=0;i<6;i++)
        {
          OLED_WR_Byte(ul[i],OLED_DATA);
        }
			OLED_Set_Pos(x,y+1);
			  for(i=0;i<6;i++)
        {
          OLED_WR_Byte(uh[i],OLED_DATA);
        }
			}
      
      if(size ==0xff)
			{
        c+=' ';
			OLED_Set_Pos(x,y);	
			for(i=0;i<16;i++)
			OLED_WR_Byte(HanZi_u16[c*32+i],OLED_DATA);
			OLED_Set_Pos(x,y+1);
			for(i=0;i<16;i++)
			OLED_WR_Byte(HanZi_u16[c*32+i+16],OLED_DATA);
			}
}

//m^n函数
uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}				  

//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{         	
	uint8_t t,temp,sizeD;
	uint8_t enshow=0;	
 if(size==16)
    sizeD=8;
   else 
    sizeD=6;
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+sizeD*t,y,' ',size);
				continue;
			}else enshow=1; 
		}
	 	OLED_ShowChar(x+sizeD*t,y,temp+'0',size); 
	}
} 
//显示一个字符号串
//x:0~（Max_Column-1）
//y:0-7				 
//size:选择字体 16/8 
void OLED_ShowString(uint8_t x,uint8_t y, uint8_t *p,uint8_t size)
{
	uint8_t j=0,sizeD;
  if(size==16)
    sizeD=8;
   else 
    sizeD=6;
	while (p[j]!='\0')
	{		
    if(x>Max_Column-1)
      {
        x=0;
        if(size==16)
         y+=4;
        else
         y++;
      }

    OLED_ShowChar(x,y,p[j],size);
    x+=sizeD;
			j++;
	}
}
void OLED_ShowHZ(uint8_t x,uint8_t y, uint8_t *p)
{
	uint8_t j=0;
	while (p[j]!='\0')
	{		
    if(x>Max_Column-1)
      {
        x=0;
        y+=2;
      }

    OLED_ShowChar(x,y,p[j],0xff);
    x+=16;
			j++;
	}
}
//初始化SSD1306					    
void OLED_Init(void)
{ 	
  OLED_RST_Set();
	HAL_Delay(100);
	OLED_RST_Clr();
	HAL_Delay(100);
	OLED_RST_Set(); 
					  
	OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
	OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
	OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WR_Byte(0x00,OLED_CMD);//-not offset
	OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
	OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
	OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
	OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
	OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WR_Byte(0x02,OLED_CMD);//
	OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
	OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
	OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
	OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel

	OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/ 
	OLED_Clear();
	OLED_Set_Pos(0,0); 	
}  





