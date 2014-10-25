#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H
#include "stm32f10x.h"

#define RED_L  	0xf800		//这里与摄像头的颜色冲突，故加以改变。
#define GREEN_L	0x07e0
#define BLUE_L 	0x001f
#define WHITE	0xffff
#define BLACK	0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D   	//灰色0 3165 00110 001011 00101
#define GRAY1   0x8410      	//灰色1      00000 000000 00000
#define GRAY2   0x4208      	//灰色2  1111111111011111


#define LCD_CTRL   	  	GPIOA				//定义TFT数据端口为PA组
#define LCD_CON         GPIOE				//定义TFT控制端口为PE组
#define LCD_LED        	GPIO_Pin_11  //MCU_PA11   		对应接液晶屏(或者模块)TFT --PIN_LED背光正极（背光可以由IO口提供电流，或者外接3.3V电压）
#define LCD_CS        	GPIO_Pin_0 //MCU_PE0			对应接液晶屏(或者模块)TFT --CS
#define LCD_SCL        	GPIO_Pin_5	//MCU_PA5			对应接液晶屏(或者模块)TFT --SCL
#define LCD_SDA        	GPIO_Pin_7	//MCU_PA7 MOSI	对应接液晶屏(或者模块)TFT --SDA 
#define LCD_SDO        	GPIO_Pin_6	//MCU_PA6 MISO	对应接液晶屏(或者模块)TFT --SDO 
#define LCD_RS         	GPIO_Pin_1	//MCU_PE1			对应接液晶屏(或者模块)TFT --RS/DC
#define LCD_RST     	  GPIO_Pin_4	//MCU_PA4			对应接液晶屏(或者模块)TFT --RST



//#define LCD_CS_SET(x) LCD_CTRL->ODR=(LCD_CTRL->ODR&~LCD_CS)|(x ? LCD_CS:0)

//液晶控制口置1操作语句宏定义
#define	LCD_CS_SET  	LCD_CON->BSRR=LCD_CS    
#define	LCD_RS_SET  	LCD_CON->BSRR=LCD_RS    
#define	LCD_SDA_SET  	LCD_CTRL->BSRR=LCD_SDA    
#define	LCD_SCL_SET  	LCD_CTRL->BSRR=LCD_SCL    
#define	LCD_RST_SET  	LCD_CTRL->BSRR=LCD_RST    
#define	LCD_LED_SET  	LCD_CTRL->BSRR=LCD_LED    

//液晶控制口置0操作语句宏定义
#define	LCD_CS_CLR  	LCD_CON->BRR=LCD_CS    
#define	LCD_RS_CLR  	LCD_CON->BRR=LCD_RS    
#define	LCD_SDA_CLR  	LCD_CTRL->BRR=LCD_SDA    
#define	LCD_SCL_CLR  	LCD_CTRL->BRR=LCD_SCL    
#define	LCD_RST_CLR  	LCD_CTRL->BRR=LCD_RST    
#define	LCD_LED_CLR  	LCD_CTRL->BRR=LCD_LED  


#define LCD_DATAOUT(x) LCD_DATA->ODR=x; //数据输出
#define LCD_DATAIN     LCD_DATA->IDR;   //数据输入

#define LCD_WR_DATA(data){\
LCD_RS_SET;\
LCD_CS_CLR;\
LCD_DATAOUT(data);\
LCD_WR_CLR;\
LCD_WR_SET;\
LCD_CS_SET;\
} 



//void LCD_GPIO_Init(void);
void Lcd_WriteIndex(u8 Index);
void Lcd_WriteData(u8 Data);
void Lcd_WriteReg(u8 Index,u8 Data);
u16 Lcd_ReadReg(u8 LCD_Reg);
void Lcd_Reset(void);
void Lcd_Init(void);
void Lcd_Clear(u16 Color);
void Lcd_SetXY(u16 x,u16 y);
void Gui_DrawPoint(u16 x,u16 y,u16 Data);
unsigned int Lcd_ReadPoint(u16 x,u16 y);
void Lcd_WriteData16Bit(u8 DataH,u8 DataL);

#endif
