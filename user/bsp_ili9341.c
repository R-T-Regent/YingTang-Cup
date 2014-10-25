//********************************************************************************
//默认程序接线说明：
#define LCD_CTRL   	  	GPIOA				//定义TFT数据端口为PA组
#define LCD_LED        	GPIO_Pin_11  //MCU_PA11   		对应接液晶屏(或者模块)TFT --PIN_LED背光正极（背光可以由IO口提供电流，或者外接3.3V电压）
#define LCD_CS        	GPIO_Pin_0 //MCU_PE0			对应接液晶屏(或者模块)TFT --CS
#define LCD_SCL        	GPIO_Pin_5	//MCU_PA5			对应接液晶屏(或者模块)TFT --SCL
#define LCD_SDA        	GPIO_Pin_7	//MCU_PA7 MOSI	对应接液晶屏(或者模块)TFT --SDA 
#define LCD_SDO        	GPIO_Pin_6	//MCU_PA6 MISO	对应接液晶屏(或者模块)TFT --SDO 
#define LCD_RS         	GPIO_Pin_1	//MCU_PE1			对应接液晶屏(或者模块)TFT --RS/DC
#define LCD_RST     	  GPIO_Pin_4	//MCU_PA4			对应接液晶屏(或者模块)TFT --RST
//********************************************************************************


#include "stm32f10x.h"
#include "bsp_ili9341.h"
#include "bsp_ili9341_Config.h"
#include "delay.h"

//spi 写一个字节
u8 SPI_WriteByte(SPI_TypeDef* SPIx,u8 Byte)
{
	while((SPIx->SR&SPI_I2S_FLAG_TXE)==RESET);		//等待发送区空	  
	SPIx->DR=Byte;	 	//发送一个byte   
	while((SPIx->SR&SPI_I2S_FLAG_RXNE)==RESET);//等待接收完一个byte  
	return SPIx->DR;          	     //返回收到的数据			
} 

//设置SPI的速度
//SpeedSet:1,高速;0,低速;
void SPI_SetSpeed(SPI_TypeDef* SPIx,u8 SpeedSet)
{
	SPIx->CR1&=0XFFC7;
	if(SpeedSet==1)//高速
	{
		SPIx->CR1|=SPI_BaudRatePrescaler_2;//Fsck=Fpclk/2	
	}
	else//低速
	{
		SPIx->CR1|=SPI_BaudRatePrescaler_32; //Fsck=Fpclk/32
	}
	SPIx->CR1|=1<<6; //SPI设备使能
} 

/****************************************************************************
* 名    称：void ili9220B_WriteIndex(u16 idx)
* 功    能：写 ili9220B 控制器寄存器地址
* 入口参数：idx   寄存器地址
* 出口参数：无
* 说    明：调用前需先选中控制器，内部函数
****************************************************************************/
void Lcd_WriteIndex(u8 Index)
{
   //SPI 写命令时序开始
   //LCD_CS_CLR;
   LCD_RS_CLR;
   //SPIv_WriteByte(Index);
   SPI_WriteByte(SPI1,Index);
   
   //LCD_CS_SET;
}

/****************************************************************************
* 名    称：void ili9220B_WriteData(u16 dat)
* 功    能：写 ili9220B 寄存器数据
* 入口参数：dat     寄存器数据
* 出口参数：无
* 说    明：向控制器指定地址写入数据，调用前需先写寄存器地址，内部函数
****************************************************************************/
void Lcd_WriteData(u8 Data)
{
   //LCD_CS_CLR;
   LCD_RS_SET;
   //SPIv_WriteByte(Data);
   SPI_WriteByte(SPI1,Data);

   //LCD_CS_SET; 
}

void Lcd_WriteData16Bit(u8 DataH,u8 DataL)
{
	Lcd_WriteData(DataH);
	Lcd_WriteData(DataL);
}

void Lcd_WriteIndex16Bit(u8 DataH,u8 DataL)
{
	Lcd_WriteIndex(DataH);
	Lcd_WriteIndex(DataL);
}



void Lcd_Reset(void)
{
	LCD_RST_CLR;
	delay_ms(100);
	LCD_RST_SET;
	delay_ms(50);
}


void Lcd_Init(void)
{
	//SPIv_Init();
//	SPI1_Init();
	Lcd_Reset();
	
	Lcd_WriteIndex(0xCB);  
        Lcd_WriteData(0x39); 
        Lcd_WriteData(0x2C); 
        Lcd_WriteData(0x00); 
        Lcd_WriteData(0x34); 
        Lcd_WriteData(0x02); 

        Lcd_WriteIndex(0xCF);  
        Lcd_WriteData(0x00); 
	Lcd_WriteData(0x81);
//        Lcd_WriteData(0XC1); 
        Lcd_WriteData(0X30); 
 
        Lcd_WriteIndex(0xE8);  
        Lcd_WriteData(0x85); 
        Lcd_WriteData(0x00); 
        Lcd_WriteData(0x78); 
 
        Lcd_WriteIndex(0xEA);  
        Lcd_WriteData(0x00); 
        Lcd_WriteData(0x00); 
 
        Lcd_WriteIndex(0xED);  
        Lcd_WriteData(0x64); 
        Lcd_WriteData(0x03); 
        Lcd_WriteData(0X12); 
        Lcd_WriteData(0X81); 
/* Pump ratio control (F7h) */
        Lcd_WriteIndex(0xF7);  
        Lcd_WriteData(0x20); 
	/* Frame Rate Control (In Normal Mode/Full Colors) (B1h) */
	Lcd_WriteIndex(0xB1);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x1B);
	
	
        Lcd_WriteIndex(0xC0);    //Power control 
        Lcd_WriteData(0x23);   //VRH[5:0] 
 
        Lcd_WriteIndex(0xC1);    //Power control 
        Lcd_WriteData(0x10);   //SAP[2:0];BT[3:0] 
 
        Lcd_WriteIndex(0xC5);    //VCM control 
        Lcd_WriteData(0x3e); //对比度调节
        Lcd_WriteData(0x28); 
 
        Lcd_WriteIndex(0xC7);    //VCM control2 
        Lcd_WriteData(0x86);  //--
 
        Lcd_WriteIndex(0x36);    // Memory Access Control 
#ifdef H_VIEW
        Lcd_WriteData(0xE8); //C8	   //48 68竖屏//28 E8 横屏
#else
				Lcd_WriteData(0x48); 
#endif

        Lcd_WriteIndex(0x3A);    
        Lcd_WriteData(0x55); 

        Lcd_WriteIndex(0xB1);    
        Lcd_WriteData(0x00);  
        Lcd_WriteData(0x18); 
 
        Lcd_WriteIndex(0xB6);    // Display Function Control 
//        Lcd_WriteData(0x08); 
//        Lcd_WriteData(0x82);
//        Lcd_WriteData(0x27);  
 	Lcd_WriteData(0x0A);
	Lcd_WriteData(0xA2);
	
	
        Lcd_WriteIndex(0xF2);    // 3Gamma Function Disable 
        Lcd_WriteData(0x00); 
 
        Lcd_WriteIndex(0x26);    //Gamma curve selected 
        Lcd_WriteData(0x01); 
 
        Lcd_WriteIndex(0xE0);    //Set Gamma 
        Lcd_WriteData(0x0F); 
        Lcd_WriteData(0x31); 
        Lcd_WriteData(0x2B); 
        Lcd_WriteData(0x0C); 
        Lcd_WriteData(0x0E); 
        Lcd_WriteData(0x08); 
        Lcd_WriteData(0x4E); 
        Lcd_WriteData(0xF1); 
        Lcd_WriteData(0x37); 
        Lcd_WriteData(0x07); 
        Lcd_WriteData(0x10); 
        Lcd_WriteData(0x03); 
        Lcd_WriteData(0x0E); 
        Lcd_WriteData(0x09); 
        Lcd_WriteData(0x00); 

        Lcd_WriteIndex(0XE1);    //Set Gamma 
        Lcd_WriteData(0x00); 
        Lcd_WriteData(0x0E); 
        Lcd_WriteData(0x14); 
        Lcd_WriteData(0x03); 
        Lcd_WriteData(0x11); 
        Lcd_WriteData(0x07); 
        Lcd_WriteData(0x31); 
        Lcd_WriteData(0xC1); 
        Lcd_WriteData(0x48); 
        Lcd_WriteData(0x08); 
        Lcd_WriteData(0x0F); 
        Lcd_WriteData(0x0C); 
        Lcd_WriteData(0x31); 
        Lcd_WriteData(0x36); 
        Lcd_WriteData(0x0F); 
		
		//	/* 设置液晶扫描方向为 左下角->右上角 */
	Lcd_WriteIndex(0x36); 
	Lcd_WriteData(0x68);	
	Lcd_WriteIndex(0X2A); 
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x01);
	Lcd_WriteData(0x3F);	

	Lcd_WriteIndex(0X2B); 
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0xEF);
	
        Lcd_WriteIndex(0x11);    //Exit Sleep 
        delay_ms(120); 
				
        Lcd_WriteIndex(0x29);    //Display on 
       Lcd_WriteIndex(0x2c); 

}


/*************************************************
函数名：LCD_Set_Region
功能：设置lcd显示区域，在此区域写点数据自动换行
入口参数：xy起点和终点,Y_IncMode表示先自增y再自增x
返回值：无
*************************************************/
void Lcd_SetRegion(u16 x_start,u16 y_start,u16 x_end,u16 y_end)
{	
	Lcd_WriteIndex(0x2a);
	Lcd_WriteData16Bit(x_start>>8,x_start);
	Lcd_WriteData16Bit(x_end>>8,x_end);
	Lcd_WriteIndex(0x2b);
	Lcd_WriteData16Bit(y_start>>8,y_start);
	Lcd_WriteData16Bit(y_end>>8,y_end);
	Lcd_WriteIndex(0x2c);

}

/*************************************************
函数名：LCD_Set_XY
功能：设置lcd显示起始点
入口参数：xy坐标
返回值：无
*************************************************/
void Lcd_SetXY(u16 x,u16 y)
{
  	Lcd_WriteIndex(0x2a);
	Lcd_WriteData16Bit(x>>8,x);
	Lcd_WriteIndex(0x2b);
	Lcd_WriteData16Bit(y>>8,y);

	Lcd_WriteIndex(0x2c);
}

	
/*************************************************
函数名：LCD_DrawPoint
功能：画一个点
入口参数：无
返回值：无
*************************************************/
void Gui_DrawPoint(u16 x,u16 y,u16 Data)
{
	Lcd_SetXY(x,y);
	Lcd_WriteData(Data>>8);
	Lcd_WriteData(Data);

}    

/*****************************************
 函数功能：读TFT某一点的颜色                          
 出口参数：color  点颜色值                                 
******************************************/
unsigned int Lcd_ReadPoint(u16 x,u16 y)
{
  unsigned int Data;
  Lcd_SetXY(x,y);

  //Lcd_ReadData();//丢掉无用字节
  //Data=Lcd_ReadData();
  Lcd_WriteData(Data);
  return Data;
}
/*************************************************
函数名：Lcd_Clear
功能：全屏清屏函数
入口参数：填充颜色COLOR
返回值：无
*************************************************/
void Lcd_Clear(u16 Color)               
{	
   unsigned int i,m;
   Lcd_SetRegion(0,0,X_MAX_PIXEL-1,Y_MAX_PIXEL-1);
   //LCD_CS_CLR;
   //LCD_RS_SET;
   LCD_RS_SET;
   //SPIv_WriteByte(Data);

   
   for(i=0;i<Y_MAX_PIXEL;i++)
   {
    for(m=0;m<X_MAX_PIXEL;m++)
      {	 
	  	//SPIv_WriteByte(Color>>8);  
		//SPIv_WriteByte(Color);
		SPI_WriteByte(SPI1,Color>>8);
		SPI_WriteByte(SPI1,Color);
	  	//Lcd_WriteData16Bit(Color>>8,Color);
		//Lcd_WriteData(Color>>8);
		//Lcd_WriteData(Color);
      }   
	}
	 // LCD_CS_SET;  
}
