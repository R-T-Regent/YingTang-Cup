/**
  ******************************************************************************
  * @file    main.c
  * @author  Regent 刘家伦
  * @version V2.3.0
  * @date    20/10/2014
  * @brief   英唐杯主程序。
  ***********本程序占用的系统资源和I/O非常少。**********************************
  ***********可使用更为低级的控制芯片去完成设计与生产，节省成本。***************
  ******************************************************************************
  * @attention 本作品使用Cortex M3 内核的 STM32F103ZET6作为控制芯片。
  * ILI9341作为QVGA驱动芯片。
  * 使用了 硬件SPI与ILI9341通讯，使用了SDIO与SD卡通讯。
  * OV7725作为摄像头驱动。 使用FIFO作为数据缓冲。
  * 挂载了FATFS文件系统。SD卡使用SDIO+DMA加快传输速度。
  * 考虑到程序的简单性，没有挂载操作系统。
  * 减小了数据量，没有使用外部FLASH。
  * 考虑到设计为监控。故没有使用GUI界面，TFT彩屏只是用来显示采集到的视频数据。
  ******************************************************************************
  * @revise    修订日志
  * 2014/08/25 初建工程。移植HAL&SUART_BSP。
  * 2014/09/10 使用FSMC驱动ILI9341。
  * 2014/09/15 废除FSMC驱动ILI9341的程序。
  * 2014/09/20 写入SPI1 SD卡 BSP。
  * 2014/09/25 写入SPI2 ILI9341 BSP。
  * 2014/10/05 移植SPI2到SPI1。
  * 2014/10/08 删除SPI1 SD卡BSP，移植SDIO_BSP。
  *	2014/10/10 加入文件系统FATFS.09。
			   录入AVI文件头及创建AVI相关代码。
  * 2014/10/21 调整OV7725线中断优先级为1。
			   尝试在QVGA屏上显示采集到的图像，分辨率320*240。
			   因为从节约I/O和节约能源的角度去设计，所以速度较慢。
  * 2014/10/22 调整ILI9341的CS、RS从PA9、PA10到PE0、PE1。
  *			   加入USART1串口调试。中断优先级为2。
  * 		   加入过一段时间自动建立新录像机能。
  *            减少采集数据量，加快传输。
  */

//********************************************************************************
//SPI1 ILI9341默认程序接线说明：
#define LCD_CTRL   	  	GPIOA				//定义TFT数据端口为PA组
#define LCD_LED        	GPIO_Pin_11  //MCU_PA11   		对应接液晶屏(或者模块)TFT --PIN_LED背光正极（背光可以由IO口提供电流，或者外接3.3V电压）
#define LCD_CS        	GPIO_Pin_0 //MCU_PE0			对应接液晶屏(或者模块)TFT --CS
#define LCD_SCL        	GPIO_Pin_5	//MCU_PA5			对应接液晶屏(或者模块)TFT --SCL
#define LCD_SDA        	GPIO_Pin_7	//MCU_PA7 MOSI	对应接液晶屏(或者模块)TFT --SDA 
#define LCD_SDO        	GPIO_Pin_6	//MCU_PA6 MISO	对应接液晶屏(或者模块)TFT --SDO 
#define LCD_RS         	GPIO_Pin_1	//MCU_PE1			对应接液晶屏(或者模块)TFT --RS/DC
#define LCD_RST     	  GPIO_Pin_4	//MCU_PA4			对应接液晶屏(或者模块)TFT --RST

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "bsp_ili9341.h"
//#include "bsp_tftGUI.h" //没有使用GUI界面，TFT彩屏只是用来显示采集到的视频数据。
#include "bsp_sdcard.h"
#include "bsp_ov7725.h"
#include "bsp_usart.h"
#include "ff.h"
#include "delay.h"

/* Private define ------------------------------------------------------------*/
#define USART_DEBUG

/**
 * @brief AVI File Ralation.
 */
 unsigned char avi_riff[] = {  
    0x52, 0x49, 0x46, 0x46, 0x10, 0x27, 0x94, 0x04, 0x41, 0x56, 0x49, 0x20, 0x4C, 0x49, 0x53, 0x54,  /*|RIFF.lW.AVI LIST|*/ 
    0xC0, 0x00, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00, /*|....hdrlavih8...|listSize listType avih 结构大小*/
	0x09, 0x2e, 0x02, 0x00, 0x40, 0x19, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x09, 0x00, 0x00, /*|P...............|帧间时间 最大数据率 填充粒度 全局标记*/
	0XC8, 0X00, 0X00, 0X00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0X00, 0X58, 0X02, 0X00, /* |................|总帧数 交互帧数 流个数 建议缓存*/
	0X40, 0X01, 0X00, 0X00, 0XF0, 0X00, 0X00, 0X00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*@...............|width height 保留*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x74, 0x00, 0x00, 0x00, /* |........LISTt@..| LIST listSize */
	0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x38, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73, /*|strlstrh8...vids| strl strh  结构大小 流类型（vids）*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*|................| 流的处理者dwFlags wPriority wLanguage交互帧数*/
	0x01, 0x00, 0x00, 0x00, 0X04, 0X00, 0X00, 0X00, 0x00, 0x00, 0x00, 0x00, 0XC8, 0X00, 0X00, 0X00, /*|d...............| 流时间尺度  采样率  流开始时间  流长*/
	0X00, 0X58, 0X02, 0X00, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0X40, 0X01, 0XF0, 0X00, 0x73, 0x74, 0x72, 0x66, 0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, /*|@...strf(...(...| strf  位图信息头结构大小*/
	0X40, 0X01, 0X00, 0X00, 0XF0, 0X00, 0X00, 0X00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, /*|@...............| 图像宽度  图像高度 目标设备面数(1)&单位像素位数  图像的压缩类型*/
	0X00, 0X58, 0X02, 0X00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* |................| 图像大小(字节)  biXPelsPerMeter；iYPelsPerMeter； biClrUsed  */
	0x00, 0x00, 0x00, 0x00, 0x4A, 0x55, 0x4E, 0x4B, 0x14, 0x10, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, /*JUNK*/
	0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x64, 0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  
  /* 0x4C, 0x49, 0x53, 0x54, 0x44, 0xc6, 0xd4, 0x01, 0x6D, 0x6F, 0x76, 0x69, 
	0x30, 0x30, 0x64, 0x62, 0x00, 0x58, 0x02, 0x00   */
  };

 uint8_t LIST[] = { 0x4C, 0x49, 0x53, 0x54, 0x44, 0xc6, 0xd4, 0x01, 0x6D, 0x6F, 0x76, 0x69, };

 uint8_t _00db[8] = {0x30, 0x30, 0x64, 0x62, 0x00, 0x58, 0x02, 0x00};

 uint8_t junk_0[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,};
 
 uint8_t INFO[] = {0x4C, 0x49, 0x53, 0x54, 0x18, 0x00, 0x00, 0x00, 0x49, 0x4E, 0x46, 0x4F, 0x49, 0x53, 0x46, 0x54,
                   0x0C, 0x00, 0x00, 0x00, 0x4C, 0x61, 0x76, 0x66, 0x35, 0x32, 0x2E, 0x32, 0x34, 0x2E, 0x31, 0x00,  
                   0x4A, 0x55, 0x4E, 0x4B, 0xF8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                   };

 uint8_t buff[38400] = {0};
 
/* Private variables ---------------------------------------------------------*/
/*  FATFS  */
FRESULT res;
FATFS fs;
DIR dirs;
FIL avi;
UINT bw, br;
FILINFO info;
char Video_Name[12]= "Video00.avi";
extern __IO uint8_t Ov7725_vsync ;
__IO uint16_t Frame_Count = 0;

/* Private function prototypes -----------------------------------------------*/
void SPI1_Init(void);
void RCC_Configuration(void);
void Change_VideoName(char *p);
void Image_Full(void);
void RW_19200(uint8_t *);
 
/**
  * @brief Main Function.
 */
 
int main(void)
{
	uint16_t i;
  /* System Clocks Configuration ******************************************* */
	SystemInit();		//Cortex-M3初始化。
	delay_init(72);		//初始化SysTick。
	USART_Config();
	#ifdef USART_DEBUG 
	printf(" USART1 Init Complete!\r\n"); 
	#endif
	SPI1_Init();		//配置SPI1。
	#ifdef USART_DEBUG 
	printf("SPI1 Init Complete! \r\n"); 
	#endif
	SDIO_NVIC_Configuration();		//配置SDIO中断。
	#ifdef USART_DEBUG 
	printf("SDIO NVIC Configuration Complete!\r\n"); 
	#endif
	Lcd_Init();			//向ILI9341送指令，来初始化。
	LCD_LED_SET;		//开TFT背光灯，本次作品的TFT背光灯可以直接用GPIO点亮。
	#ifdef USART_DEBUG 
	printf("QVGA 2.2 Init Complete! \r\n"); 
	#endif
	f_mount(0, &fs);
	#ifdef USART_DEBUG
	printf("Register Work Area OK!\r\n");
	#endif
	Ov7725_GPIO_Config();//ov7725 寄存器配置初始化
	#ifdef USART_DEBUG 
	printf("Ov7725 GPIO Init Complete! \r\n"); 
	#endif
	while(Ov7725_Init() != SUCCESS);
	VSYNC_Init();	//ov7725 场信号线初始化
	Ov7725_vsync = 0;
	/**
	 * @brief 此处为AVI文件创建&初始化阶段。
	 */
	CHANGE_NAME:; //创建AVI文件并检测是否存在，若存在，则改名，另存。
	res = f_open(&avi, Video_Name, FA_CREATE_NEW | FA_WRITE);
	if (res == FR_EXIST)
	{   
		#ifdef USART_DEBUG
		printf("File has already existed! Try Again.. \r\n");
		#endif		
		Change_VideoName(Video_Name);
		f_close(&avi);
		goto CHANGE_NAME;
	}
	else if (res == FR_OK)
	{   
		#ifdef USART_DEBUG
		printf("File create OK! \r\nWriting Header!\r\n");
		#endif
		f_lseek(&avi,0);
		/*riff*/
		res = f_write(&avi, avi_riff, sizeof(avi_riff), &bw);
		/*JUNK*/
		for(i=0; i<256; i++)
		res = f_write(&avi, junk_0, sizeof(junk_0), &bw);
		/*INFO*/ 
		res = f_write(&avi, INFO, sizeof(INFO), &bw);
		for(i=0; i<63; i++)
		res = f_write(&avi, junk_0, sizeof(junk_0), &bw); //JUNK
		/*LIST*/ 
		res = f_write(&avi, LIST, sizeof(LIST), &bw);
		res = f_write(&avi, _00db, sizeof(_00db), &bw);
		#ifdef USART_DEBUG
		printf("AVI File Header Write OK! \r\n");
		#endif
	}
	while(1)
	{ 
		if( Ov7725_vsync == 2 )
		{
			FIFO_PREPARE;  			/* FIFO准备 */	
		/**
		  * @attention 采集FULL Image并送QVGA显示,用于DEBUG 
			Image_Full();			
			#ifdef USART_DEBUG 
			printf("76800 16bits RGB565 has displayed.\r\n"); 
			#endif
		**/
			for(i = 0; i < 4; i++)
			{
				 RW_19200(buff);		
				 f_write(&avi, buff, sizeof(buff), &bw);
			}
			res = f_write(&avi, _00db, sizeof(_00db), &bw);
			Frame_Count++; //采集一帧
			#ifdef USART_DEBUG 
			printf("One Frame Writed. \r\n"); 
			#endif
			if (Frame_Count == 20)
			{
				Frame_Count = 0;
				#ifdef USART_DEBUG
				printf("20 Frames has been writed to an AVI file.Will create a new file.\r\n");
				#endif
				f_close(&avi);
				Ov7725_vsync = 0;
				goto CHANGE_NAME;		//再次采集，这里为了起到演示的效果，将单文件采集到的帧数大幅度的降低了。
			}
			Ov7725_vsync = 0;
		}
	}
}


/**
 * @brief Display the Video with 320 * 240.
 * @attention
 *		320
 * -------------------
 *|                   |
 *|                   |
 *|                   |  240
 *|                   |
 *|                   |
 * -------------------
 **/
void Image_Full(void)
{
	uint16_t i, j;
	uint16_t Camera_Data;
	for(i = 0; i < 240; i++)
	{
		for(j = 0; j < 320; j++)
		{
			READ_FIFO_PIXEL(Camera_Data);		/* 从FIFO读出一个rgb565像素到Camera_Data变量 */		
			Lcd_WriteData16Bit(Camera_Data >> 8,Camera_Data);
//			f_write(&avi, &Camera_Data, sizeof(Camera_Data), &bw); 
		}
	}
}


void RCC_Configuration(void)
{   
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
}

void SPI1_Init(void)	
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	 
	//配置SPI1管脚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOE /* | RCC_APB2Periph_GPIOD */, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;    
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//SPI1配置选项
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 ,ENABLE);
	   
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	//使能SPI1
	SPI_Cmd(SPI1, ENABLE);   
}

/**
 * @brief Display the Video with 160 * 120.
 *		160
 * -------------------
 *|                   |
 *|                   |
 *|                   |  120
 *|                   |
 *|                   |
 * -------------------
 **/
void RW_19200(uint8_t * p)
{
	uint16_t i;
	for(i=0; i<38400; i+=2)
	{
		FIFO_RCLK_L();
		*(p+i+1) = (uint8_t)(((GPIOB->IDR) & 0xff00) >> 8);  
		FIFO_RCLK_H();
		FIFO_RCLK_L(); 
		*(p+i) = (uint8_t)((GPIOB->IDR >> 8) & 0x00ff); 
		FIFO_RCLK_H();
		Lcd_WriteData16Bit(((uint16_t)((* (p+i+1)) << 8)+(*(p+i))) >> 8,(uint16_t)((* (p+i+1)) << 8)+(*(p+i)));
	}
}

/**
* @brief:Change Video name.
**/
void Change_VideoName(char *p)
{
	*(p+6) += 1;
	if(*(p+6) == ':')
	{
		*(p+6) = '0';
		*(p+5) += 1;
	}
}

/**
  * @brief Easy Delay.
  * @retval : None
  */
/*
void Delayms(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}
*/


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */



void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/* Test & Debug Left */
//BYTE buffer[4096]={0};       		  /* file copy buffer */
//BYTE textFileBuffer[] = "Hello FATFS ver .09! This test file was designed by Regent!";
//u16 ID=0;
//	res = f_open(&avi, "test.txt", FA_CREATE_NEW | FA_WRITE);
//	if ( res == FR_OK )
//	{
//		res = f_write(&avi, textFileBuffer, sizeof(textFileBuffer), &bw);
//		f_close(&avi);      
//	}
//	res = f_open(&avi, "0:newfile.txt", FA_OPEN_EXISTING | FA_READ); 	 
//	res = f_read(&avi, buffer, sizeof(buffer), &br); 

//	printf("\r\n %s ", buffer);
//	
//	/* Close open files */
//	f_close(&avi);	                                      
//	/* Unregister work area prior to discard it */
//	f_mount(0, NULL);

/************************ COPYRIGHT 2014 Regent *****END OF FILE****/
