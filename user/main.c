/**
  ******************************************************************************
  * @file    main.c
  * @author  Regent ������
  * @version V2.3.0
  * @date    20/10/2014
  * @brief   Ӣ�Ʊ�������
  ***********������ռ�õ�ϵͳ��Դ��I/O�ǳ��١�**********************************
  ***********��ʹ�ø�Ϊ�ͼ��Ŀ���оƬȥ����������������ʡ�ɱ���***************
  ******************************************************************************
  * @attention ����Ʒʹ��Cortex M3 �ں˵� STM32F103ZET6��Ϊ����оƬ��
  * ILI9341��ΪQVGA����оƬ��
  * ʹ���� Ӳ��SPI��ILI9341ͨѶ��ʹ����SDIO��SD��ͨѶ��
  * OV7725��Ϊ����ͷ������ ʹ��FIFO��Ϊ���ݻ��塣
  * ������FATFS�ļ�ϵͳ��SD��ʹ��SDIO+DMA�ӿ촫���ٶȡ�
  * ���ǵ�����ļ��ԣ�û�й��ز���ϵͳ��
  * ��С����������û��ʹ���ⲿFLASH��
  * ���ǵ����Ϊ��ء���û��ʹ��GUI���棬TFT����ֻ��������ʾ�ɼ�������Ƶ���ݡ�
  ******************************************************************************
  * @revise    �޶���־
  * 2014/08/25 �������̡���ֲHAL&SUART_BSP��
  * 2014/09/10 ʹ��FSMC����ILI9341��
  * 2014/09/15 �ϳ�FSMC����ILI9341�ĳ���
  * 2014/09/20 д��SPI1 SD�� BSP��
  * 2014/09/25 д��SPI2 ILI9341 BSP��
  * 2014/10/05 ��ֲSPI2��SPI1��
  * 2014/10/08 ɾ��SPI1 SD��BSP����ֲSDIO_BSP��
  *	2014/10/10 �����ļ�ϵͳFATFS.09��
			   ¼��AVI�ļ�ͷ������AVI��ش��롣
  * 2014/10/21 ����OV7725���ж����ȼ�Ϊ1��
			   ������QVGA������ʾ�ɼ�����ͼ�񣬷ֱ���320*240��
			   ��Ϊ�ӽ�ԼI/O�ͽ�Լ��Դ�ĽǶ�ȥ��ƣ������ٶȽ�����
  * 2014/10/22 ����ILI9341��CS��RS��PA9��PA10��PE0��PE1��
  *			   ����USART1���ڵ��ԡ��ж����ȼ�Ϊ2��
  * 		   �����һ��ʱ���Զ�������¼����ܡ�
  *            ���ٲɼ����������ӿ촫�䡣
  */

//********************************************************************************
//SPI1 ILI9341Ĭ�ϳ������˵����
#define LCD_CTRL   	  	GPIOA				//����TFT���ݶ˿�ΪPA��
#define LCD_LED        	GPIO_Pin_11  //MCU_PA11   		��Ӧ��Һ����(����ģ��)TFT --PIN_LED�������������������IO���ṩ�������������3.3V��ѹ��
#define LCD_CS        	GPIO_Pin_0 //MCU_PE0			��Ӧ��Һ����(����ģ��)TFT --CS
#define LCD_SCL        	GPIO_Pin_5	//MCU_PA5			��Ӧ��Һ����(����ģ��)TFT --SCL
#define LCD_SDA        	GPIO_Pin_7	//MCU_PA7 MOSI	��Ӧ��Һ����(����ģ��)TFT --SDA 
#define LCD_SDO        	GPIO_Pin_6	//MCU_PA6 MISO	��Ӧ��Һ����(����ģ��)TFT --SDO 
#define LCD_RS         	GPIO_Pin_1	//MCU_PE1			��Ӧ��Һ����(����ģ��)TFT --RS/DC
#define LCD_RST     	  GPIO_Pin_4	//MCU_PA4			��Ӧ��Һ����(����ģ��)TFT --RST

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "bsp_ili9341.h"
//#include "bsp_tftGUI.h" //û��ʹ��GUI���棬TFT����ֻ��������ʾ�ɼ�������Ƶ���ݡ�
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
    0xC0, 0x00, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00, /*|....hdrlavih8...|listSize listType avih �ṹ��С*/
	0x09, 0x2e, 0x02, 0x00, 0x40, 0x19, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x09, 0x00, 0x00, /*|P...............|֡��ʱ�� ��������� ������� ȫ�ֱ��*/
	0XC8, 0X00, 0X00, 0X00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0X00, 0X58, 0X02, 0X00, /* |................|��֡�� ����֡�� ������ ���黺��*/
	0X40, 0X01, 0X00, 0X00, 0XF0, 0X00, 0X00, 0X00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*@...............|width height ����*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x74, 0x00, 0x00, 0x00, /* |........LISTt@..| LIST listSize */
	0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x38, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73, /*|strlstrh8...vids| strl strh  �ṹ��С �����ͣ�vids��*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*|................| ���Ĵ�����dwFlags wPriority wLanguage����֡��*/
	0x01, 0x00, 0x00, 0x00, 0X04, 0X00, 0X00, 0X00, 0x00, 0x00, 0x00, 0x00, 0XC8, 0X00, 0X00, 0X00, /*|d...............| ��ʱ��߶�  ������  ����ʼʱ��  ����*/
	0X00, 0X58, 0X02, 0X00, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0X40, 0X01, 0XF0, 0X00, 0x73, 0x74, 0x72, 0x66, 0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, /*|@...strf(...(...| strf  λͼ��Ϣͷ�ṹ��С*/
	0X40, 0X01, 0X00, 0X00, 0XF0, 0X00, 0X00, 0X00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, /*|@...............| ͼ����  ͼ��߶� Ŀ���豸����(1)&��λ����λ��  ͼ���ѹ������*/
	0X00, 0X58, 0X02, 0X00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* |................| ͼ���С(�ֽ�)  biXPelsPerMeter��iYPelsPerMeter�� biClrUsed  */
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
	SystemInit();		//Cortex-M3��ʼ����
	delay_init(72);		//��ʼ��SysTick��
	USART_Config();
	#ifdef USART_DEBUG 
	printf(" USART1 Init Complete!\r\n"); 
	#endif
	SPI1_Init();		//����SPI1��
	#ifdef USART_DEBUG 
	printf("SPI1 Init Complete! \r\n"); 
	#endif
	SDIO_NVIC_Configuration();		//����SDIO�жϡ�
	#ifdef USART_DEBUG 
	printf("SDIO NVIC Configuration Complete!\r\n"); 
	#endif
	Lcd_Init();			//��ILI9341��ָ�����ʼ����
	LCD_LED_SET;		//��TFT����ƣ�������Ʒ��TFT����ƿ���ֱ����GPIO������
	#ifdef USART_DEBUG 
	printf("QVGA 2.2 Init Complete! \r\n"); 
	#endif
	f_mount(0, &fs);
	#ifdef USART_DEBUG
	printf("Register Work Area OK!\r\n");
	#endif
	Ov7725_GPIO_Config();//ov7725 �Ĵ������ó�ʼ��
	#ifdef USART_DEBUG 
	printf("Ov7725 GPIO Init Complete! \r\n"); 
	#endif
	while(Ov7725_Init() != SUCCESS);
	VSYNC_Init();	//ov7725 ���ź��߳�ʼ��
	Ov7725_vsync = 0;
	/**
	 * @brief �˴�ΪAVI�ļ�����&��ʼ���׶Ρ�
	 */
	CHANGE_NAME:; //����AVI�ļ�������Ƿ���ڣ������ڣ����������档
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
			FIFO_PREPARE;  			/* FIFO׼�� */	
		/**
		  * @attention �ɼ�FULL Image����QVGA��ʾ,����DEBUG 
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
			Frame_Count++; //�ɼ�һ֡
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
				goto CHANGE_NAME;		//�ٴβɼ�������Ϊ������ʾ��Ч���������ļ��ɼ�����֡������ȵĽ����ˡ�
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
			READ_FIFO_PIXEL(Camera_Data);		/* ��FIFO����һ��rgb565���ص�Camera_Data���� */		
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
	 
	//����SPI1�ܽ�
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
	//SPI1����ѡ��
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

	//ʹ��SPI1
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
