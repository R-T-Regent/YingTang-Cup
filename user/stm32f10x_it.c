/**
  ******************************************************************************
  * @file GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and 
  *         peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "bsp_ov7725.h"
#include "bsp_sdcard.h"
#include "bsp_usart.h"

/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char USART_RX_BUF [USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
uint16_t USART_RX_STA = 0; 
extern volatile uint8_t Ov7725_vsync;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
/*void PPP_IRQHandler(void)
{
}*/

void SDIO_IRQHandler(void) 
{
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
}

/* ov7725 场中断 服务程序 */
void EXTI0_IRQHandler(void)
{
    if ( EXTI_GetITStatus(EXTI_Line0) != RESET ) 	//检查EXTI_Line0线路上的中断请求是否发送到了NVIC 
    {
        if( Ov7725_vsync == 0 )
        {
            FIFO_WRST_L(); 	                      //拉低使FIFO写(数据from摄像头)指针复位
            FIFO_WE_H();	                        //拉高使FIFO写允许
            
            Ov7725_vsync = 1;	   	
            FIFO_WE_H();                          //使FIFO写允许
            FIFO_WRST_H();                        //允许使FIFO写(数据from摄像头)指针运动
        }
        else if( Ov7725_vsync == 1 )
        {
            FIFO_WE_L();                          //拉低使FIFO写暂停
            Ov7725_vsync = 2;
        }
        EXTI_ClearITPendingBit(EXTI_Line0);		    //清除EXTI_Line0线路挂起标志位        
    }    
}

 void USART1_IRQHandler()
 {
      char Res=0;     
     
     	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
		{
		  Res =USART_ReceiveData(USART1);
		 
		    if((USART_RX_STA&0x8000)==0)
			{
			    if(USART_RX_STA&0x4000)
				{  
				  if(Res!=0x0a)USART_RX_STA=0;
				  else 
                  {
                    USART_RX_STA |= 0x8000;
				    USART_RX_BUF[USART_RX_STA&0X3FFF] = '\0';                           
                  }
				}
			    else 
				{	
				    if(Res==0x0d)USART_RX_STA|=0x4000;
				    else
					{ 
   				 	   USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
   				 	   USART_RX_STA++;
   					   if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0; 
					}		 
				}
			}   		 
     } 
 }
 
/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
