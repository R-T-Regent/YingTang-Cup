#include "bsp_usart.h"

 void USART_Config()
 {
      GPIO_InitTypeDef GPIO_InitStructure;
      USART_InitTypeDef USART_InitStructure;
      NVIC_InitTypeDef NVIC_InitStructure;    

	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

	   /*GPIO_Configuate*/
	   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

	   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	   GPIO_Init(GPIOA, &GPIO_InitStructure);

	   /*USART_Configuratr*/
	   USART_InitStructure.USART_BaudRate = 19200;
	   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	   USART_InitStructure.USART_StopBits = USART_StopBits_1;
	   USART_InitStructure.USART_Parity =USART_Parity_No;
	   USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_Init(USART1, &USART_InitStructure);

	    
      
 #ifdef EN_USART1_RX  
   
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
 #endif   
      
      USART_Cmd(USART1, ENABLE);                       

 }

 int fputc(int ch, FILE *f)
 {
     USART_SendData(USART1, (unsigned char)ch);
	   while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
	 
	   return ch;
 }
