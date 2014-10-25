
#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

#define USART_REC_LEN        7
#define EN_USART1_RX
void USART_Config(void);

#ifdef __cplusplus
}
#endif

#endif
