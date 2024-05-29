#ifndef __USART_DEBUG_H__
#define __USART_DEBUG_H__

#include "stm32f4xx_hal.h"

#define USART1_DEBUG 

void UART_DMA_SEND(int data);
void usart_printf(const char *fmt,...);

#endif
