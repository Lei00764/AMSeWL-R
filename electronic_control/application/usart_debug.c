/*
* @Author: RM_2022_CQD
* @Date:   2020-10-21 13:17:43
* @Last Modified by:   RM
* @Last Modified time: 2021-11-17 20:47:03
*/
#include "usart_debug.h"
#include "struct_typedef.h"
#include "usart.h"
#include "bsp_usart.h"
#include "stdarg.h"
#include <stdio.h>

#define uart8datasize 16
uint8_t u8_sendbuffer[uart8datasize];

void UART_DMA_SEND(int data){
	int temp;
	//u8 buffer[16];
	int i=0;
	int len;
	for(i=0;i<uart8datasize;i++){
		u8_sendbuffer[i]=0x00;
	}
	i=0;
	
	/*----------ASCII转换-----------*/
	if(data>=0){
		while(data/10!=0){
		temp=data-data/10*10;
		u8_sendbuffer[i]=temp+'0';
		i++;
		data/=10;
		}
		u8_sendbuffer[i]=data+'0';
		i++;
		u8_sendbuffer[i]=':';
		i++;
		u8_sendbuffer[i]='l';
		i++;
				u8_sendbuffer[i]='{';
		len=i+1;
	}
	else{
		int neg_data=-data;
		while(neg_data/10!=0){
			temp=neg_data-neg_data/10*10;
			u8_sendbuffer[i]=temp+'0';
			i++;
			neg_data/=10;
		}
		u8_sendbuffer[i]=neg_data+'0';
		i++;
		u8_sendbuffer[i]='-';
		i++;
		u8_sendbuffer[i]=':';
		i++;
		u8_sendbuffer[i]='l';
		i++;
		u8_sendbuffer[i]='{';
		len=i+1;
	}
	/*----------ASCII转换-----------*/
	
	
	uint8_t temp_char;
	for(i=0;i<len/2;i++){
			temp_char=u8_sendbuffer[i];
			u8_sendbuffer[i]=u8_sendbuffer[len-i-1];
			u8_sendbuffer[len-1-i]=temp_char;
		}
		u8_sendbuffer[len]='}';
		len++;
		u8_sendbuffer[len]='\n';
#ifndef AutoAim		
//		usart1_tx_dma_enable(u8_sendbuffer,uart8datasize);
#endif		
		usart1_tx_dma_enable(u8_sendbuffer,uart8datasize);
}


void usart_printf(const char *fmt,...)
{
 static uint8_t tx_buf[256] = {0};
 static va_list ap;
 static uint16_t len;
 va_start(ap, fmt);
 //return length of string 
 //返回字符串长度
 len = vsprintf((char *)tx_buf, fmt, ap);
 va_end(ap);
 usart1_tx_dma_enable(tx_buf, len);
}


