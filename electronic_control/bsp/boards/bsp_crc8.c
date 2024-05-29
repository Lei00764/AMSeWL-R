/*
* @Author: TJSP_2022_TY
* @Date:   2021-11-30
* @Last Modified by:   
* @Last Modified time:  
* @brief: ��ֲ��20���ϴ��룬Hal�������ƺ��������Ժ��ڵ����Ż�����
*/

#include "bsp_crc8.h"
extern const uint8_t CRC8_INIT;
extern const uint8_t CRC8_table[256];
uint8_t mycrc8Check(uint8_t *buff,uint16_t len)
{
	uint8_t ucIndex,ucCRC8=(uint8_t)CRC8_INIT;
	while (len--)
	{
		ucIndex = ucCRC8^(*buff++);
		ucCRC8 = CRC8_table[ucIndex];
	}
	return(ucCRC8);
}
