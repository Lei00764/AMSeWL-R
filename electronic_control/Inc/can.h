/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

//#define SCM_rx_mes supercap_module_receive  //¸Ä¸öÃû
//extern SCM_rx_mes SCM_rx_message;
typedef struct{
	int16_t Current_position;
	int16_t Current_speed;
	
	float Continuous_current_position;
	int16_t Continuous_current_position_filtered;
	
	int16_t Electric_ti;
	int16_t Temparature;

	
}Motor_Feedback;

extern Motor_Feedback CAN1_201;
extern Motor_Feedback CAN1_202;

extern Motor_Feedback CAN2_201;
extern Motor_Feedback CAN2_202;
extern Motor_Feedback CAN2_203;
extern Motor_Feedback CAN2_204;
extern Motor_Feedback CAN2_205;
extern Motor_Feedback CAN2_206;
extern Motor_Feedback CAN2_207;
extern Motor_Feedback CAN2_208;	//Cover
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
