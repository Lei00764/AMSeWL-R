/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"

#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500

#define COVER_OPEN_PWM 500
#define COVER_CLOSE_PWM 2000

#define SERVO_COVER_KEY  KEY_PRESSED_OFFSET_R

const RC_ctrl_t *servo_rc;
covermoni cover;

/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ¶æ»úÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();

    while(1)
    {
//        //if((servo_rc->key.v & KEY_PRESSED_OFFSET_SHIFT) == KEY_PRESSED_OFFSET_SHIFT){
//				if(switch_is_mid(servo_rc->rc.s[0])){
//					if(cover == cover_off){
//						cover = cover_on;
//						servo_pwm_set(COVER_OPEN_PWM,3);
//						break;
//					}
//					else if(cover == cover_on){
//						cover = cover_off;
//						servo_pwm_set(COVER_CLOSE_PWM,3);
//						break;
//					}
//				}
//				else{
//					break;
//				}
        osDelay(10);
    }
}


