/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.���̹��ʿ���
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             ֻ����80w���ʣ���Ҫͨ�����Ƶ�������趨ֵ,������ƹ�����40w������
  *             JUDGE_TOTAL_CURRENT_LIMIT��POWER_CURRENT_LIMIT��ֵ�����е�������ٶ�
  *             (����max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f



/**
  * @brief          ����Ŀ��ֵ��С
  * @param[in]      para: Ŀ��ֵ min����Сֵ max�����ֵ
  * @retval         ���ƺ��ֵ
  */
float out_Limit(float para, float min, float max)
{
	if(min<=max)
	{
		return para < min ? min : (para > max ? max : para);
	}
	else
	{
		return para < max ? max : (para > min ? min : para);
	}
}


/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
  * @param[in]      chassis_power_control: ��������
  * @retval         none
  */
fp32 wheeltes = 0.0f;
void chassis_power_control(chassis_move_t* chassis_power_control)
{
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
//    fp32 total_current_limit = 0.0f;
//    fp32 total_current = 0.0f;
    uint8_t robot_id = get_robot_id();
    fp32 speed_set_ori;//δ���Ƶ�Ŀ��ת��
    fp32 speed_lim = 0;//ת������


    //fp32 max_speed_dif = 0;//maxΪ���ת�ٲ�
    //fp32 set_speed_sum = 0;//���ƺ�Ŀ��ת�ٺ�
    //fp32 speed_set_sum_lim;//���ƺ�Ŀ��ת�ٺͣ�ԭ������
    //fp32 speed_set_sum_min = 0; //���ƺ���СĿ��ת�ٺͣ�ԭ������
    //speed_set_ori = chassis_power_control->motor_chassis.speed_set;
    if(toe_is_error(REFEREE_TOE))
    {

    }
    else
    {
        get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);//��û�������

        //max_speed_dif = fabs(chassis_power_control->motor_chassis.speed - chassis_power_control->motor_chassis.speed_set);//ת�ٲ�

        //speed_lim= 50*pow(5,chassis_power_buffer/30);//ת������
			
			
        speed_lim = 0.2 * sqrt(chassis_power_buffer);


    }
}
		

	

