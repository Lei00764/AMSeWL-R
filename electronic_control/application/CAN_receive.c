/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "bsp_can.h"
#include "can.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);     \
        (ptr)->temperate = (data)[6];                                   \
    }

		int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

// 达妙电机数据接收	
	

void get_motor_measure_new(motor_measure_t_new* mtr,uint8_t Data[8])                                \
    {                   	
				(mtr)->last_ecd = mtr->ecd; 
				(mtr)->id = ((Data)[0])&0x0F; 
				(mtr)->state = ((Data)[0])>>4; 
			  (mtr)->p_int=((Data)[1]<<8)| (Data)[2]; 
				(mtr)->v_int=((Data)[3]<<4)|((Data)[4]>>4); 
				(mtr)->t_int=(((Data)[4]&0xF)<<8)|(Data)[5]; 
				(mtr)->ecd = uint_to_float(mtr->p_int, P_MIN, P_MAX, 16);  
				(mtr)->speed_rpm = uint_to_float(mtr->v_int, V_MIN, V_MAX, 12);   
				(mtr)->toq = uint_to_float(mtr->t_int, T_MIN, T_MAX, 12);   
				(mtr)->Tmos = (float)((Data)[6]); 
				(mtr)->Tcoil = (float)((Data)[7]); 
			if(((mtr)->ecd-(mtr)->last_ecd)*1000<50 && ((mtr)->ecd-(mtr)->last_ecd)*1000>-50)
			{
				(mtr)->ecd = (mtr)->last_ecd;
			}
				(mtr)->speed_rad = (mtr)->speed_rpm * RPMTORAD;
		}
/*
 * motor_chassis[0:3] 从0到3分别对应3508_M1至3508_M4
 * motor_chassis[4:7] 从4到7分别对应6020_M1至6020_M4
*/
static motor_measure_t motor_chassis[8];
static motor_measure_t_new motor_gimbal_yaw;
static motor_measure_t_new motor_gimbal_pitch;

/*
	motor_gimbal[0:1] [yaw,pitch]
	motor_gimbal[2:5]	四个摩擦轮M3508
	motor_gimbal[6:7]	两个波单轮6002
*/
static motor_measure_t motor_gimbal[8];

static CAN_TxHeaderTypeDef  chassis_angel_tx_message;
static uint8_t              chassis_angel_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              chassis_can_send_data[8];
static uint8_t              gimbal_can_send_data[8];
static uint8_t       				shoot_can_send_data[6];
static	cmd_vel_t cmd_vel;

static CAN_TxHeaderTypeDef  chassis_controller_tx_message;
static uint8_t              chassis_controller_can_send_data[8];


		

		
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
extern gimbal_control_t gimbal_control;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
		extern gimbal_control_t gimbal_control; 

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if(hcan == &hcan2)
	{

		}
	if(hcan == &hcan1)
	{
		switch (rx_header.StdId)
		{
				case CAN_CHASSIS_1:
				{
				static uint8_t i = 0;
				get_motor_measure(&motor_chassis[i], rx_data);
						break;
				}
			case CAN_CHASSIS_2:
			{
				static uint8_t i = 1;
				get_motor_measure(&motor_chassis[i], rx_data);
				break;
			}
			case CAN_CHASSIS_3:
			{
				static uint8_t i = 2;
				get_motor_measure(&motor_chassis[i], rx_data);
				break;
			}
			case CAN_CHASSIS_4:
			{
				static uint8_t i = 3;
				get_motor_measure(&motor_chassis[i], rx_data);
				break;
			}
			case CAN_GIMBAL_YAW:
			{
				static uint8_t i = 4;
				get_motor_measure(&motor_chassis[i], rx_data);
				break;
			}
				case CAN_GIMBAL_PITCH:
			{
				static uint8_t i = 5;
				get_motor_measure(&motor_chassis[i], rx_data);
		if(motor_chassis[5].ecd - motor_chassis[5].last_ecd < -7000)
		{
			gimbal_control.gimbal_3508_count ++;
		}
		if(motor_chassis[5].ecd - motor_chassis[5].last_ecd > 7000)
		{
			gimbal_control.gimbal_3508_count --;
		}
		gimbal_control.gimbal_pitch_motor.relative_angle = (8192 * gimbal_control.gimbal_3508_count + motor_chassis[5].ecd - gimbal_control.gimbal_pitch_motor.offset_ecd);
		gimbal_control.gimbal_pitch_motor.relative_angle = gimbal_control.gimbal_pitch_motor.relative_angle / 4096 * 3.1415 / 19;
				break;
			}
			case UP_DOWN_1:
			{
				static uint8_t i = 6;
				get_motor_measure(&motor_chassis[i], rx_data);
		if(motor_chassis[6].ecd - motor_chassis[6].last_ecd < -4096)
		{
			gimbal_control.gimbal_updown_count1 ++;
		}
		else if(motor_chassis[6].ecd - motor_chassis[6].last_ecd > 4096)
		{
			gimbal_control.gimbal_updown_count1 --;
		}
		
		gimbal_control.gimbal_updown_motor1.relative_angle = (8192 * gimbal_control.gimbal_updown_count1 + gimbal_control.gimbal_updown_motor1.gimbal_motor_measure->ecd - gimbal_control.gimbal_updown_motor1.offset_ecd);
		gimbal_control.gimbal_updown_motor1.relative_angle = gimbal_control.gimbal_updown_motor1.relative_angle / 4096 * 3.14 / 36;
				break;
			}
			case UP_DOWN_2:
			{
				static uint8_t i = 7;
				get_motor_measure(&motor_chassis[i], rx_data);
				break;
			}
		}
		
	}
}




void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_gimbal(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x1ff;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = motor1 >> 8;
    gimbal_can_send_data[1] = motor1;
    gimbal_can_send_data[2] = motor2 >> 8;
    gimbal_can_send_data[3] = motor2;
    gimbal_can_send_data[4] = motor3 >> 8;
    gimbal_can_send_data[5] = motor3;
    gimbal_can_send_data[6] = motor4 >> 8;
    gimbal_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

int float_2_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }





/**
  * @brief          返回底盘航向6020电机数据指针
  * @param[in]      范围[4,7]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_angel_measure_point(uint8_t i)
{
    return &motor_chassis[i];
}



/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[i];
}

/**
  * @brief          返回上位机数据指针
  * @param[in]      
  * @retval         
  */
const cmd_vel_t *get_uppercom_point(void)
{
    return &cmd_vel;
}


/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}


/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor1_measure_point(void)
{
    return &motor_chassis[6];
}

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor2_measure_point(void)
{
    return &motor_chassis[7];
}

/**
  * @brief          返回摩擦电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_fricl_motor1_measure_point(void)
{
    return &motor_gimbal[2];
}

const motor_measure_t *get_fricr_motor1_measure_point(void)
{
    return &motor_gimbal[3];
}

const motor_measure_t *get_fricl_motor2_measure_point(void)
{
    return &motor_gimbal[4];
}

const motor_measure_t *get_fricr_motor2_measure_point(void)
{
    return &motor_gimbal[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */

