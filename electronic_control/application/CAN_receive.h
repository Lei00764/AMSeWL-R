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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define SHOOT_CAN hcan1
#define GIMBAL_CAN hcan1
#define GIMBAL_YAW_CAN hcan2
#define CHASSIS_COM hcan1
#define CHASSIS_CONTROLLER hcan2

 #define P_MIN -3.14f
 #define P_MAX 3.14f
 #define V_MIN -30.0f
 #define V_MAX 30.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -10.0f
 #define T_MAX 10.0f
 
 #define Speed_Min 0.f
 #define Speed_Max 40.f
 
 
 #define RPMTORAD 0.10471976

typedef struct
{
		float x_data;
		float y_data;
		float z_data;
	
}cmd_vel_t;

/* CAN send and receive ID */
typedef enum
{
	//底盘八个电机ID
		CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_6020_M1_ID = 0x205,
    CAN_6020_M2_ID = 0x206,
    CAN_6020_M3_ID = 0x207,
	  CAN_6020_M4_ID = 0x208,

		CAN_CHASSIS_REFEREE = 0x11F,

} can_msg_id_e;

typedef enum
{
	//云台八个电机ID
CAN_CHASSIS_1 = 0x201,
CAN_CHASSIS_2 = 0x202,
CAN_CHASSIS_3 = 0x203,
CAN_CHASSIS_4 = 0x204,
CAN_GIMBAL_YAW = 0x205,
CAN_GIMBAL_PITCH = 0x206,
UP_DOWN_1 = 0x207,
UP_DOWN_2 = 0x208,

} can2_msg_id_e;


//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;


//新电机接受数据
/*
POS 表示电机的位置信息
VEL 表示电机的速度信息
T 表示电机的扭矩信息
T_MOS 表示驱动上 MOS 的平均温度，单位℃
T_Rotor 表示电机内部线圈的平均温度，单位℃
*/
typedef struct
{
	uint16_t id;
	uint16_t state;
	uint16_t p_int;
	int16_t v_int;
	uint16_t t_int;
	int16_t kp_int;
	int16_t kd_int;
	float ecd;			//电机的角度：弧度
	float speed_rpm;
	float speed_rad;
	float toq;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
  float last_ecd;
} motor_measure_t_new;

extern void send_yaw_to_chassis(float yaw_ecd);

/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */


/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

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
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_gimbal(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

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
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
  * @brief          返回底盘航向6020电机数据指针
  * @param[in]      i: 电机编号,范围[4,7]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_angel_measure_point(uint8_t i);

extern const cmd_vel_t *get_uppercom_point(void);//返回上位机指针

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor1_measure_point(void);
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor2_measure_point(void);

/**
  * @brief          返回摩擦电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_fricl_motor1_measure_point(void);
const motor_measure_t *get_fricr_motor1_measure_point(void);
const motor_measure_t *get_fricl_motor2_measure_point(void);
const motor_measure_t *get_fricr_motor2_measure_point(void);




#endif
