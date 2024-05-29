/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//in the beginning of task ,wait a time
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//the channel num of controlling vertial speed 
//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//the channel num of controlling horizontal speed
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
#define CHASSIS_GUNLUN_CHANNEL 4

#define CHASSIS_RACE_SPEED 1.0f
#define MAX_CHASSIS_SPEED 1.1f

//in some mode, can use remote control to control rotation speed
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

//the channel of choosing chassis mode,
//ѡ�����״̬ ����ͨ����
#define CHASSIS_MODE_CHANNEL 0//0right 1left

#define LEFT_SWTICH 1//0right 1left
#define RIGHT_SWTICH 0//0right 1left

//rocker value (max 660) change to vertial speed (m/s)
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.002f
//rocker value (max 660) change to horizontal speed (m/s)
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.002f
//in following yaw angle mode, rocker value add to angle
//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//in not following yaw angle mode, rocker value change to rotation speed
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//rocker value deadline
//ҡ������
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


#define MOTOR_DISTANCE_TO_CENTER 0.2f

//chassis task control time  2ms
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//chassis control frequence, no use now.
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//chassis 3508 max motor control current
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//press the key, chassis will swing
//����ҡ�ڰ���
#define SWING_LEFT_KEY KEY_PRESSED_OFFSET_Q
#define SWING_RIGHT_KEY KEY_PRESSED_OFFSET_E
//chassi forward, back, left, right key
//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508 rmp change to chassis speed,
//m3508ת���ɵ����ٶ�(m/s)�ı�����
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//single chassis motor max speed
//�������̵������ٶ�
#define NORMAL_MAX_CHASSIS_SPEED 1.0f
//chassis forward or back max speed
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//chassis left or right max speed
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

#define CHASSIS_WZ_SET_SCALE 0.1f

//when chassis is not set to move, swing max angle
//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//when chassis is set to move, swing max angle
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//chassis motor speed PID
//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 5000.0f

//chassis follow angle PID
//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 40.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
	CHASSIS_RACE_MODE, 
	CHASSIS_DOWN,			//down
	CHASSIS_RAW_MODE,	//ң����ֵ����
  CHASSIS_ESCAPE,   //����ģʽ����δд���ȷ�����
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef enum
{
	STEP1,
	STEP2,
	STEP3,
	STEP4,
	STEP5,
	STEP6,
	STEP7,
	STEP8,
} chassis_move_mode;

typedef enum
{
	READY,
	ONGOING,
	WAIT,
} chassis_auto_condition;



typedef struct
{
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��, the point to remote control
	chassis_mode_e chassis_mode;               //state machine. ���̿���״̬��
  chassis_mode_e last_chassis_mode;          //last state machine.�����ϴο���״̬��
  chassis_motor_t motor_chassis[4];          //chassis motor data.���̵������
  pid_type_def motor_speed_pid[4];             //motor speed PID.���̵���ٶ�pid
	
	uint8_t auto_move_flag;
	uint16_t auto_move_count;
	chassis_move_mode auto_move_mode;
	chassis_auto_condition auto_move_condition; 
	fp32 vx_set;
	fp32 vy_set;
	fp32 wz_set;
	fp32 max_speed;
	fp32 min_speed;
} chassis_move_t;





/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

extern chassis_move_t chassis_move;

#endif
