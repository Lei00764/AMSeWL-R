/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "autoaim.h"


//�Ƿ��л�������Ϊ1û��Ϊ0
#define SLIP_RING 1

//����ṹ���У���ֵ�˲����鳤��
#define PITCH_CURRENT_AVERAGE_FILTER_BUFFER_LENGTH 20

#define AVERAGE_FILTER_BUFFER_LENGTH 80
//yaw�����Ǿ�ֵ�˲����鳤��
#define AVERAGE_FILTER_BUFFER_LENGTH_YAW 80
//pitch�����Ǿ�ֵ�˲����鳤��
#define AVERAGE_FILTER_BUFFER_LENGTH_PITCH 8
//test mode, 0 close, 1 open
//��̨����ģʽ �궨�� 0 Ϊ��ʹ�ò���ģʽ
#define GIMBAL_TEST_MODE 0
#define GIMBAL_SYSTEM_IDENTIFICATION_MODE 0    //��̨��Ҫϵͳ��ʶ��ģʽ
#define GIMBAL_NO_NEED_GAME_START_MODE 0  			//����Ҫ������ʼ��־�ڱ��Ϳ���ִ���Զ�ģʽ�µ����񣬱���ʱ��Ϊ0
#define GIMBAL_VISION_SHOOT_BULLET_MODE 0			//�������ģʽ���Զ�ģʽ����Ҫ������λ����flag�ж��Ƿ�򵯣����󲦸˿��ƴ򵯣��²�һ�λ��д�һ�ε�
#define GIMBAL_NO_AUTO_SEARCH_MODE 0					//��̨���Զ�ģʽ�²����Զ�ɨ��
//ң���������£���̨��������ͨ��ֵ
#define REMOTE_CONTROL_GIMBAL_ON_OR_OFF 4

//ң���������£���̨����������ֵ
#define REMOTE_CONTROL_GIMBAL_NUM -300

//����ƫ��

#define YAW_AUTO_OFFSET 0.036f
#define PITCH_AUTO_OFFSET 0.07f//0.1134380453752182f


/**********************************��̨****************************************/

//yaw speed close-loop PID params, max out and max iout
//yaw �ٶȻ� PID�����Լ� PID���������������

#define YAW_SPEED_PID_KP        10000.0f//2500.0f//3600.0f
#define YAW_SPEED_PID_KI        40.0f//1.5f//20.0f
#define YAW_SPEED_PID_KD        0.0f//0.0f//0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f //30000.0
#define YAW_SPEED_PID_MAX_IOUT  1000.0f


#define YAW_GYRO_ABSOLUTE_PID_KP      10.0f//..22.0f//8.0f//20.76f // 20.0f//26.0f
#define YAW_GYRO_ABSOLUTE_PID_KI       0.0f//..0.08//0.0f// 0.0f//0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD       0.0//..-1.4f//0.15f// 3.4f//0.3f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   8000.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f


#define YAW_SPEED_ENCODE_PID_KP        2.5f//10000.0f//2500.0f//3600.0f
#define YAW_SPEED_ENCODE_PID_KI        0.3f//40.0f//1.5f//20.0f
#define YAW_SPEED_ENCODE_PID_KD        -0.3f//0.0f//0.0f
#define YAW_SPEED_ENCODE_PID_MAX_OUT   10.0f //30000.0
#define YAW_SPEED_ENCODE_PID_MAX_IOUT  1.0f

//yaw encode angle close-loop PID params, max out and max iout
//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP        1.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.5f
#define YAW_ENCODE_RELATIVE_PID_KD        0.32f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   2.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f


//pitch speed close-loop PID params, max out and max iout
//pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_KP        1.2f//1.5f //2.0f//1.0f
#define PITCH_SPEED_PID_KI        0.0f
#define PITCH_SPEED_PID_KD        0.3f//0.0f
#define PITCH_SPEED_PID_MAX_OUT   3.0f
#define PITCH_SPEED_PID_MAX_IOUT  0.0f

#define PITCH_SPEED_ENCODE_PID_KP        1.0f
#define PITCH_SPEED_ENCODE_PID_KI        0.019f
#define PITCH_SPEED_ENCODE_PID_KD        -0.5f
#define PITCH_SPEED_ENCODE_PID_MAX_OUT   1.0f
#define PITCH_SPEED_ENCODE_PID_MAX_IOUT  0.0f


#define PITCH_ENCODE_RELATIVE_PID_KP 2.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 3.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f


#define PITCH_GYRO_ABSOLUTE_PID_KP 15.0f//8.0f//15.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.02f//0.04f//0.02f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.5f//	0.4f//0.1f//0.5f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 4.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.4f


//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 300
//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0


#define MIDDLE_YAW    36694
#define MIDDLE_PITCH  27923 //36046

#define MAX_PITCH 4700     //6200    //11.26�ĵ�û��
#define MIN_PITCH 3900

#define PITCH_TURN  0
#define YAW_TURN    0

/**************************�Զ�����***************************/

//�Զ�����������yaw���Լ�pitch��ɨ������-���ٶ�
#define SEARCH_YAW_SPEED 0.00085f
#define SEARCH_PITCH_SPEED 0.0007f

//��̨����ɨ������У�yaw���ɨ�跶Χ
#define SEARCH_YAW_MAX 1.3f
#define SEARCH_YAW_MIN -1.3f

#define SEARCH_PITCH_HI 0.126647547f
#define SEARCH_PITCH_LO -0.248646989f


//turn speed
//��ͷ��̨�ٶ�
#define TURN_SPEED    0.04f
//���԰�����δʹ��
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//rocker value deadband
//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_DEADBAND   80


//����ң����ģʽ�£�ң����ͨ��ֵ������ֵ֮���ת������ң����ͨ��ֵתΪ��������ֵ��
#define YAW_RC_SEN    -0.0008f
#define PITCH_RC_SEN  -0.0008f //0.005
//���������λ���ٶ�ģʽ��ң��������yaw��pitch������ٶ�rad/s��
#define YAW_MAX_SPEED 1
#define PITCH_MAX_SPEED 5
//���������yaw��pitch��id
#define YAW_MOTOR_ID 0x1
#define PITCH_MOTOR_ID 0x18

#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00015f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 1

//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE_DM  32768
#define ECD_RANGE_DM       65536
//��̨��ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//��̨��ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.002f
#define GIMBAL_INIT_ANGLE_ERROR  0.1f
#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//��̨У׼��ֵ��ʱ�򣬷���ԭʼ����ֵ���Լ���תʱ�䣬ͨ���������ж϶�ת
#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

//�ж�ң�����������ʱ���Լ�ң�����������жϣ�������̨yaw����ֵ�Է�������Ư��
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

//�������ֵת���ɽǶ�ֵ
#ifndef MOTOR_ECD_TO_RAD_DM
#define MOTOR_ECD_TO_RAD_DM 0.00076699 //      2*  PI  /65536
#endif

#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD  0.00076699//      2*  PI  /8192
#endif
#define SWING_LEFT_KEY KEY_PRESSED_OFFSET_Q
#define SWING_RIGHT_KEY KEY_PRESSED_OFFSET_E

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
    GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
		GIMBAL_MOTOR_GYRO_READY,//��������ǿ���ģʽ������down״̬
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
		GIMBAL_MOTOR_AUTO, //Autoaimmode
		GIMBAL_MOTO_DOWN,
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;
		fp32 err_angle;
    fp32 last_err;

    fp32 max_out;
    fp32 max_iout;
    fp32 max_dout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
    fp32 dT;
} gimbal_PID_t;

typedef struct
{
    const motor_measure_t* gimbal_motor_measure;
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
		pid_type_def gimbal_motor_encode_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
		gimbal_motor_mode_e last_remote_mote_mode;  //��һ��ң�����µ�yawģʽ����Ϊ��Ա�������Ա���
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad
	  fp32 max_absolute_angle; //rad
    fp32 min_absolute_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
		fp32 motor_relative_speed;     //�����Ա����ٶ�
		fp32 motor_relative_speed_set;	//�����Ա����ٶ�Ŀ��ֵ
	
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s ���������ǽ����yaw���Խ��ٶ�
    fp32 motor_gyro_set;     //rad/s yaw���Խ��ٶȵ�Ŀ��ֵ
		fp32 motor_last_gyro; 
    fp32 motor_speed;					
	  fp32 absolute_motor_speed;
    fp32 raw_cmd_current;    //
    fp32 current_set;
    fp32 given_current;
		fp32 averrage_gyro_buffer[AVERAGE_FILTER_BUFFER_LENGTH];  //���ƾ�ֵ�˲����飬�����������ٶ��˲�

} gimbal_motor_t;

typedef struct
{
    const motor_measure_t* gimbal_motor_measure;
    pid_type_def gimbal_motor_absolute_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad
	  fp32 max_absolute_angle; //rad
    fp32 min_absolute_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
		fp32 motor_relative_speed;     //�����Ա����ٶ�
		fp32 motor_relative_speed_set;	//�����Ա����ٶ�Ŀ��ֵ

    fp32 raw_cmd_current;    //
    fp32 current_set;
    fp32 given_current;
		int move_flag;
} gimbal_motor_t1;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;


typedef enum
{
	STEP1_,
	STEP2_,
	STEP3_,
	STEP4_,
	STEP5_,
} gimbal_move_mode;

typedef enum
{
	GREADY,
	GONGOING,
	GWAIT,
} gimbal_auto_condition;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
		gimbal_motor_t1 gimbal_updown_motor1;
		gimbal_motor_t1 gimbal_updown_motor2;
    gimbal_step_cali_t gimbal_cali;
		int16_t gimbal_3508_count;
		int16_t gimbal_updown_count1;
		int16_t gimbal_updown_count2;
			uint16_t game_start_count;
		fp32 gimbal_3508_pos;
		uint8_t bump_flag;
		gimbal_move_mode auto_gimbal_mode;
		gimbal_auto_condition auto_gimbal_condition;
		uint16_t auto_gimbal_count;
		int auto_grab_count;
} gimbal_control_t;



/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */
/**
  * @brief          ����yaw �������ָ��
  * @param[in]      none
  * @retval         yaw���ָ��
  */
extern const gimbal_motor_t* get_yaw_motor_point(void);

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
/**
  * @brief          ����pitch �������ָ��
  * @param[in]      none
  * @retval         pitch
  */
extern const gimbal_motor_t* get_pitch_motor_point(void);

/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms)
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          ��̨���񣬼�� GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */

extern void gimbal_task(void const* pvParameters);

/**
  * @brief          gimbal cali calculate, return motor offset encode, max and min relative angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max relative angle
  * @param[out]     min_yaw:yaw min relative angle
  * @param[out]     max_yaw:pitch max relative angle
  * @param[out]     min_yaw:pitch min relative angle
  * @retval         none
  */
/**
  * @brief          ��̨У׼���㣬��У׼��¼����ֵ,��� ��Сֵ����
  * @param[out]     yaw ��ֵ ָ��
  * @param[out]     pitch ��ֵ ָ��
  * @param[out]     yaw �����ԽǶ� ָ��
  * @param[out]     yaw ��С��ԽǶ� ָ��
  * @param[out]     pitch �����ԽǶ� ָ��
  * @param[out]     pitch ��С��ԽǶ� ָ��
  * @retval         ����1 ����ɹ�У׼��ϣ� ����0 ����δУ׼��
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  */
extern bool_t cmd_cali_gimbal_hook(uint16_t* yaw_offset, uint16_t* pitch_offset, fp32* max_yaw, fp32* min_yaw, fp32* max_pitch, fp32* min_pitch);

/**
  * @brief          gimbal cali data, set motor offset encode, max and min relative angle
  * @param[in]      yaw_offse:yaw middle place encode
  * @param[in]      pitch_offset:pitch place encode
  * @param[in]      max_yaw:yaw max relative angle
  * @param[in]      min_yaw:yaw min relative angle
  * @param[in]      max_yaw:pitch max relative angle
  * @param[in]      min_yaw:pitch min relative angle
  * @retval         none
  */
/**
  * @brief          ��̨У׼���ã���У׼����̨��ֵ�Լ���С����е��ԽǶ�
  * @param[in]      yaw_offse:yaw ��ֵ
  * @param[in]      pitch_offset:pitch ��ֵ
  * @param[in]      max_yaw:max_yaw:yaw �����ԽǶ�
  * @param[in]      min_yaw:yaw ��С��ԽǶ�
  * @param[in]      max_yaw:pitch �����ԽǶ�
  * @param[in]      min_yaw:pitch ��С��ԽǶ�
  * @retval         ���ؿ�
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  */

extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern void grab_reset(void);
void up_down_motro_move(fp32 target_angle, gimbal_control_t *gimbal_control, int *flag);

#endif
