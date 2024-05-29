/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
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


//是否有滑环，有为1没有为0
#define SLIP_RING 1

//电机结构体中，均值滤波数组长度
#define PITCH_CURRENT_AVERAGE_FILTER_BUFFER_LENGTH 20

#define AVERAGE_FILTER_BUFFER_LENGTH 80
//yaw陀螺仪均值滤波数组长度
#define AVERAGE_FILTER_BUFFER_LENGTH_YAW 80
//pitch陀螺仪均值滤波数组长度
#define AVERAGE_FILTER_BUFFER_LENGTH_PITCH 8
//test mode, 0 close, 1 open
//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0
#define GIMBAL_SYSTEM_IDENTIFICATION_MODE 0    //云台需要系统辨识的模式
#define GIMBAL_NO_NEED_GAME_START_MODE 0  			//不需要比赛开始标志哨兵就可以执行自动模式下的任务，比赛时置为0
#define GIMBAL_VISION_SHOOT_BULLET_MODE 0			//自瞄测试模式，自动模式不需要根据上位机的flag判断是否打弹，由左拨杆控制打弹，下拨一次回中打一次弹
#define GIMBAL_NO_AUTO_SEARCH_MODE 0					//云台在自动模式下不会自动扫描
//遥控器控制下，云台解锁滑轮通道值
#define REMOTE_CONTROL_GIMBAL_ON_OR_OFF 4

//遥控器控制下，云台解锁滑轮阈值
#define REMOTE_CONTROL_GIMBAL_NUM -300

//自瞄偏置

#define YAW_AUTO_OFFSET 0.036f
#define PITCH_AUTO_OFFSET 0.07f//0.1134380453752182f


/**********************************云台****************************************/

//yaw speed close-loop PID params, max out and max iout
//yaw 速度环 PID参数以及 PID最大输出，积分输出

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
//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP        1.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.5f
#define YAW_ENCODE_RELATIVE_PID_KD        0.32f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   2.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f


//pitch speed close-loop PID params, max out and max iout
//pitch 速度环 PID参数以及 PID最大输出，积分输出
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


//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 300
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0


#define MIDDLE_YAW    36694
#define MIDDLE_PITCH  27923 //36046

#define MAX_PITCH 4700     //6200    //11.26改的没试
#define MIN_PITCH 3900

#define PITCH_TURN  0
#define YAW_TURN    0

/**************************自动搜索***************************/

//自动搜索过程中yaw轴以及pitch轴扫的增量-》速度
#define SEARCH_YAW_SPEED 0.00085f
#define SEARCH_PITCH_SPEED 0.0007f

//云台自主扫描过程中，yaw轴的扫描范围
#define SEARCH_YAW_MAX 1.3f
#define SEARCH_YAW_MIN -1.3f

#define SEARCH_PITCH_HI 0.126647547f
#define SEARCH_PITCH_LO -0.248646989f


//turn speed
//掉头云台速度
#define TURN_SPEED    0.04f
//测试按键尚未使用
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//rocker value deadband
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   80


//定义遥控器模式下，遥控器通道值到控制值之间的转化，将遥控器通道值转为增量弧度值。
#define YAW_RC_SEN    -0.0008f
#define PITCH_RC_SEN  -0.0008f //0.005
//定义达妙电机位置速度模式（遥控器）下yaw，pitch的最大速度rad/s。
#define YAW_MAX_SPEED 1
#define PITCH_MAX_SPEED 5
//定义达妙电机yaw与pitch的id
#define YAW_MOTOR_ID 0x1
#define PITCH_MOTOR_ID 0x18

#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00015f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 1

//电机码盘值最大以及中值
#define HALF_ECD_RANGE_DM  32768
#define ECD_RANGE_DM       65536
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.002f
#define GIMBAL_INIT_ANGLE_ERROR  0.1f
#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

//电机编码值转化成角度值
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
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
		GIMBAL_MOTOR_GYRO_READY,//电机陀螺仪控制模式，处于down状态
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
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
		gimbal_motor_mode_e last_remote_mote_mode;  //上一次遥控器下的yaw模式，分为相对编码与绝对编码
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad
	  fp32 max_absolute_angle; //rad
    fp32 min_absolute_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
		fp32 motor_relative_speed;     //电机相对编码速度
		fp32 motor_relative_speed_set;	//电机相对编码速度目标值
	
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s 根据陀螺仪结算的yaw绝对角速度
    fp32 motor_gyro_set;     //rad/s yaw绝对角速度的目标值
		fp32 motor_last_gyro; 
    fp32 motor_speed;					
	  fp32 absolute_motor_speed;
    fp32 raw_cmd_current;    //
    fp32 current_set;
    fp32 given_current;
		fp32 averrage_gyro_buffer[AVERAGE_FILTER_BUFFER_LENGTH];  //递推均值滤波数组，用于陀螺仪速度滤波

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
		fp32 motor_relative_speed;     //电机相对编码速度
		fp32 motor_relative_speed_set;	//电机相对编码速度目标值

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
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
extern const gimbal_motor_t* get_yaw_motor_point(void);

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
/**
  * @brief          返回pitch 电机数据指针
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
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
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
  * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     yaw 中值 指针
  * @param[out]     pitch 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     pitch 最大相对角度 指针
  * @param[out]     pitch 最小相对角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
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
  * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
  * @param[in]      yaw_offse:yaw 中值
  * @param[in]      pitch_offset:pitch 中值
  * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
  * @param[in]      min_yaw:yaw 最小相对角度
  * @param[in]      max_yaw:pitch 最大相对角度
  * @param[in]      min_yaw:pitch 最小相对角度
  * @retval         返回空
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */

extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern void grab_reset(void);
void up_down_motro_move(fp32 target_angle, gimbal_control_t *gimbal_control, int *flag);

#endif
