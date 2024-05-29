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

#include "gimbal_task.h"
#include "chassis_task.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include "usart_debug.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "usart.h"

#include "pid.h"
#include "bsp_buzzer.h"
//motor enconde value format, range[0-8191]
//�������ֵ���� 0��8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }
		
		
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     init:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t* init);
/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
  * @param[out]     gimbal_set_mode:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t* set_mode);
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     gimbal_feedback_update:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t* feedback_update);
/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��̨ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     mode_change:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t* mode_change);

/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle, unit rad
  */
/**
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
  * @param[out]     gimbal_set_control:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t* set_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current,
  *                 motor current will be sent to motor
  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t* control_loop);

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t* gimbal_motor);
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t* gimbal_motor);
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_RAW������ֱֵ�ӷ��͵�CAN����.
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t* gimbal_motor);
/**
  * @brief          limit angle set in GIMBAL_MOTOR_GYRO mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��GIMBAL_MOTOR_GYROģʽ�����ƽǶ��趨,��ֹ�������
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t* gimbal_motor, fp32 add);
static void gimbal_absolute_angle_no_limit(gimbal_motor_t* gimbal_motor, fp32 add);
static void gimbal_motor_absolute_angle_control_pitch(gimbal_motor_t* gimbal_motor);

static void gimbal_auto_angle_no_limit(gimbal_motor_t* gimbal_motor, fp32 add);
/**
  * @brief          limit angle set in GIMBAL_MOTOR_ENCONDE mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��GIMBAL_MOTOR_ENCONDEģʽ�����ƽǶ��趨,��ֹ�������
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t* gimbal_motor, fp32 add);
static void gimbal_auto_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_relative_angle_no_limit(gimbal_motor_t* gimbal_motor, fp32 add);
/**
  * @brief          gimbal angle pid init, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      maxout: pid max out
  * @param[in]      intergral_limit: pid max iout
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
/**
  * @brief          ��̨�Ƕ�PID��ʼ��, ��Ϊ�Ƕȷ�Χ��(-pi,pi)��������PID.c��PID
  * @param[out]     pid:��̨PIDָ��
  * @param[in]      maxout: pid������
  * @param[in]      intergral_limit: pid���������
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t* pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
//static void gimbal_PID_init_oldmethod(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout,fp32 max_dout, fp32 kp, fp32 ki, fp32 kd, fp32 dT);
/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��̨PID��������pid��out,iout
  * @param[out]     pid_clear:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t* pid_clear);
fp32 PID_calc_1(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          gimbal angle pid calc, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      get: angle feeback
  * @param[in]      set: angle set-point
  * @param[in]      error_delta: rotation speed
  * @retval         pid out
  */
/**
  * @brief          ��̨�Ƕ�PID����, ��Ϊ�Ƕȷ�Χ��(-pi,pi)��������PID.c��PID
  * @param[out]     pid:��̨PIDָ��
  * @param[in]      get: �Ƕȷ���
  * @param[in]      set: �Ƕ��趨
  * @param[in]      error_delta: ���ٶ�
  * @retval         pid ���
  */
static fp32 gimbal_PID_calc(gimbal_PID_t* pid, fp32 get, fp32 set, fp32 error_delta);
static fp32 gimbal_PID_calc1(gimbal_PID_t* pid, fp32 get, fp32 set, fp32 error_delta);

//static fp32 gimbal_PID_calc_oldmethod(gimbal_PID_t *pid, fp32 get, fp32 set);
/**
  * @brief          gimbal calibration calculate
  * @param[in]      gimbal_cali: cali data
  * @param[out]     yaw_offset:yaw motor middle place encode
  * @param[out]     pitch_offset:pitch motor middle place encode
  * @param[out]     max_yaw:yaw motor max machine angle
  * @param[out]     min_yaw: yaw motor min machine angle
  * @param[out]     max_pitch: pitch motor max machine angle
  * @param[out]     min_pitch: pitch motor min machine angle
  * @retval         none
  */
/**
  * @brief          ��̨У׼����
  * @param[in]      gimbal_cali: У׼����
  * @param[out]     yaw_offset:yaw�����̨��ֵ
  * @param[out]     pitch_offset:pitch �����̨��ֵ
  * @param[out]     max_yaw:yaw �������е�Ƕ�
  * @param[out]     min_yaw: yaw �����С��е�Ƕ�
  * @param[out]     max_pitch: pitch �������е�Ƕ�
  * @param[out]     min_pitch: pitch �����С��е�Ƕ�
  * @retval         none
  */
static void calc_gimbal_cali(const gimbal_step_cali_t* gimbal_cali, uint16_t* yaw_offset, uint16_t* pitch_offset, fp32* max_yaw, fp32* min_yaw, fp32* max_pitch, fp32* min_pitch);

//���ƾ�ֵ�˲��������������ٶ��˲�
fp32 average_filter_pitch(gimbal_motor_t *gimbal_motor,int buffer_len);
fp32 average_filter_yaw(gimbal_motor_t *gimbal_motor,int buffer_len);
fp32 average_filter_pitch_current(gimbal_motor_t* gimbal_motor,fp32 *buffer,int buffer_len);
fp32 pitch_current[PITCH_CURRENT_AVERAGE_FILTER_BUFFER_LENGTH];
void grab_staff(fp32 grab_angle,fp32 yaw_angle, fp32 pitch_angle, gimbal_control_t* gimbal_grab);
void yaw_pitch_slow_move(fp32 target_angle_yaw,fp32 target_angle_pitch , gimbal_control_t *gimbal_control,gimbal_move_mode mode);
void yaw_pitch_move(fp32 target_yaw,fp32 target_pitch,gimbal_control_t *gimbal_control, int *flag);
void vision_assistant_move(rx_data vision_data, gimbal_control_t *gimbal_control,int staff);
void put_staff(fp32 grab_pitch, fp32 grab_yaw, int grab_staff,fp32 put_pitch,fp32 put_yaw,gimbal_control_t *put_staff, int hight);
//gimbal control data
//��̨���������������
gimbal_control_t gimbal_control;

const int16_t *shoot1_can_set_current;

const int16_t *shoot2_can_set_current;
fp32 put_pos_pitch = 1.30f;
fp32 box_pos[3] = {1.62,0,-1.60};
fp32 coca_pos[3] = {2.78,3.1,-2.835};
fp32 grab_coca_down_pos = -12.626f;
fp32 grab_coca_up_pos = -70.558f;
fp32 put_coca_pos = -40.0f;
//����c���ϵ��¼yaw����ֵ
int count_flag = 200;

fp32 add_current = -1.8;
fp32 angle_pitch;
//pitch���±�����־
int pt_flag = 0;
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
	

int last_s1;
int bump_change_flag;
int grab_task_flag = 0;
int put_task_flag = 0;

void gimbal_task(void const* pvParameters)
{
    //�ȴ������������������������
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //gimbal init
    //��̨��ʼ��
    gimbal_init(&gimbal_control);

    //�����ʼ��
    //autoaim_init();
    //wait for all motor online

    while(1)
    {
				bump_change_flag = 0;
        gimbal_set_mode(&gimbal_control);                    //������̨����ģʽ
								
			
        gimbal_mode_change_control_transit(&gimbal_control); //����ģʽ�л� �������ݹ���
        gimbal_feedback_update(&gimbal_control);             //��̨���ݷ���
        gimbal_set_control(&gimbal_control);  				//������̨������
			        
				taskENTER_CRITICAL();
        gimbal_control_loop(&gimbal_control);     			//��̨����PID����
        taskEXIT_CRITICAL();


			
				//��̨����ģʽ��ϵͳ��ʶ��ȡ���������������ɵļ����ź�
				#if GIMBAL_SYSTEM_IDENTIFICATION_MODE
				#else
			if(gimbal_control.gimbal_rc_ctrl->rc.s[0] != 2)
			{
				CAN_cmd_gimbal(gimbal_control.gimbal_yaw_motor.given_current,gimbal_control.gimbal_pitch_motor.given_current,gimbal_control.gimbal_updown_motor1.given_current,gimbal_control.gimbal_updown_motor2.given_current);
			}
			else
			{
			CAN_cmd_gimbal(0,0,0,0);
			}
			//CAN_cmd_gimbal(0,1000,0,0);
			if(count_flag > 1)
				{
					gimbal_control.gimbal_updown_motor1.offset_ecd = gimbal_control.gimbal_updown_motor1.gimbal_motor_measure->ecd;
					gimbal_control.gimbal_updown_motor2.offset_ecd = gimbal_control.gimbal_updown_motor2.gimbal_motor_measure->ecd;
					gimbal_control.gimbal_pitch_motor.offset_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
					count_flag--;
					buzzer_on(1, 20000);			
				}
				else
				{
				buzzer_off();
				}
			
        //send_to_computer(-gimbal_control.gimbal_yaw_motor.absolute_angle, gimbal_control.gimbal_pitch_motor.absolute_angle);
				#endif
				
				if(gimbal_control.gimbal_rc_ctrl->rc.s[1] != last_s1)
				{
					bump_change_flag = 1;
				}
				//ң����ģʽ�£����ÿ���
				if(gimbal_control.gimbal_rc_ctrl->rc.s[0] == 3)
				{
				if(bump_change_flag)
				{
					if(gimbal_control.bump_flag == 1)gimbal_control.bump_flag = 2;
					else if(gimbal_control.bump_flag == 2) gimbal_control.bump_flag = 3;
					else if(gimbal_control.bump_flag == 3) gimbal_control.bump_flag = 1;
				}
				if(gimbal_control.bump_flag == 1)
				{
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,500);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,500);
				}
				else if(gimbal_control.bump_flag == 2)
				{
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2500);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,500);
				}
				else if(gimbal_control.bump_flag == 3)
				{
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2500);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,2500);
				}
			}
				last_s1 = gimbal_control.gimbal_rc_ctrl->rc.s[1];

				vTaskDelay(1);
    }
}


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
  * @param[in]      max_pitch:pitch �����ԽǶ�
  * @param[in]      min_pitch:pitch ��С��ԽǶ�
  * @retval         ���ؿ�
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}


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
bool_t cmd_cali_gimbal_hook(uint16_t* yaw_offset, uint16_t* pitch_offset, fp32* max_yaw, fp32* min_yaw, fp32* max_pitch, fp32* min_pitch)
{
    if(gimbal_control.gimbal_cali.step == 0)
    {
        gimbal_control.gimbal_cali.step             = GIMBAL_CALI_START_STEP;
        //�������ʱ������ݣ���Ϊ��ʼ���ݣ����ж������Сֵ
        gimbal_control.gimbal_cali.max_pitch        = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw          = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_pitch        = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw          = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        return 0;
    }
    else if(gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        gimbal_control.gimbal_yaw_motor.offset_ecd              = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_relative_angle      = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_relative_angle      = *min_yaw;
        gimbal_control.gimbal_pitch_motor.offset_ecd            = *pitch_offset;
        gimbal_control.gimbal_pitch_motor.max_relative_angle    = *max_pitch;
        gimbal_control.gimbal_pitch_motor.min_relative_angle    = *min_pitch;
        gimbal_control.gimbal_cali.step = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          calc motor offset encode, max and min relative angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max relative angle
  * @param[out]     min_yaw:yaw min relative angle
  * @param[out]     max_yaw:pitch max relative angle
  * @param[out]     min_yaw:pitch min relative angle
  * @retval         none
  */
/**
  * @brief          ��̨У׼���㣬��У׼��¼����ֵ,��� ��Сֵ
  * @param[out]     yaw ��ֵ ָ��
  * @param[out]     pitch ��ֵ ָ��
  * @param[out]     yaw �����ԽǶ� ָ��
  * @param[out]     yaw ��С��ԽǶ� ָ��
  * @param[out]     pitch �����ԽǶ� ָ��
  * @param[out]     pitch ��С��ԽǶ� ָ��
  * @retval         none
  */
static void calc_gimbal_cali(const gimbal_step_cali_t* gimbal_cali, uint16_t* yaw_offset, uint16_t* pitch_offset, fp32* max_yaw, fp32* min_yaw, fp32* max_pitch, fp32* min_pitch)
{
    if(gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN

#endif
}

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
const gimbal_motor_t* get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

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
const gimbal_motor_t* get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     init:"gimbal_control"����ָ��.
  * @retval         none
  */

/**
		@update 21.08.11 ������ֵ��ʼ��
*/
extern uint8_t receive_data[6];
 static void gimbal_init(gimbal_control_t* init)
{

    static const fp32 Pitch_speed_encode_pid[3] = {PITCH_SPEED_ENCODE_PID_KP, PITCH_SPEED_ENCODE_PID_KI, PITCH_SPEED_ENCODE_PID_KD};
		static const fp32 Pitch_speed_pid[3] = {3500, 0.1, -3000};
		static const fp32 updown_speed_pid[3] = {800, 0.5, 0.0};
			MX_USART1_UART_Init();
			HAL_UART_Receive_IT(&huart1,receive_data,12);
    //�������ָ���ȡ
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
		init->gimbal_updown_motor1.gimbal_motor_measure = get_trigger_motor1_measure_point();
		init->gimbal_updown_motor2.gimbal_motor_measure = get_trigger_motor2_measure_point();

    //��ʼ�������ֵ����λ
    init->gimbal_yaw_motor.offset_ecd = 2063;
    init->gimbal_pitch_motor.offset_ecd = 6463;
		init->gimbal_updown_motor1.offset_ecd = 1000;
		init->gimbal_updown_motor2.offset_ecd = 1000;
		init->game_start_count = 0;
    init->gimbal_yaw_motor.max_relative_angle = PI;
    init->gimbal_yaw_motor.min_relative_angle = -PI;
		
		init->gimbal_pitch_motor.min_relative_angle = 0.0;
		init->gimbal_pitch_motor.max_relative_angle = 1.30;
		
		
		init->gimbal_pitch_motor.min_absolute_angle = -0.42;
		init->gimbal_pitch_motor.max_absolute_angle = 0.2;
		
		init->gimbal_yaw_motor.min_absolute_angle = SEARCH_YAW_MIN;
		init->gimbal_yaw_motor.max_absolute_angle = SEARCH_YAW_MAX;


    //����������ָ���ȡ
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();

    //ң��������ָ���ȡ
    init->gimbal_rc_ctrl = get_remote_control_point();
		last_s1 = init->gimbal_rc_ctrl->rc.s[1];
    //��ʼ�����ģʽ
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTO_DOWN;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTO_DOWN;
		
		//��ʼ����ģʽ
		init->bump_flag = 1;
		
    //��ʼ��yaw���pid
		//�ǶȻ�
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, 10, 0, 10, 0, -30); //max_out  max_iout  p  i  d
   //�ٶȻ�
	  static const fp32 Yaw_speed_pid[3] = {10000.0f, 30.0f, -20000.0f};
		PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);

    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, 10, 1.5, 15, 0.02, 0);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid,PID_POSITION,Pitch_speed_pid,10000,3000);
		
		    PID_init(&init->gimbal_updown_motor1.gimbal_motor_absolute_angle_pid, PID_POSITION, updown_speed_pid, 10000, 7000);
		    PID_init(&init->gimbal_updown_motor2.gimbal_motor_absolute_angle_pid, PID_POSITION, updown_speed_pid, 10000, 7000);


    //�������PID
		
    gimbal_total_pid_clear(init);

    gimbal_feedback_update(init);

		
    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;
		init->gimbal_yaw_motor.motor_last_gyro = init->gimbal_yaw_motor.motor_gyro;


    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
		
		init->auto_gimbal_mode = STEP1_;
		init->auto_gimbal_condition = GWAIT;
		init->auto_grab_count = 0;
		init->auto_grab_count = 0;
}

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
  * @param[out]     gimbal_set_mode:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t* set_mode)
{
    if(set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     gimbal_feedback_update:"gimbal_control"����ָ��.
  * @retval         none
  */\

fp32 testangle;
static void gimbal_feedback_update(gimbal_control_t* feedback_update)
{
    if(feedback_update == NULL)
    {
        return;
    }
    //��̨���ݸ���
    feedback_update->gimbal_pitch_motor.absolute_angle = -*(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
		
		feedback_update->gimbal_yaw_motor.absolute_motor_speed = *feedback_update->gimbal_INT_gyro_point;

		//	pitch
//    if(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd - feedback_update->gimbal_pitch_motor.gimbal_motor_measure->last_ecd < -4096)
//		{
//			feedback_update->gimbal_3508_count ++;
//		}
//		if(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd - feedback_update->gimbal_pitch_motor.gimbal_motor_measure->last_ecd > 4096)
//		{
//			feedback_update->gimbal_3508_count --;
//		}
//		feedback_update->gimbal_pitch_motor.relative_angle = (8192 * feedback_update->gimbal_3508_count + feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd - feedback_update->gimbal_pitch_motor.offset_ecd);
//		feedback_update->gimbal_pitch_motor.relative_angle = feedback_update->gimbal_pitch_motor.relative_angle / 4096 * 3.1415 / 19;
		
    static fp32 speed3_fliter_1 = 0.0f;
    static fp32 speed3_fliter_2 = 0.0f;
    static fp32 speed3_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter3_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed3_fliter_1 = speed3_fliter_2;
    speed3_fliter_2 = speed3_fliter_3;
    speed3_fliter_3 = speed3_fliter_2 * fliter3_num[0] + speed3_fliter_1 * fliter3_num[1] + (feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm * 0.0055087719298246f) * fliter3_num[2];
    feedback_update->gimbal_pitch_motor.motor_relative_speed = speed3_fliter_3;
		
		
		//updown1
//		if(feedback_update->gimbal_updown_motor1.gimbal_motor_measure->ecd - feedback_update->gimbal_updown_motor1.gimbal_motor_measure->last_ecd < -4096)
//		{
//			feedback_update->gimbal_updown_count1 ++;
//		}
//		else if(feedback_update->gimbal_updown_motor1.gimbal_motor_measure->ecd - feedback_update->gimbal_updown_motor1.gimbal_motor_measure->last_ecd > 4096)
//		{
//			feedback_update->gimbal_updown_count1 --;
//		}
//		
//		feedback_update->gimbal_updown_motor1.relative_angle = (8192 * feedback_update->gimbal_updown_count1 + feedback_update->gimbal_updown_motor1.gimbal_motor_measure->ecd - feedback_update->gimbal_updown_motor1.offset_ecd);
//		feedback_update->gimbal_updown_motor1.relative_angle = feedback_update->gimbal_updown_motor1.relative_angle / 4096 * 3.14 / 36;
		
		//updown2
		if(feedback_update->gimbal_updown_motor2.gimbal_motor_measure->ecd - feedback_update->gimbal_updown_motor2.gimbal_motor_measure->last_ecd < -4096)
		{
			feedback_update->gimbal_updown_count2 ++;
		}
		else if(feedback_update->gimbal_updown_motor2.gimbal_motor_measure->ecd - feedback_update->gimbal_updown_motor2.gimbal_motor_measure->last_ecd > 4096)
		{
			feedback_update->gimbal_updown_count2 --;
		}
		
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (feedback_update->gimbal_updown_motor1.gimbal_motor_measure->speed_rpm * 0.00290888208665721596153948461415f) * fliter_num[2];
    feedback_update->gimbal_updown_motor1.motor_relative_speed = speed_fliter_3;
		
    static fp32 speed1_fliter_1 = 0.0f;
    static fp32 speed1_fliter_2 = 0.0f;
    static fp32 speed1_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter1_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed1_fliter_1 = speed1_fliter_2;
    speed1_fliter_2 = speed1_fliter_3;
    speed1_fliter_3 = speed1_fliter_2 * fliter1_num[0] + speed1_fliter_1 * fliter1_num[1] + (feedback_update->gimbal_updown_motor2.gimbal_motor_measure->speed_rpm * 0.00290888208665721596153948461415f) * fliter1_num[2];
    feedback_update->gimbal_updown_motor2.motor_relative_speed = speed1_fliter_3;
		
		
		feedback_update->gimbal_updown_motor2.relative_angle = (8192 * feedback_update->gimbal_updown_count2 + feedback_update->gimbal_updown_motor2.gimbal_motor_measure->ecd - feedback_update->gimbal_updown_motor2.offset_ecd);
		feedback_update->gimbal_updown_motor2.relative_angle = feedback_update->gimbal_updown_motor2.relative_angle / 4096 * 3.14 / 36;
		
		feedback_update->gimbal_3508_pos = feedback_update->gimbal_pitch_motor.relative_angle;
		feedback_update->gimbal_pitch_motor.motor_gyro = -*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
		feedback_update->gimbal_pitch_motor.motor_relative_speed = feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm* 3.1415 / 4096;
    
		
		feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
		feedback_update->gimbal_yaw_motor.motor_relative_speed = feedback_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm * 3.1415 / 4096;
		

    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);
		feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.absolute_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        +arm_sin_f32(feedback_update->gimbal_pitch_motor.absolute_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));

}

/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle, unit rad
  */
/**
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
  */


static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;

    if(relative_ecd > 4096)
    {
        relative_ecd -= 8192;
    }
    else if(relative_ecd < -4096)
    {
        relative_ecd += 8192;
    }
    return relative_ecd * MOTOR_ECD_TO_RAD_DM;
}

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     gimbal_mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��̨ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     gimbal_mode_change:"gimbal_control"����ָ��.
  * @retval         none
  */
extern chassis_move_t chassis_move;

static void gimbal_mode_change_control_transit(gimbal_control_t* gimbal_mode_change)
{
    if(gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw���״̬���л���������
    if(gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if(gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    else if(gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch���״̬���л���������
    if(gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if(gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
				gimbal_mode_change->gimbal_updown_motor1.relative_angle_set = gimbal_mode_change->gimbal_updown_motor1.relative_angle;
				gimbal_mode_change->gimbal_updown_motor2.relative_angle_set = gimbal_mode_change->gimbal_updown_motor2.relative_angle;


    }
    else if(gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ������̨�����趨ֵ������ֵ��ͨ��gim11111bal_behaviour_control_set�������õ�
  * @param[out]     gimbal_set_control:"gimbal_control"����ָ��.
  * @retval         none
  */
extern  gimbal_behaviour_e gimbal_behaviour;
fp32 angle_set_step1 = 0.0f;
extern uint8_t my_data[6];
extern rx_data Rx_data;
fp32 step1_yaw[6] = {1.52,2.08,2.80,-2.76,-2.08,-1.45};//1.5
fp32 step1_pitch[6] = {0.18,0.45,0.518,0.46,0.38,0.05};
int coca_count = 0;
int box_count = 0;
int step1_flag = 0;
fp32 grab_put_yaw = 0.0f;
fp32 grab_put_pitch = 0.0f;
int coca_believe_count = 0;
int box_believe_count = 0;
int pre_down_flag = 0;
fp32 down_pos = 0;
fp32 put_pitch[6] ={0.52,0.55,0.56,0.56,0.56,0.56};//0.38
fp32 put_yaw[6] = {0.99,0.88,-0.96,-0.96,-0.96,-0.96};
fp32 put_hight[6] = {-56,-70,-56,-70,-70,-70};
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
		fp32 updown_add = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, &updown_add, set_control);
		fp32 updown_current_angle1 = set_control->gimbal_updown_motor1.relative_angle_set;
		fp32 updown_current_angle2 = set_control->gimbal_updown_motor2.relative_angle_set;
		set_control->gimbal_updown_motor1.relative_angle_set = updown_current_angle1 + updown_add;
		set_control->gimbal_updown_motor2.relative_angle_set = updown_current_angle2 + updown_add;

    //yaw���ģʽ����
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle + gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO ||set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO_READY)
    {
        //gyroģʽ�£������ǽǶȿ���
			gimbal_absolute_angle_no_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
       
    }
		

    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //encondeģʽ�£��������Ƕȿ���
#if SLIP_RING
			gimbal_relative_angle_no_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
#else
			gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
#endif         
    }
		else if(set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTO_DOWN)
		{
			gimbal_auto_angle_no_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
		}

    //pitch���ģʽ����

    if (set_control->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL] == 3 )
    {
        //gyroģʽ�£������ǽǶȿ���
        //gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
				set_control->gimbal_pitch_motor.relative_angle_set = set_control->gimbal_pitch_motor.relative_angle_set + (fp32)set_control->gimbal_rc_ctrl->rc.ch[3] * -0.0008 / 1000;
			if(set_control->gimbal_pitch_motor.relative_angle_set > set_control->gimbal_pitch_motor.max_relative_angle)
			{
				set_control->gimbal_pitch_motor.relative_angle_set = set_control->gimbal_pitch_motor.max_relative_angle;
			}
			else if(set_control->gimbal_pitch_motor.relative_angle_set < set_control->gimbal_pitch_motor.min_relative_angle)
			{
				set_control->gimbal_pitch_motor.relative_angle_set = set_control->gimbal_pitch_motor.min_relative_angle;
			}
    }
	
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //encondeģʽ�£��������Ƕȿ���
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
		
		
if(set_control->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL] == 1)
{
	
	//�жϴ��ڵȴ�ģʽ����������ʾ���
	if(set_control->auto_gimbal_mode == STEP1_ && set_control->auto_gimbal_condition == GWAIT)
	{
		if(chassis_move.auto_move_mode == STEP2 && chassis_move.auto_move_condition == WAIT)
		{
			set_control->auto_gimbal_condition = GREADY;
			set_control->auto_gimbal_mode = STEP1_;
//			set_control->auto_gimbal_condition = GWAIT;
			set_control->gimbal_yaw_motor.relative_angle_set = set_control->gimbal_yaw_motor.relative_angle;
			set_control->gimbal_pitch_motor.relative_angle_set = set_control->gimbal_pitch_motor.relative_angle;
		}
	}
	
	else if(set_control->auto_gimbal_mode == STEP2_ && set_control->auto_gimbal_condition == GWAIT)
	{
			set_control->auto_gimbal_condition = GREADY;
		
			set_control->gimbal_yaw_motor.relative_angle_set = set_control->gimbal_yaw_motor.relative_angle;
			set_control->gimbal_pitch_motor.relative_angle_set = set_control->gimbal_pitch_motor.relative_angle;
	}
	
	else if(set_control->auto_gimbal_mode == STEP3_ && set_control->auto_gimbal_condition == GWAIT)
	{
		set_control->auto_gimbal_condition = GREADY;
	}
	else if(set_control->auto_gimbal_mode == STEP4_ && set_control->auto_gimbal_condition == GWAIT)
	{
		if(chassis_move.auto_move_mode == STEP3 && chassis_move.auto_move_condition == WAIT)
		{
			set_control->auto_gimbal_condition = GREADY;
		}
	}
	
	//���ھ���״̬�������������Ӧ�Ĳ���
	if(set_control->auto_gimbal_condition == GREADY)
	{
		if(set_control->auto_gimbal_mode == STEP1_)
		{
			if(set_control->auto_grab_count<6)
			
			up_down_motro_move(-80,set_control,&step1_flag);
			if(step1_flag)
			yaw_pitch_slow_move(step1_yaw[set_control->auto_grab_count],step1_pitch[set_control->auto_grab_count],set_control,STEP2_);
			box_believe_count = 0;
			coca_believe_count = 0;	
		}
		else if(set_control->auto_gimbal_mode == STEP2_)
		{
			if(Rx_data.staff == 1 + 48)
			{
			coca_believe_count++;
				if(coca_believe_count > 1000)
				{
					coca_believe_count = 1000;
					box_believe_count = 0;
				}
			}
			else if(Rx_data.staff == 0 + 48)
			{
			box_believe_count++;
				if(box_believe_count > 1000)
				{
					box_believe_count= 1000;
					coca_believe_count = 0;
				}
			}
			if(coca_believe_count > 800)
			{
				down_pos = -53;
			}
			else if(box_believe_count > 800)
			{
				down_pos = -80;
			}
			else
			{
			down_pos = -50;
			}
			up_down_motro_move(down_pos,set_control,&pre_down_flag);
			if(pre_down_flag==1)
			{
			put_task_flag = 0;
			vision_assistant_move(Rx_data,set_control,down_pos);
			}
			if(put_task_flag == 1)
			{
				pre_down_flag = 0;
				set_control->auto_gimbal_mode = STEP3_;
				set_control->auto_gimbal_condition = GWAIT;
			}
			
		}
		else if(set_control->auto_gimbal_mode == STEP3_)
		{
			grab_task_flag = 0;

			if(coca_believe_count > 800)
			{
				grab_put_yaw = coca_pos[coca_count];
			}
			else if(box_believe_count > 800)
			{
				grab_put_yaw = box_pos[box_count];
			}
			else
			{
				grab_put_yaw = set_control->gimbal_yaw_motor.relative_angle;
			}
			
			grab_staff(grab_coca_down_pos,grab_put_yaw,put_pos_pitch,set_control);
			if(grab_task_flag == 1)
			{
				if(box_believe_count>800)
				{
					box_count++;
				}
				else if(coca_believe_count > 800)
				{
					coca_count++;
				}
				box_believe_count = 0;
				coca_believe_count = 0;
				set_control->auto_grab_count++;
				if(set_control->auto_grab_count == 6)
				{
					set_control->auto_gimbal_mode = STEP4_;
				}
				else
				{
				set_control->auto_gimbal_mode = STEP1_;
				}
				set_control->auto_gimbal_condition = GWAIT;
				step1_flag = 0;
			}
		}
		else if(set_control->auto_gimbal_mode == STEP4_)
		{
			if(set_control->auto_grab_count > 0)
			{
			if(box_count > 0)
			{
			put_staff(put_pos_pitch,box_pos[3 - box_count],0,put_pitch[6 - set_control->auto_grab_count],put_yaw[6 - set_control->auto_grab_count],set_control,put_hight[6-set_control->auto_grab_count]);
			
			if(put_task_flag == 1)
			{
				set_control->auto_grab_count --;
				put_task_flag = 0;
				box_count--;			}
		}
			else if(coca_count > 0)
			{
			put_staff(put_pos_pitch,coca_pos[3 - coca_count],0,put_pitch[6 - set_control->auto_grab_count],put_yaw[6 - set_control->auto_gimbal_count ],set_control,put_hight[6 - set_control->auto_grab_count]);
			
			if(put_task_flag == 1)
			{
				set_control->auto_grab_count --;
				put_task_flag = 0;
				coca_count--;
			}
			}
		}
		else
		{
			set_control->auto_gimbal_mode = STEP5_;

		}
	}	
		set_control->gimbal_yaw_motor.relative_angle_set = rad_format(set_control->gimbal_yaw_motor.relative_angle_set);
	}
}
}

void yaw_pitch_slow_move(fp32 target_angle_yaw,fp32 target_angle_pitch , gimbal_control_t *gimbal_control,gimbal_move_mode mode)
{
	if(fabs(gimbal_control->gimbal_yaw_motor.relative_angle - target_angle_yaw) < 0.08 && fabs(gimbal_control->gimbal_pitch_motor.relative_angle - target_angle_pitch) < 0.08 )
	{
		gimbal_control->auto_gimbal_mode = mode;
		gimbal_control->auto_gimbal_condition = GWAIT;
	}
	if(rad_format(target_angle_yaw - gimbal_control->gimbal_yaw_motor.relative_angle_set) < 0)
	{
		gimbal_control->gimbal_yaw_motor.relative_angle_set = gimbal_control->gimbal_yaw_motor.relative_angle_set - 0.0003;
	}
	else if(rad_format(target_angle_yaw - gimbal_control->gimbal_yaw_motor.relative_angle_set) > 0)
	{
		gimbal_control->gimbal_yaw_motor.relative_angle_set = gimbal_control->gimbal_yaw_motor.relative_angle_set + 0.0003;
	}
	if(gimbal_control->gimbal_pitch_motor.relative_angle_set > target_angle_pitch)
	{
		gimbal_control->gimbal_pitch_motor.relative_angle_set = gimbal_control->gimbal_pitch_motor.relative_angle_set - 0.0005;
	}
	else if(gimbal_control->gimbal_pitch_motor.relative_angle_set  < target_angle_pitch)
	{
		gimbal_control->gimbal_pitch_motor.relative_angle_set = gimbal_control->gimbal_pitch_motor.relative_angle_set + 0.0005;
	}
}
fp32 speed_k = 1.2;

 void vision_assistant_move(rx_data vision_data, gimbal_control_t *gimbal_control,int staff)
{
if(vision_data.grab_flag == 1 + 48)
{
	gimbal_control->auto_gimbal_mode = STEP3_;
	gimbal_control->auto_gimbal_condition = GWAIT;
}

else 
{
	if(staff == -53)
{
	speed_k = 1.2;
}
else if(staff == -80)
{
	speed_k = 3;
}
//	gimbal_control->gimbal_yaw_motor.relative_angle_set = vision_data.yaw_angle + gimbal_control->gimbal_yaw_motor.relative_angle;
	if(vision_data.yaw_angle > 0.1)
		gimbal_control->gimbal_yaw_motor.relative_angle_set = gimbal_control->gimbal_yaw_motor.relative_angle_set + 0.00002 * speed_k;
	else if(vision_data.yaw_angle < -0.1)
		gimbal_control->gimbal_yaw_motor.relative_angle_set = gimbal_control->gimbal_yaw_motor.relative_angle_set - 0.00002 * speed_k;
	if(vision_data.yaw_angle > 0 && vision_data.yaw_angle < 0.1)
	{
		gimbal_control->gimbal_yaw_motor.relative_angle_set = gimbal_control->gimbal_yaw_motor.relative_angle_set + 0.00001 * vision_data.yaw_angle * 8 * speed_k;
	}
	else if(vision_data.yaw_angle < 0 && vision_data.yaw_angle > -0.1)
	{
		gimbal_control->gimbal_yaw_motor.relative_angle_set = gimbal_control->gimbal_yaw_motor.relative_angle_set + 0.00001 * vision_data.yaw_angle * 8 * speed_k;
	}


		if(vision_data.pitch_angle > 1.5)
		{
			vision_data.pitch_angle = 1.5;
		}
		else if(vision_data.pitch_angle < -1.5)
		{
			vision_data.pitch_angle = -1.5;
		}
		if(vision_data.pitch_angle  > 0)
		gimbal_control->gimbal_pitch_motor.relative_angle_set = gimbal_control->gimbal_pitch_motor.relative_angle_set + 0.00002 * speed_k * vision_data.pitch_angle ;
	else if(vision_data.pitch_angle < 0)
		gimbal_control->gimbal_pitch_motor.relative_angle_set = gimbal_control->gimbal_pitch_motor.relative_angle_set + 0.00002 * speed_k * vision_data.pitch_angle ;
}

if(gimbal_control->gimbal_pitch_motor.relative_angle_set > gimbal_control->gimbal_pitch_motor.max_relative_angle)
	gimbal_control->gimbal_pitch_motor.relative_angle_set = gimbal_control->gimbal_pitch_motor.max_relative_angle;
else if(gimbal_control->gimbal_pitch_motor.relative_angle_set < gimbal_control->gimbal_pitch_motor.min_relative_angle)
		gimbal_control->gimbal_pitch_motor.relative_angle_set = gimbal_control->gimbal_pitch_motor.min_relative_angle;
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */

static void gimbal_absolute_angle_limit(gimbal_motor_t* gimbal_motor, fp32 add)
{
    static fp32 bias_angle;
    static fp32 angle_set;
    if(gimbal_motor == NULL)
    {
        return;
    }
    //now angle error
    //��ǰ�������Ƕ�
//    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //relative angle + angle error + add_angle > max_relative angle
    //��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�

    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO������λ�汾��Ϊ�˱�֤��ʽһ�������Ե�������������
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_absolute_angle_no_limit(gimbal_motor_t* gimbal_motor, fp32 add)
{
    static fp32 angle_set;
    if(gimbal_motor == NULL)
    {
        return;
    }
    angle_set = gimbal_motor->relative_angle_set;
    gimbal_motor->relative_angle_set = rad_format(angle_set + add);
}

static void gimbal_auto_angle_no_limit(gimbal_motor_t* gimbal_motor, fp32 add)
{
    if(gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->absolute_angle_set = rad_format(add);
}


/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */

static void gimbal_relative_angle_limit(gimbal_motor_t* gimbal_motor, fp32 add)
{
    if(gimbal_motor == NULL)
    {
        return;
    }
		
			//pitch���½Ƕ���λ
			if(gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
			//if(gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd > 0)
			{
					if(add>0)
							add = 0;
				}
			if(gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
			//if(gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd < -0.8)
			{
				if(add<0)
							add = 0;
			}

    gimbal_motor->relative_angle_set += add;

}


/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE������λ�汾
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_relative_angle_no_limit(gimbal_motor_t* gimbal_motor, fp32 add)
{
    if(gimbal_motor == NULL)
    {
        return;
    }
		fp32 angle_set;
		angle_set = gimbal_motor->relative_angle_set;
    gimbal_motor->relative_angle_set = rad_format(add+angle_set);
//    //�Ƿ񳬹���� ��Сֵ
//    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
//    {
//        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
//    }
//    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
//    {
//        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
//    }
}


/**
  * @brief          control loop, according to control set-point, calculate motor current,
  *                 motor current will be sent to motor
  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t* control_loop)
{
    if(control_loop == NULL)
    {
        return;
    }
    if(control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if(control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)   //ң����������->yaw���ԽǶȿ���
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if(control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO_READY) 
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if(control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)	////ң����������->yaw��ԽǶȿ���
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
    }
		else if(control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTO_DOWN)
		{
			gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
		}
		
		
    if(control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if(control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control_pitch(&control_loop->gimbal_pitch_motor);
			


		

    }
    else if(control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
    }
		else if(control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTO_DOWN)
		{
			gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
		}
		
		
		if(control_loop->gimbal_updown_motor1.move_flag == 1)
		{
			control_loop->gimbal_updown_motor1.motor_relative_speed_set = 15;
		}
		else if(control_loop->gimbal_updown_motor1.move_flag == -1)
		{
			control_loop->gimbal_updown_motor1.motor_relative_speed_set = -15;
		}
		else
		{
			control_loop->gimbal_updown_motor1.motor_relative_speed_set = 0;
		}
		
		    control_loop->gimbal_updown_motor1.given_current = PID_calc_1(&control_loop->gimbal_updown_motor1.gimbal_motor_absolute_angle_pid, 
													control_loop->gimbal_updown_motor1.motor_relative_speed, 
													control_loop->gimbal_updown_motor1.motor_relative_speed_set);
		
		    control_loop->gimbal_updown_motor2.given_current = PID_calc_1(&control_loop->gimbal_updown_motor2.gimbal_motor_absolute_angle_pid, 
													control_loop->gimbal_updown_motor2.motor_relative_speed, 
													-control_loop->gimbal_updown_motor1.motor_relative_speed_set);


}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t* gimbal_motor)
{
    if(gimbal_motor == NULL)
    {
        return;
    }


    //�ǶȻ����ٶȻ�����pid����
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, 
													gimbal_motor->relative_angle, 
													gimbal_motor->relative_angle_set, 
													gimbal_motor->motor_relative_speed);
		
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid,gimbal_motor->motor_relative_speed, gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
		

		
		//for test
		if (gimbal_motor->gimbal_motor_mode == GIMBAL_MOTO_DOWN)
		{
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
		}
		else
			{
				//if(gimbal_motor->)
		 gimbal_motor->given_current =  gimbal_motor->current_set;
		}
}

static void gimbal_motor_absolute_angle_control_pitch(gimbal_motor_t* gimbal_motor)
{
    if(gimbal_motor == NULL)
    {
        return;
    }


    //�ǶȻ����ٶȻ�����pid����
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, 
													gimbal_motor->relative_angle, 
													gimbal_motor->relative_angle_set, 
													gimbal_motor->motor_relative_speed);
		
		
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid,gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
		
		 gimbal_motor->given_current =  gimbal_motor->current_set;
		

}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //�ǶȻ����ٶȻ�����pid����
    //gimbal_motor->motor_relative_speed_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set,-gimbal_motor->gimbal_motor_measure->speed_rad);
    //gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_encode_pid, gimbal_motor->motor_relative_speed, gimbal_motor->motor_relative_speed_set);
    //����ֵ��ֵ
    gimbal_motor->given_current = (gimbal_motor->current_set);
}
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_RAW������ֱֵ�ӷ��͵�CAN����.
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t* gimbal_motor)
{
    if(gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}


/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     gimbal_init:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t* pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if(pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}
/*
static void gimbal_PID_init_oldmethod(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout,fp32 max_dout, fp32 kp, fp32 ki, fp32 kd, fp32 dT)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
		pid->max_dout = max_dout;
    pid->max_out = maxout;

		pid->dT = dT;

}
*/

static fp32 gimbal_PID_calc(gimbal_PID_t* pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if(pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
		pid->err_angle = pid->err * 180 / PI;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

fp32 PID_calc_1(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}


static fp32 gimbal_PID_calc1(gimbal_PID_t* pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if(pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = err;
		pid->err_angle = pid->err * 180 / 2 / PI;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}
/*
static fp32 gimbal_PID_calc_oldmethod(gimbal_PID_t *pid, fp32 get, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;
    pid->err = rad_format(set - get) * 57.3f;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->last_err * pid->dT;
    pid->Dout = pid->kd * (pid->err - pid->last_err) / pid->dT;
    abs_limit(&pid->Iout, pid->max_iout);
		abs_limit(&pid->Dout, pid->max_dout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
		pid->last_err = pid->err;
    return pid->out;

}
*/
/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     gimbal_pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��̨PID��������pid��out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t* gimbal_pid_clear)
{
    if(gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->last_err = gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}


	static void gimbal_auto_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
	{
			static fp32 bias_angle;
			if (gimbal_motor == NULL)
			{
					return;
			}
			//now angle error
			//��ǰ�������Ƕ�
			bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
			//relative angle + angle error + add_angle > max_relative angle
			//��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
			if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
			{
					//�����������е�Ƕȿ��Ʒ���
					if (add > 0.0f)
					{
							//calculate max add_angle
							//�����һ��������ӽǶȣ�
							add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
					}
			}
			else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    gimbal_motor->absolute_angle_set = rad_format(add);
}
fp32 average_filter_yaw(gimbal_motor_t *gimbal_motor,int buffer_len)
{
	static int i = 0;
	fp32 sum = 0;
	if(i == buffer_len){i = 0;}
	gimbal_motor->averrage_gyro_buffer[i] = gimbal_motor->motor_gyro;
	i++;
	for(int n = 0;n < buffer_len;n++)
	{
	sum = gimbal_motor->averrage_gyro_buffer[n] + sum;
	}
	return sum / buffer_len;
}

fp32 average_filter_pitch(gimbal_motor_t *gimbal_motor,int buffer_len)
{
	static int i = 0;
	fp32 sum = 0;
	if(i == buffer_len){i = 0;}
	gimbal_motor->averrage_gyro_buffer[i] = gimbal_motor->motor_gyro;
	i++;
	for(int n = 0;n < buffer_len;n++)
	{
	sum = gimbal_motor->averrage_gyro_buffer[n] + sum;
	}
	return sum / buffer_len;
}

fp32 average_filter_pitch_current(gimbal_motor_t* gimbal_motor,fp32 *buffer,int buffer_len)
{
	static int i = 0;
	fp32 sum = 0;
	if(i == buffer_len){i = 0;}
	buffer[i] = gimbal_motor->given_current;
	i++;
	for(int n = 0;n < buffer_len;n++)
	{
	sum = buffer[n] + sum;
	}
	return sum / buffer_len;
}

fp32 angle_set = 0;
int pos1_flag = 0;  //�����Ƿ��µ�ָ��λ��
int pos2_flag = 0;  //�����Ƿ�������ָ��λ��
int pos3_flag = 0;  //yaw���������Ƿ��˶���ָ��λ�ã�׼��������ŵ�����
int pos4_flag = 0;	 //�����Ƿ��µ�ָ��λ�ã�׼�����û���
int pos5_flag = 0;  
int grab_flag = 0;
int put_flag = 0;  //��ŷ����������û���
int task_flag = 0;

void grab_reset(void)
{
	pos1_flag = 0;  //�����Ƿ��µ�ָ��λ��
	pos2_flag = 0;  //�����Ƿ�������ָ��λ��
	pos3_flag = 0;  //yaw���������Ƿ��˶���ָ��λ�ã�׼��������ŵ�����
	pos4_flag = 0;	 //�����Ƿ��µ�ָ��λ�ã�׼�����û���
	pos5_flag = 0;  
	grab_flag = 0;
	put_flag = 0;  //��ŷ����������û���
	task_flag = 0;
}
void grab_staff(fp32 grab_angle,fp32 yaw_angle, fp32 pitch_angle, gimbal_control_t* gimbal_grab)
{

		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2500);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,500);
	
		if(pos1_flag == 0)
		{	
			up_down_motro_move(grab_angle,gimbal_grab,&pos1_flag);
		}
		
		if(pos1_flag == 1 && grab_flag == 0) grab_flag = 1000;
		if(grab_flag > 1) grab_flag --;
		if(grab_flag == 1 && pos2_flag == 0)
		{			
			up_down_motro_move(grab_coca_up_pos,gimbal_grab,&pos2_flag);
			if(pos2_flag == 1) 
			{
				gimbal_grab->gimbal_yaw_motor.relative_angle_set = gimbal_grab->gimbal_yaw_motor.relative_angle;
				gimbal_grab->gimbal_pitch_motor.relative_angle_set = gimbal_grab->gimbal_pitch_motor.relative_angle;
			}
		}
		if(pos2_flag == 1 && pos3_flag == 0)
		{
			yaw_pitch_move(yaw_angle,pitch_angle,gimbal_grab,&pos3_flag);
		}
		if(pos3_flag > 1) pos3_flag --;
		
		else if(pos3_flag == 1 && pos4_flag == 0)
		{
			
			up_down_motro_move(put_coca_pos,gimbal_grab,&pos4_flag);

		}
		if(pos4_flag == 1 && put_flag == 0)
		{
			put_flag = 500;
		}
		if(put_flag > 1)
		{
			put_flag --;
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2500);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,2500);
		}
		if(put_flag == 1 && pos5_flag == 0)
		{
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2500);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,2500);
			up_down_motro_move(-80.0f,gimbal_grab,&pos5_flag);

		}
		if(pos5_flag == 1)
		{
		 pos1_flag = 0;
		  pos2_flag = 0;
		 pos3_flag = 0;
		 pos4_flag = 0;
			pos5_flag = 0;
		 grab_flag = 0;
		 put_flag = 0;
		 task_flag = 0;
		grab_task_flag = 1;
		}
		
		}

		
		int put_pos1 = 0;
		int put_pos2 = 0; 
		int put3 = 0;
		int put4 = 0;
		int put5 = 0;
		int put6 = 0;
		int put7 = 0;
		int put8 = 0;
		
void put_staff(fp32 grab_pitch, fp32 grab_yaw, int grab_staff,fp32 put_pitch,fp32 put_yaw,gimbal_control_t *put_staff,int hight)
{
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2500);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,500);
	
		if(put_pos1 == 0)
		{
			yaw_pitch_move(grab_yaw,grab_pitch,put_staff,&put_pos1);
		}
		if(put_pos1 > 1)put_pos1 --;
		if(put_pos1 == 1 && put_pos2 == 0) put_pos2 = 1000;
		if(put_pos2 > 1) put_pos2 --;
		if(put_pos2 == 1 && put3 == 0)
		{
			up_down_motro_move(-32,put_staff,&put3);
		}
		if(put3 ==1 && put4 == 0) put3 = 1000;
		if(put3 > 2) put3 --;
		if(put3 == 2 && put4 == 0)
		{
			up_down_motro_move(-80,put_staff,&put4);
		}
		if(put4 == 1 && put5 == 0)
		{
			yaw_pitch_move(put_yaw,put_pitch,put_staff,&put5);
		}
		if(put5 > 1)put5 --;
		if(put5 == 1 && put6 == 0)
		{
			up_down_motro_move(hight,put_staff,&put6);
		}
		if(put6 == 1 && put7 == 0)
		{
			put7 = 1000;
		}
		if(put7>1)
		{
			put7--;
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2500);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,2500);
		}
		if(put7 == 1 && put8 == 0)
			put8 = 1000;
		if(put8>1)put8--;
		if(put8 == 1)
		{
		put_pos1 = 0;
		put_pos2 = 0; 
		put3 = 0;
		put4 = 0;
		put5 = 0;
		put6 = 0;
		put7 = 0;
		put8 = 0;
		put_task_flag = 1;
		put_staff->gimbal_updown_motor1.move_flag = 0;

		}
	}
		
	
void up_down_motro_move(fp32 target_angle, gimbal_control_t *gimbal_control, int *flag)
{
			if((gimbal_control->gimbal_updown_motor1.relative_angle - target_angle) > 0.5)
		{
			gimbal_control->gimbal_updown_motor1.move_flag = -1;
		}
		else if((gimbal_control->gimbal_updown_motor1.relative_angle - target_angle) < -0.5)
		{
			gimbal_control->gimbal_updown_motor1.move_flag = 1;
		}
		else
		{
			gimbal_control->gimbal_updown_motor1.move_flag = 0;
			*flag = 1;
		}
}
fp32 speed_p_k = 1;
fp32 speed_y_k = 1;
int yaw_pitch_count = 0;
void yaw_pitch_move(fp32 target_yaw,fp32 target_pitch,gimbal_control_t *gimbal_control, int *flag)
{
		if(fabs(gimbal_control->gimbal_yaw_motor.relative_angle - target_yaw) < 0.025 && fabs(gimbal_control->gimbal_pitch_motor.relative_angle - target_pitch) < 0.025)
		{ 
			yaw_pitch_count ++;
		}
		else 
		{
		yaw_pitch_count = 0;
		}
		
		if(yaw_pitch_count > 300)
		{
			*flag = 500;
		}
		
		if(fabs(target_yaw - gimbal_control->gimbal_yaw_motor.relative_angle) > 0.3)
		{
		speed_y_k = 2;
		}
		else if(fabs(target_yaw - gimbal_control->gimbal_yaw_motor.relative_angle_set) < 0.3)
		{
		speed_y_k = 2 * (fabs(gimbal_control->gimbal_pitch_motor.relative_angle - target_pitch) + 0.2) ;
		}
		
		if(rad_format(target_yaw - gimbal_control->gimbal_yaw_motor.relative_angle_set) > 0 )
		{
			gimbal_control->gimbal_yaw_motor.relative_angle_set = gimbal_control->gimbal_yaw_motor.relative_angle_set + 0.00035 * speed_y_k;
		}
		else if(rad_format(target_yaw - gimbal_control->gimbal_yaw_motor.relative_angle_set) < 0)
		{
			gimbal_control->gimbal_yaw_motor.relative_angle_set = gimbal_control->gimbal_yaw_motor.relative_angle_set - 0.00035 * speed_y_k;
		}
		
		if(fabs(gimbal_control->gimbal_pitch_motor.relative_angle - target_pitch) > 0.3)
		{
		speed_p_k = 4 * fabs(gimbal_control->gimbal_pitch_motor.relative_angle - target_pitch);
		}
		else if(fabs(gimbal_control->gimbal_pitch_motor.relative_angle - target_pitch) < 0.3)
		{
		speed_p_k = 2 * (fabs(gimbal_control->gimbal_pitch_motor.relative_angle - target_pitch) + 0.2) ;
		}
		if(gimbal_control->gimbal_pitch_motor.relative_angle_set < target_pitch )
		{
			gimbal_control->gimbal_pitch_motor.relative_angle_set = gimbal_control->gimbal_pitch_motor.relative_angle_set + 0.0001 * speed_p_k;
		}
		else if(gimbal_control->gimbal_pitch_motor.relative_angle_set > target_pitch )
		{
			gimbal_control->gimbal_pitch_motor.relative_angle_set = gimbal_control->gimbal_pitch_motor.relative_angle_set - 0.0001 * speed_p_k;
		}
}