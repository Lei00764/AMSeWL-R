//#ifndef __SUPERCAP_H_
//#define __SUPERCAP_H_

//#include "struct_typedef.h"
//#include "CAN_receive.h"
//#include "remote_control.h"

//#define Cap_Offline_Debug 0
//#define Referee_System 1
//#define SUPERCAP_TASK_INIT_TIME 100
//#define SUPERCAP_CONTROL_TIME 1
//#define SUPER_CAP_ON 1

//typedef enum{
//	cap_dis_ucharge = 0, //flag = 0		��������õ�طŵ�
//	cap_dis_charge  = 1, //flag = 1		������õ�طŵ�
//	cap_en_ucharge  = 2, //flag = 2		����絫���ݹ���
//	cap_en_charge   = 3, //flag = 3		����ҵ��ݷŵ�
//	
//	cap_just_disable = 4,
//}cap_state;

//typedef struct{
//	//const supercap_module_receive *cap_message;
//	const RC_ctrl_t *cap_rc; 
//}cap_control_t;

//extern int32_t Power_Limitation_Num;
//extern int32_t Residue_Power;
//extern int32_t Lost_Connection_Count;
//extern int32_t Chassis_Power;
//extern cap_state cap_FSM;
//extern cap_state cap_FSM_ex;

//extern void supercap_task(void const * argument);

//#endif
