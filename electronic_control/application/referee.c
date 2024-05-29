/******************
裁判信息解读与透传数据帧封装程序
update: 2017.5.7
        视情况调用最后三个函数
        全局变量说明见RefereeInfo.h
        支持上传3个float数据
******************/
#include "referee.h"
#include <cstdint>
#include "gimbal_task.h"
#include <stdio.h>
#include <string.h>
#include "protocol.h"
#include "CAN_receive.h"
//#include "Task.h"
u16 cmd_id;  // 数据包ID
// 裁判信息相关结构体  
ext_game_status_t                           ext_game_status;// 比赛状态数据（0x0001）
ext_game_result_t                          ext_game_result;//比赛结果数据(0x0002)
ext_game_robot_HP_t                 ext_game_robot_HP;//机器人存存活数据（0x0003）
//ext_dart_status_t                          ext_dart_status;//飞镖发射状态(0x0004)//新手册中没有
ext_ICRA_buff_debuff_zone_status_t         ext_ICRA_buff_debuff_zone_status; //人工智能挑战赛加成与惩罚区状态(0x0005)



ext_event_data_t                           ext_event_data;//场地时事件数据（0x0101）
ext_supply_projectile_action_t             ext_supply_projectile_action;//补给站动作标识（0x0102）
//ext_supply_projectile_booking_t            ext_supply_projectile_booking;//补给站预约子弹（0x0103）//新手册中没有
ext_referee_warning_t                      ext_referee_warning;//裁判警告信息(0x0104)
ext_dart_remaining_time_t                  ext_dart_remaining_time;//飞镖发射口倒计时(0x0105)


ext_game_robot_status_t                     ext_game_robot_status;//比赛机器人状态(0x0201)
ext_power_heat_data_t                      ext_power_heat_data;////实时功率热量数据（0x0202）
ext_game_robot_pos_t                       ext_game_robot_pos;//机器人位置（0x0203）
ext_buff_t                                 ext_buff;//机器人增益（0x0204）
aerial_robot_energy_t                      ext_aerial_robot_energy;//空中机器人能量状态（0x0205）
ext_robot_hurt_t                           ext_robot_hurt;//伤害状态（0x0206）
ext_shoot_data_t                           ext_shoot_data;//实时射击信息（0x0207）
ext_bullet_remaining_t                     ext_bullet_remaining;//子弹剩余发射数(0x0208)
ext_rfid_status_t                          ext_rfid_status;//机器人RFID状态(0x0209)
ext_dart_client_cmd_t                      ext_dart_client_cmd;//飞镖机器人客户端指令数据(0x020A)

//-------------0x0301部分开始-------------------
ext_student_interactive_header_data_t      ext_student_interactive_header_data;//交互数据接收信息（0x0301）
//client_custom_data_t                       client_custom_data;
robot_interactive_data_t                   robot_interactive_data;//机器人间交互数据，内容 ID:0x0200~0x02FF
ext_client_custom_graphic_delete_t         ext_client_custom_graphic_delete;//客户端删除图形，内容 ID:0x0100;
graphic_data_struct_t                      graphic_data_struct;//图形数据
ext_client_custom_graphic_single_t         ext_client_custom_graphic_single;//客户端绘制一个图形
ext_client_custom_graphic_double_t         ext_client_custom_graphic_double;//客户端绘制两个图形
ext_client_custom_graphic_five_t           ext_client_custom_graphic_five;//客户端绘制五个图形
ext_client_custom_graphic_seven_t          ext_client_custom_graphic_seven;//客户端绘制七个图形
ext_client_custom_character_t              ext_client_custom_character;//客户端绘制字符
//-------------0x0301部分结束-------------------


//robot_interactive_data_t                 robot_interactive_data;交互数据接收信息(0x0302)
ext_robot_command_t                        ext_robot_command; //小地图下发信息标识(0x0303)/*发送频率：触发时发送.*/
ext_client_map_command_t                   ext_client_map_command; //小地图接收信息标识(0x0305)

frame_header_struct_t ext_referee_receive_header;
frame_header_struct_t ext_referee_send_header;

uint8_t USART6_dma[80];		//DMA接收数据
uint8_t Personal_Data[128];	//DMA发送数据
//extern SCM_rx_mes *SCM_rx_message;//超级电容数据
const u8 sof = 0xA5;  // 帧头
u8 custom_info_counter = 0;  // 自定义数据包序号
u8 seq = 0;  // 发过来的包序号
u8 seq_real = 0;

Bytes2Float bytes2float;  // flaot和字节互转
Bytes2U32 bytes2u32;  // flaot和u16互转

u8 referee_message[64];  // 完整数据帧存放, 理论44就够。
int message_byte_counter = 0;  // 字节存放位置计数
u8 blood_counter = 0;  // (debug)被打计数

int	shoot_seq = 0;//shoot number



uint32_t last_time_tick_1ms_1 = 0;
uint32_t time_interval_1 = 0;
char str1[30];
char str2[30];
int MY_CLIENT_ID = clientid_red_hero;
int MY_ROBOT_ID = robotid_red_hero;
bool UI_if_init = 0;
int UI_count = 0;
void init_referee_struct_data(void)
{
 //   memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&ext_referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&ext_game_status, 0, sizeof(ext_game_status_t));
    memset(&ext_game_result, 0, sizeof(ext_game_result_t));
    memset(&ext_game_robot_HP, 0, sizeof(ext_game_robot_HP_t));


    memset(&ext_event_data, 0, sizeof(ext_event_data_t));
    memset(&ext_supply_projectile_action, 0, sizeof(ext_supply_projectile_action_t));
//    memset(&ext_supply_projectile_booking, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&ext_referee_warning, 0, sizeof(ext_referee_warning_t));


    memset(&ext_game_robot_status, 0, sizeof(ext_game_robot_status_t));
    memset(&ext_power_heat_data, 0, sizeof(ext_power_heat_data_t));
    memset(&ext_game_robot_pos, 0, sizeof(ext_game_robot_pos_t));
    memset(&ext_buff, 0, sizeof(ext_buff));
    memset(&ext_aerial_robot_energy, 0, sizeof(aerial_robot_energy));
    memset(&ext_robot_hurt, 0, sizeof(ext_robot_hurt_t));
    memset(&ext_shoot_data, 0, sizeof(ext_shoot_data_t));
    memset(&ext_bullet_remaining, 0, sizeof(ext_bullet_remaining_t));


    memset(&ext_student_interactive_header_data, 0, sizeof(ext_student_interactive_header_data_t));



}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&ext_referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&ext_game_status, frame + index, sizeof(ext_game_status_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&ext_game_result, frame + index, sizeof(ext_game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&ext_game_robot_HP, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;


        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&ext_event_data, frame + index, sizeof(ext_event_data_t));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&ext_supply_projectile_action, frame + index, sizeof(ext_supply_projectile_action_t));
        }
        break;
//        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
//        {
//            memcpy(&ext_supply_projectile_booking, frame + index, sizeof(ext_supply_projectile_booking_t));
//        }
//        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&ext_referee_warning, frame + index, sizeof(ext_referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&ext_game_robot_status, frame + index, sizeof(ext_game_robot_status_t));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&ext_power_heat_data, frame + index, sizeof(ext_power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&ext_game_robot_pos, frame + index, sizeof(ext_game_robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&ext_buff, frame + index, sizeof(ext_buff_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&ext_aerial_robot_energy, frame + index, sizeof(aerial_robot_energy_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&ext_robot_hurt, frame + index, sizeof(ext_robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&ext_shoot_data, frame + index, sizeof(ext_shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&ext_bullet_remaining, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&ext_student_interactive_header_data, frame + index, sizeof(ext_student_interactive_header_data_t));
        }
        break;
        default:
        {
            break;
        }
    }
}

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = ext_power_heat_data.chassis_power;
    *buffer = ext_power_heat_data.chassis_power_buffer;

}


uint8_t get_robot_id(void)
{
    return ext_game_robot_status.robot_id;
}

//void UI_send_init() {
//	if (ext_power_heat_data.chassis_volt > 0) {
//		if (UI_if_init == 0) { //the first time to draw, need init
//			if (ext_game_robot_status.robot_id == robotid_red_hero) {
//				MY_CLIENT_ID = clientid_red_hero;
//				MY_ROBOT_ID = robotid_red_hero;
//			}
//			else if (ext_game_robot_status.robot_id == robotid_blue_hero) {
//				MY_CLIENT_ID = clientid_blue_hero;
//				MY_ROBOT_ID = robotid_blue_hero;
//			}

//			if (UI_count == 0) {
//				send_multi_graphic();//aim line
//				UI_count = 1;
//			}
//			else if (UI_count == 1) {
//				//		send_pitch_graphic(1); pitch angle keshihua
//				send_single_icon("x", 1800, 700, 1, 2); //CAP
//				UI_count = 2;
//			}

//			else if (UI_count == 2) {
//				send_single_icon("z", 1800, 800, 1, 4);
//				UI_count = 3;
//			}
//			//FRIC 
//			else if (UI_count == 3) {
//				send_single_icon("y", 1800, 750, 1, 4);
//				UI_count = 4;
//			}

//			else if (UI_count == 4) {
//				send_single_icon("w", 1800, 850, 1, 4);
//				UI_count = 5;
//			}
//			else if (UI_count == 5) {
//				UI_if_init = 1;
//				UI_count = 0;
//			}
//		}

//		else if (UI_if_init == 1)
//		{


//			if (UI_count == 0) {
//				//		send_pitch_graphic(2);
//				if (24000 < SCM_rx_message->cap_vol) {
//					send_single_icon("x", 1800, 700, 2, 2);
//				}
//				else if (21000 < SCM_rx_message->cap_vol) {
//					send_single_icon("x", 1800, 700, 2, 1);
//				}
//				else {
//					send_single_icon("x", 1800, 700, 2, 4);
//				}
//				UI_count = 1;
//			}

//			else if (UI_count == 1)
//			{
//				if (1) {//此处代表摩擦轮开了 frictionState == Friction_ON
//					send_single_icon("z", 1800, 800, 2, 2);
//				}
//				else {
//					send_single_icon("z", 1800, 800, 2, 4);
//				}
//				UI_count = 2;
//			}
//			//FRIC 
//			else if (UI_count == 2) {
//				if (1) {  //Feed_Mode == -1
//					send_single_icon("y", 1800, 750, 2, 4);
//				}
//				else {
//					send_single_icon("y", 1800, 750, 2, 2);
//				}
//				UI_count = 0;
//			}
////			else if (UI_count == 3) {
////				if ((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_Z) == KEY_PRESSED_OFFSET_Z) {
////					send_single_icon("w", 1800, 850, 2, 2);
////				}
////				else {
////					send_single_icon("w", 1800, 850, 2, 4);
////				}
////				UI_count = 0;
////			}

//		}
//	}
//}

void UI_send_update() {/*
	if (	UI_if_init == 1){
	send_pitch_graphic(2);

	memset(str1,' ',30);
	if (frictionState == Friction_ON){
		memcpy(str1,"FRIC__ON",8);
		send_string(str1,"z",1800,800,2,1);
	}
	else {
		memcpy(str1,"FRIC_OFF",8);
		send_string(str1,"z",1800,800,2,2);
	}


	memset(str2,' ',30);
	if (Feed_Mode == Feed_stuck){
		memcpy(str2,"STUCK!!!!",9);
		send_string(str2,"y",1800,750,2,2);
	}
	else if(Feed_Mode == Feed_singleshoot){
		memcpy(str2,"ONE_SHOOT",9);
		send_string(str2,"y",1800,750,2,1);
	}
	else if(Feed_Mode == Feed_clearbomb){
		memcpy(str2,"FULL_FIRE",9);
		send_string(str2,"y",1800,750,2,2);
	}
	else{
		memcpy(str2,"FEED_STOP",9);
		send_string(str2,"y",1800,750,2,1);
	}
}*/
}


void init_referee_info() {
	ext_game_robot_pos.x = 0;
	ext_game_robot_pos.y = 0;
	ext_game_robot_pos.z = 0;
	ext_game_robot_pos.yaw = 0;
	ext_power_heat_data.shooter_id1_17mm_cooling_heat = 0;
	ext_power_heat_data.shooter_id2_17mm_cooling_heat = 0;
	ext_power_heat_data.shooter_id1_42mm_cooling_heat = 0;
}

//crc8 生成多项式:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT2 = 0xff;
const unsigned char CRC8_TAB[256] = {
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};


//crc16 生成多项式: 未知
uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


/*
** Descriptions: CRC8 校验码生成
** Input: 信息数据, 信息数据长, CRC8_INIT2(0xff)
** Output: CRC8校验码
*/
unsigned char Get_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength, unsigned char ucCRC8) {
	unsigned char ucIndex;
	while (dwLength--) {
		ucIndex = ucCRC8 ^ (*pchMessage++);
		ucCRC8 = CRC8_TAB[ucIndex];
	}
	return(ucCRC8);
}


/*
** Descriptions: CRC8 校验程序
** Input: 需校验数据, 数据长度(= 信息长度 + 校验码长度)
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength) {
	unsigned char ucExpected = 0;
	if ((pchMessage == 0) || (dwLength <= 2))
		return false;
	ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT2);
	return (ucExpected == pchMessage[dwLength - 1]);  // 末位校验码判断
}


/*
** Descriptions: 增添 CRC8 至信息数据尾部
** Input: 信息数据(尾部需留空1位以加入校验码), 信息数据长度
** Output: 带校验码的数据
*/
void Append_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength) {
	unsigned char ucCRC = 0;
	if ((pchMessage == 0) || (dwLength <= 2))
		return;
	ucCRC = Get_CRC8_Check_Sum((unsigned char*)pchMessage, dwLength - 1, CRC8_INIT2);  // 校验码生成
	pchMessage[dwLength - 1] = ucCRC;  // 增添至尾部
}


/*
** Descriptions: CRC16 校验码生成
** Input: 信息数据, 信息数据长, CRC_INIT(0xffff)
** Output: CRC16校验码
*/
uint16_t Get_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC)
{
	uint8_t chData;
	if (pchMessage == NULL) {
		return 0xFFFF;
	}
	while (dwLength--) {
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
	}
	return wCRC;
}


/*
** Descriptions: CRC16 校验程序
** Input: 需校验数据, 数据长度(= 信息长度 + 校验码长度, 即包含了校验码的长度)
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength) {
	uint16_t wExpected = 0;
	if ((pchMessage == NULL) || (dwLength <= 2)) {
		return false;
	}
	wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}


/*
** Descriptions: 增添 CRC16 至信息数据尾部
** Input: 信息数据(尾部需留空2位以加入校验码), 信息数据长度
** Output: 带校验码的数据
*/
void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength) {
	uint16_t wCRC = 0;
	if ((pchMessage == NULL) || (dwLength <= 2)) {
		return;
	}
	wCRC = Get_CRC16_Check_Sum((u8*)pchMessage, dwLength - 2, CRC_INIT);
	pchMessage[dwLength - 2] = (u8)(wCRC & 0x00ff);
	pchMessage[dwLength - 1] = (u8)((wCRC >> 8) & 0x00ff);
}


///////////////////



// 单字节数组转u16(2字节), 高低位反序
uint16_t _bytes2u16(uint8_t* chosen_Message) {
	uint32_t temp = 0;
	// 长度为2，高低位逆序拼合
	temp = *chosen_Message;
	chosen_Message++;
	temp += (*chosen_Message << 8);
	return temp;
}


// 单字节数组转u32(4字节), 高低位未知
uint32_t _bytes4u32(uint8_t* chosen_Message) {
	bytes2u32.b[0] = chosen_Message[0];
	bytes2u32.b[1] = chosen_Message[1];
	bytes2u32.b[2] = chosen_Message[2];
	bytes2u32.b[3] = chosen_Message[3];
	return bytes2u32.u32_value;
}


// 单字节数组转float(4字节), 高低位正常
float _bytes2float(uint8_t* chosen_Message) {
	bytes2float.b[0] = chosen_Message[0];
	bytes2float.b[1] = chosen_Message[1];
	bytes2float.b[2] = chosen_Message[2];
	bytes2float.b[3] = chosen_Message[3];
	return bytes2float.f;
}
//0~7bit是从左往右还是从右往左？？如果是从右往左，这里是不是有问题？

// float转4长度单字节数组
void float2bytes(float chosen_value, u8* res_message) {
	int i;
	bytes2float.f = chosen_value;
	for (i = 0; i < 4; i++)
		res_message[i] = bytes2float.b[i];
}
// 比赛机器人状态（0x0001）, 发送频率为10Hz。
void ext_game_status_interpret(uint8_t* ext_game_status_Message)
{
	
	uint8_t* a;
	memcpy(a, ext_game_status_Message, 1);
	ext_game_status.game_type = (*a) >> 4;//取高四位
	ext_game_status.game_progress = (*a) & 0x000f;//取低四位

	memcpy((uint8_t*)&ext_game_status.stage_remain_time, ext_game_status_Message + 1, 2);
	memcpy((uint8_t*)&ext_game_status.SyncTimeStamp, ext_game_status_Message + 3, 8);
}

//比赛结果数据(0x0002)
void  ext_game_result_interpret(uint8_t* ext_game_result_t_Message)
{
	ext_game_result.winner = *ext_game_result_t_Message;

}

//机器人血量数据（0x0003）
void ext_game_robot_HP_interpret(uint8_t* ext_game_robot_HP_t_Message)
{
	//memcpy((uint8_t*)&ext_game_robot_HP.robot_legion,ext_game_robot_HP_t_Message,2);
	memcpy((uint8_t*)&ext_game_robot_HP, ext_game_robot_HP_t_Message, 2);

}

//人工智能挑战赛加成与惩罚区状态(0x0005)
void ext_ICRA_buff_debuff_zone_status_interpret(uint8_t* ext_ICRA_buff_debuff_zone_status_t_Message)
{
	uint8_t *F12,*F34,*F56;
	//F1，F2
	memcpy(F12, ext_ICRA_buff_debuff_zone_status_t_Message, 1);
	ext_ICRA_buff_debuff_zone_status.F1_zone_status = (*F12) >> 7;//取字节最高一位
	ext_ICRA_buff_debuff_zone_status.F1_zone_buff_debuff_status = (*F12) & 0x0070;//低8位：01110000
	ext_ICRA_buff_debuff_zone_status.F2_zone_status = (*F12) & 0x0008;            //与00001000相与，取1bit
	ext_ICRA_buff_debuff_zone_status.F2_zone_buff_debuff_status = (*F12) & 0x0007;//与00000111相与，取最低3bit
	//F3，F4
	memcpy(F34, ext_ICRA_buff_debuff_zone_status_t_Message+1, 1);
	ext_ICRA_buff_debuff_zone_status.F3_zone_status = (*F34) >> 7;//取字节最高一位
	ext_ICRA_buff_debuff_zone_status.F3_zone_buff_debuff_status = (*F34) & 0x0070;//低8位：01110000
	ext_ICRA_buff_debuff_zone_status.F4_zone_status = (*F34) & 0x0008;
	ext_ICRA_buff_debuff_zone_status.F4_zone_buff_debuff_status = (*F34) & 0x0007;
	//F5，F6
	memcpy(F56, ext_ICRA_buff_debuff_zone_status_t_Message+2, 1);
	ext_ICRA_buff_debuff_zone_status.F5_zone_status = (*F56) >> 7;//取字节最高一位
	ext_ICRA_buff_debuff_zone_status.F5_zone_buff_debuff_status = (*F56) & 0x0070;//低8位：01110000
	ext_ICRA_buff_debuff_zone_status.F6_zone_status = (*F56) & 0x0008;
	ext_ICRA_buff_debuff_zone_status.F6_zone_buff_debuff_status = (*F56) & 0x0007;


	memcpy((uint8_t*)&ext_ICRA_buff_debuff_zone_status.red1_bullet_left, ext_ICRA_buff_debuff_zone_status_t_Message + 3, 2);
	memcpy((uint8_t*)&ext_ICRA_buff_debuff_zone_status.red2_bullet_left, ext_ICRA_buff_debuff_zone_status_t_Message + 5, 2);
	memcpy((uint8_t*)&ext_ICRA_buff_debuff_zone_status.blue1_bullet_left, ext_ICRA_buff_debuff_zone_status_t_Message + 7, 2);
	memcpy((uint8_t*)&ext_ICRA_buff_debuff_zone_status.blue2_bullet_left, ext_ICRA_buff_debuff_zone_status_t_Message + 9, 2);

}


//场地时事件数据（0x0101）...........这个得重写，差的太多
//手册里说的什么占领状态啥的，在手册中都没有给提供变量，这不行啊；
void ext_event_data_interpret(uint8_t* ext_event_data_t_Message)
{
	memcpy((uint8_t*)&ext_event_data.event_type, ext_event_data_t_Message, 4);
}

//补给站动作标识（0x0102）
void ext_supply_projectile_action_interpret(uint8_t* ext_supply_projectile_action_Message)
{
	memcpy((uint8_t*)&ext_supply_projectile_action.supply_projectile_id, ext_supply_projectile_action_Message, 1);
	memcpy((uint8_t*)&ext_supply_projectile_action.supply_robot_id, ext_supply_projectile_action_Message + 1, 1);
	memcpy((uint8_t*)&ext_supply_projectile_action.supply_projectile_step, ext_supply_projectile_action_Message + 2, 1);
	memcpy((uint8_t*)&ext_supply_projectile_action.supply_projectile_num, ext_supply_projectile_action_Message + 3, 1);
}

//补给站预约子弹（0x0103）
//void ext_supply_projectile_booking_interpret(uint8_t * ext_supply_projectile_booking_Message)
//{
//	memcpy((uint8_t*)&ext_supply_projectile_booking.supply_projectile_id,ext_supply_projectile_booking_Message,1);
//	memcpy((uint8_t*)&ext_supply_projectile_booking.supply_num,ext_supply_projectile_booking_Message+1,1);
//}

//裁判警告信息：cmd_id(0x0104)
void ext_referee_warning_interpret(uint8_t* ext_referee_warning_t_Message)
{
	memcpy((uint8_t*)&ext_referee_warning.level, ext_referee_warning_t_Message, 1);
	memcpy((uint8_t*)&ext_referee_warning.foul_robot_id, ext_referee_warning_t_Message+1, 1);

}

//飞镖发射口倒计时：cmd_id (0x0105)
void ext_dart_remaining_time_interpret(uint8_t* ext_dart_remaining_time_t_Message)
{
	memcpy((uint8_t*)&ext_dart_remaining_time.dart_remaining_time, ext_dart_remaining_time_t_Message, 1);
}

//比赛机器人状态(0x0201)
void ext_game_robot_status_interpret(uint8_t* ext_game_robot_status_Message)
{
	memcpy((uint8_t*)&ext_game_robot_status.robot_id, ext_game_robot_status_Message, 1);
	memcpy((uint8_t*)&ext_game_robot_status.robot_level, ext_game_robot_status_Message + 1, 1);
	memcpy((uint8_t*)&ext_game_robot_status.remain_HP, ext_game_robot_status_Message + 2, 2);
	memcpy((uint8_t*)&ext_game_robot_status.max_HP, ext_game_robot_status_Message + 4, 2);
	memcpy((uint8_t*)&ext_game_robot_status.shooter_id1_17mm_cooling_rate, ext_game_robot_status_Message + 6, 2);
	memcpy((uint8_t*)&ext_game_robot_status.shooter_id1_17mm_cooling_limit, ext_game_robot_status_Message + 8, 2);
	memcpy((uint8_t*)&ext_game_robot_status.shooter_id1_17mm_speed_limit, ext_game_robot_status_Message + 10, 2);
	memcpy((uint8_t*)&ext_game_robot_status.shooter_id2_17mm_cooling_rate, ext_game_robot_status_Message + 12, 2);
	memcpy((uint8_t*)&ext_game_robot_status.shooter_id2_17mm_cooling_limit, ext_game_robot_status_Message + 14, 2);
	memcpy((uint8_t*)&ext_game_robot_status.shooter_id2_17mm_speed_limit, ext_game_robot_status_Message + 16, 2);
	memcpy((uint8_t*)&ext_game_robot_status.shooter_id1_42mm_cooling_rate, ext_game_robot_status_Message + 18, 2);
	memcpy((uint8_t*)&ext_game_robot_status.shooter_id1_42mm_cooling_limit, ext_game_robot_status_Message + 20, 2);
	memcpy((uint8_t*)&ext_game_robot_status.shooter_id1_42mm_speed_limit, ext_game_robot_status_Message + 22, 2);

	//这里还有机器人底盘功率限制上限没写
	//写了：
	memcpy((uint8_t*)&ext_game_robot_status.chassis_power_limit, ext_game_robot_status_Message + 24, 2);
	uint8_t a;
	//memcpy(&a, ext_game_robot_status_Message + 14, 1);/////////////////
	memcpy(&a, ext_game_robot_status_Message + 26, 1);
	ext_game_robot_status.mains_power_gimbal_output = a >> 7;
	ext_game_robot_status.mains_power_chassis_output = (a >> 6) & 0x0001;
	ext_game_robot_status.mains_power_shooter_output = (a >> 5) & 0x0001;

}

///////实时功率热量数据（0x0202）
void ext_power_heat_data_interpret(uint8_t* ext_power_heat_data_Message)
{

	memcpy((uint8_t*)&ext_power_heat_data.chassis_volt, ext_power_heat_data_Message, 2);
	ext_power_heat_data.chassis_volt;
	memcpy((uint8_t*)&ext_power_heat_data.chassis_current, ext_power_heat_data_Message + 2, 2);
	memcpy((float*)&ext_power_heat_data.chassis_power, ext_power_heat_data_Message + 4, 4);
	// float temp;
	//ext_power_heat_data.chassis_power=_bytes2float(ext_power_heat_data_Message+4);
	//ext_power_heat_data.chassis_power_buffer=_bytes2float(ext_power_heat_data_Message+8);
	memcpy((uint8_t*)&ext_power_heat_data.chassis_power_buffer, (ext_power_heat_data_Message + 8), 2);
	memcpy((uint8_t*)&ext_power_heat_data.shooter_id1_17mm_cooling_heat, ext_power_heat_data_Message + 10, 2);
	memcpy((uint8_t*)&ext_power_heat_data.shooter_id2_17mm_cooling_heat, ext_power_heat_data_Message + 12, 2);
	memcpy((uint8_t*)&ext_power_heat_data.shooter_id1_42mm_cooling_heat, ext_power_heat_data_Message + 14, 2);
}


////机器人位置（0x0203）
void ext_game_robot_pos_interpret(uint8_t* ext_game_robot_pos_Message)
{
	memcpy((uint8_t*)&ext_game_robot_pos.x, ext_game_robot_pos_Message, 4);
	memcpy((uint8_t*)&ext_game_robot_pos.y, ext_game_robot_pos_Message + 4, 4);
	memcpy((uint8_t*)&ext_game_robot_pos.z, ext_game_robot_pos_Message + 8, 4);
	memcpy((uint8_t*)&ext_game_robot_pos.yaw, ext_game_robot_pos_Message + 12, 4);

}

////机器人增益（0x0204）
void ext_buff_musk_interpret(uint8_t* ext_buff_musk_Message)
{
	memcpy((uint8_t*)&ext_buff.power_rune_buff, ext_buff_musk_Message, 1);

}

//空中机器人能量状态（0x0205）
void aerial_robot_energy_interpret(uint8_t* aerial_robot_energy_Message)
{
	
//	memcpy((uint8_t*)&ext_aerial_robot_energy.energy_point,aerial_robot_energy_Message,1);
    memcpy((uint8_t*)&ext_aerial_robot_energy.attack_time, aerial_robot_energy_Message + 1, 2);
}

////伤害状态（0x0206）
void  ext_robot_hurt_interpret(uint8_t* ext_robot_hurt_Message)
{
	uint8_t a;
	memcpy((uint8_t*)&a, ext_robot_hurt_Message, 1);

	ext_robot_hurt.armor_id = a >> 4;
	ext_robot_hurt.hurt_type = a & 0x000f;
}

////实时射击信息（0x0207）
void ext_shoot_data_interpret(uint8_t* ext_shoot_data_Message)
{

	memcpy((uint8_t*)&ext_shoot_data.bullet_type, ext_shoot_data_Message, 1);
	memcpy((uint8_t*)&ext_shoot_data.shooter_id, ext_shoot_data_Message + 1, 1);
	memcpy((uint8_t*)&ext_shoot_data.bullet_freq, ext_shoot_data_Message + 2, 1);
	memcpy((uint8_t*)&ext_shoot_data.bullet_speed, ext_shoot_data_Message + 3, 4);  //子弹射速

	shoot_seq++;
}

//子弹剩余发射数(0x0208)
void ext_bullet_remaining_interpret(uint8_t* ext_bullet_remaining_Message)
{
	memcpy((uint8_t*)&ext_bullet_remaining.remaining_num_17mm, ext_bullet_remaining_Message, 2);
	memcpy((uint8_t*)&ext_bullet_remaining.remaining_num_42mm, ext_bullet_remaining_Message + 2, 2);
	memcpy((uint8_t*)&ext_bullet_remaining.coin_remaining_num, ext_bullet_remaining_Message + 4, 2);
}

//机器人RFID状态(0x0209)
void ext_rfid_status_interpret(uint8_t* ext_rfid_status_t_Message)
{
	memcpy((uint8_t*)&ext_rfid_status.rfid_status, ext_rfid_status_t_Message, 4);
}

//非标机器人客户端指令数据(0x020A)
void ext_dart_client_cmd_interpret(uint8_t* ext_dart_client_cmd_Message)
{
	memcpy((uint8_t*)&ext_dart_client_cmd.dart_launch_opening_status, ext_dart_client_cmd_Message, 1);
	memcpy((uint8_t*)&ext_dart_client_cmd.dart_attack_target, ext_dart_client_cmd_Message+1, 1);
	memcpy((uint8_t*)&ext_dart_client_cmd.target_change_time, ext_dart_client_cmd_Message + 2, 2);
	memcpy((uint8_t*)&ext_dart_client_cmd.operate_launch_cmd_time, ext_dart_client_cmd_Message + 4, 2);
}

////交互数据接收信息（0x0301）
//先实现随着内容ID（data_cmd_id）变化的各内容数据段的读取函数，最后利用switch
//统一在ext_student_interactive_header_data_interpret函数中读取。

//内容ID：0x0200~0x02FF:己方机器人间通信
void robot_interactive_data_interpret(uint8_t* robot_interactive_data_Message)
{
	memcpy((uint8_t*)robot_interactive_data.data, robot_interactive_data_Message, sizeof(robot_interactive_data.data));
}

//内容ID：0x0100:客户端删除图形
void ext_client_custom_graphic_delete_interpret(uint8_t* ext_client_custom_graphic_delete_Message)
{
	memcpy((uint8_t*)&ext_client_custom_graphic_delete.operate_tpye, ext_client_custom_graphic_delete_Message, 1);
	memcpy((uint8_t*)&ext_client_custom_graphic_delete.layer, ext_client_custom_graphic_delete_Message+1, 1);
}

//内容ID：0x0101:客户端绘制一个图形
void ext_client_custom_graphic_single_interpret(uint8_t* ext_client_custom_graphic_single_Message)
{
	memcpy((uint8_t*)&ext_client_custom_graphic_single.graphic_data_struct, ext_client_custom_graphic_single_Message, 15 * 1);
}
//内容ID：0x0102:客户端绘制二个图形
void ext_client_custom_graphic_double_interpret(uint8_t* ext_client_custom_graphic_double_Message)
{
	memcpy((uint8_t*)ext_client_custom_graphic_double.graphic_data_struct, ext_client_custom_graphic_double_Message, 15 * 2);
}
//内容ID：0x0103:客户端绘制五个图形
void ext_client_custom_graphic_five_interpret(uint8_t* ext_client_custom_graphic_five_Message)
{
	memcpy((uint8_t*)ext_client_custom_graphic_five.graphic_data_struct, ext_client_custom_graphic_five_Message, 15 * 5);
}
//内容ID：0x0104:客户端绘制七个图形
void ext_client_custom_graphic_seven_interpret(uint8_t* ext_client_custom_graphic_seven_Message)
{
	memcpy((uint8_t*)ext_client_custom_graphic_seven.graphic_data_struct, ext_client_custom_graphic_seven_Message, 15 * 7);
}
//内容ID：0x0110:客户端绘制字符图形
void ext_client_custom_character_interpret(uint8_t* ext_client_custom_character_Message)
{
	memcpy((uint8_t*)&ext_client_custom_character.graphic_data_struct, ext_client_custom_character_Message, 15);
	memcpy((uint8_t*)ext_client_custom_character.data, ext_client_custom_character_Message+15, 30);

}
//
void ext_student_interactive_header_data_interpret(uint8_t* ext_student_interactive_header_data_Message)
{
	memcpy((uint8_t*)&ext_student_interactive_header_data.data_cmd_id, ext_student_interactive_header_data_Message, 2);//数据段的内容ID
	memcpy((uint8_t*)&ext_student_interactive_header_data.sender_ID, ext_student_interactive_header_data_Message + 2, 2);
	memcpy((uint8_t*)&ext_student_interactive_header_data.receiver_ID, ext_student_interactive_header_data_Message + 4, 2);
	uint8_t*  content= ext_student_interactive_header_data_Message + 6;
	switch (ext_student_interactive_header_data.data_cmd_id)
	{
		//内容ID：0x0100:客户端删除图形
		case 0x0100:ext_client_custom_graphic_delete_interpret(content); break;
		//内容ID：0x0101:客户端绘制一个图形
		case 0x0101:ext_client_custom_graphic_single_interpret(content); break;
		//内容ID：0x0102:客户端绘制二个图形
		case 0x0102:ext_client_custom_graphic_double_interpret(content); break;
		//内容ID：0x0103:客户端绘制五个图形
		case 0x0103:ext_client_custom_graphic_five_interpret(content); break;
		//内容ID：0x0104:客户端绘制七个图形
		case 0x0104:ext_client_custom_graphic_seven_interpret(content); break;
		//内容ID：0x0110:客户端绘制字符图形
		case 0x0110:ext_client_custom_character_interpret(content); break;
		default:  break;
	};
	//内容ID：0x0200~0x02FF:己方机器人间通信
	if (ext_student_interactive_header_data.data_cmd_id > 0x0199 && ext_student_interactive_header_data.data_cmd_id < 0x0300)
	{
		robot_interactive_data_interpret(content);
	}
//	memcpy((uint8_t*)&ext_student_interactive_header_data.blood_value, ext_student_interactive_header_data_Message + 6, 2);
}
//////////////////////////////////////////////////
//客户端自定义数据（0x0301），内容ID：data_cmd(0xD180)
//void client_custom_data_interpret(uint8_t * client_custom_data_Message)
//{
//	memcpy((uint8_t*)&client_custom_data.data1,client_custom_data_Message+6,4);
//	memcpy((uint8_t*)&client_custom_data.data2,client_custom_data_Message+10,4);
//	memcpy((uint8_t*)&client_custom_data.data3,client_custom_data_Message+14,4);
//	memcpy((uint8_t*)&client_custom_data.masks,client_custom_data_Message+18,1);
//	
//}

//void robot_interactive_data_interpret(uint8_t * robot_interactive_data_Message)
//{
	//memcpy(&robot_interactive_data.data,robot_interactive_data_Message,sizeof(data));

//}

////交互数据接收信息（0x0302）
//void robot_interactive_data_interpret(uint8_t* robot_interactive_data_Message)
//{
//	memcpy((uint8_t*)robot_interactive_data.data, robot_interactive_data_Message, 30);
//}

////交互数据接收信息（0x0303）

void ext_robot_command_interpret03(uint8_t * ext_robot_command_Message)
{
	memcpy((float*)&ext_robot_command.target_position_x, ext_robot_command_Message, 4);
	memcpy((float*)&ext_robot_command.target_position_y, ext_robot_command_Message+4, 4);
	memcpy((float*)&ext_robot_command.target_position_z, ext_robot_command_Message + 8, 4);
	memcpy((uint8_t*)&ext_robot_command.commd_keyboard, ext_robot_command_Message + 12, 1);
	memcpy((uint8_t*)&ext_robot_command.target_robot_ID, ext_robot_command_Message + 13, 2);

}

////图传遥控信息标识（0x0304）
void ext_robot_command_interpret04(uint8_t * ext_robot_command_Message)
{
	memcpy((uint8_t*)&ext_robot_command.mouse_x, ext_robot_command_Message, 2);
	memcpy((uint8_t*)&ext_robot_command.mouse_y, ext_robot_command_Message+2, 2);
	memcpy((uint8_t*)&ext_robot_command.mouse_z, ext_robot_command_Message + 4, 2);
	memcpy((uint8_t*)&ext_robot_command.left_button_down, ext_robot_command_Message + 6, 1);
	memcpy((uint8_t*)&ext_robot_command.right_button_down, ext_robot_command_Message + 7, 1);
	memcpy((uint8_t*)&ext_robot_command.keyboard_value, ext_robot_command_Message + 8, 2);
	memcpy((uint8_t*)&ext_robot_command.reserved, ext_robot_command_Message + 10, 2);
}

////小地图接收信息标识(0x0305)
void ext_client_map_command_interpret(uint8_t* ext_client_map_command_Message)
{
	memcpy((uint8_t*)&ext_client_map_command.target_robot_ID, ext_client_map_command_Message, 2);
	memcpy((float*)&ext_client_map_command.target_position_x, ext_client_map_command_Message+2, 4);
	memcpy((float*)&ext_client_map_command.target_position_y, ext_client_map_command_Message+6, 4);

}



// 完整校验数据帧, CRC8和CRC16
u8 Verify_frame(uint8_t* frame) {
	int frame_length;
	if (frame[0] != sof) return false;
	frame_length = _bytes2u16(&frame[1]) + 5 + 2 + 2;
	if (Verify_CRC8_Check_Sum(frame, 5) && Verify_CRC16_Check_Sum(frame, frame_length)) {
		//		if(seq > frame[3]) {
		//			return false;
		//		}
		//		else {
		//			seq = frame[3];
		//			return true;
		//		}
		return true;
	}
	else {
		return false;
	}
}

//u8 blink_time=0;
//// 数字闪烁显示测试
//void data_blink(void) {
//	blink_time++;
//	if(blink_time<60) {
////		blink_time=0;
//		extShowData.data3=8;
//		return;
//	}
//	if(blink_time<120) {
//		extShowData.data3 = 8.8888888888888888f;
//		return;
//	}
//	if(blink_time>120)  {
//		blink_time=0;
//		return;
//	}
//}

void update_from_dma(void) {
	//int i;
	u8 USART6_dma_x2[2 * USART6_dma_rx_len];
	memcpy(USART6_dma_x2, USART6_dma, USART6_dma_rx_len);
	memcpy(USART6_dma_x2 + USART6_dma_rx_len, USART6_dma, USART6_dma_rx_len);

	//	for(i=0;i<USART6_dma_rx_len;i++) {
	//		frame_interpret(USART6_dma_x2+i);
	//	}
	frame_interpret(USART6_dma_x2, 2 * USART6_dma_rx_len);
	return;
}


bool vrerify_frame(uint8_t* frame) {
	/* frame length = header(5) + cmd_id(2) + data(n) + crc16(2) */
	uint16_t frame_length = 5 + 2 + _bytes2u16(&frame[1]) + 2;
	return (Verify_CRC8_Check_Sum(frame, sizeof(ext_frame_header_t))
		&& Verify_CRC16_Check_Sum(frame, frame_length));
}
bool frame_interpret(uint8_t* _frame, uint16_t size) {
	for (uint16_t i = 0; i < size; i++) {
		if (_frame[i] == REFEREE_FRAME_HEADER_SOF && vrerify_frame(&_frame[i])) {
			//            static uint8_t seq_ex = 0;
			//            uint8_t seq = frame[3];
			//            if(frame[3]<=seq_ex && (seq_ex-frame[3])>=240) {
			//                continue;
			//            }
			//            seq_ex = seq;
			uint8_t* frame = &_frame[i];
			uint16_t length = (uint16_t)(frame[1] | (frame[2] << 8));
			i += length + 7;

			ext_cmd_id_t cmd_id = (ext_cmd_id_t)(frame[5] | (frame[6] << 8));

			//u8 blood_seq=0;
			//// 使用完整数据帧更新全部裁判信息相关结构体。(带校验)
			//u8 frame_interpret(uint8_t * frame) { 
			//    if(Verify_frame(frame) == true) {
			//		if(seq_real>=frame[3]&&(seq_real-frame[3])<100) {
			//			return false;
			//		}
			////		offical_data_flag=0;
			//		memcpy(&cmd_id,&frame[5],2);
			//    //cmdID = _bytes2u16(&frame[5]);/////////////////////////////
			//		seq_real=frame[3];
			switch (cmd_id) {
			case 0x0001:ext_game_status_interpret(&frame[7]); break;
			case 0x0002:ext_game_result_interpret(&frame[7]); break;
			case 0x0003:ext_game_robot_HP_interpret(&frame[7]); break;
			case 0x0101:ext_event_data_interpret(&frame[7]); break;
			case 0x0102:ext_supply_projectile_action_interpret(&frame[7]); break;
				//			  	case 0x0103:ext_supply_projectile_booking_interpret(&frame[7]);break;
			case 0x0201:ext_game_robot_status_interpret(&frame[7]); break;
			case 0x0202:ext_power_heat_data_interpret(&frame[7]); break;
			case 0x0203:ext_game_robot_pos_interpret(&frame[7]); break;
			case 0x0204:ext_buff_musk_interpret(&frame[7]); break;
			case 0x0205:aerial_robot_energy_interpret(&frame[7]); break;
			case 0x0206:ext_robot_hurt_interpret(&frame[7]); break;
			case 0x0207:ext_shoot_data_interpret(&frame[7]); break;
			case 0x0301:ext_student_interactive_header_data_interpret(&frame[7]); break;

				// case 1: extGameRobotState_interpret(&frame[7]);break;  // 比赛进程信息
				 //case 2: extRobotHurt_interpret(&frame[7]);break; // 伤害数据
				 //case 3: extShootData_interpret(&frame[7]);break;  // 实时射击数据
							 //case 4: extPowerHeatData_interpret(&frame[7]);break;  //实时功率热量数据
						   //case 8: extGameRobotPos_interpret(&frame[7]);break;  //// 机器人位置和枪口朝向信息
				 //case 5:StudentPropInfo_interpret(&frame[7]);break;  //赛场信息
			default: break;
			}
			// return true;
		}
		//	else {
		//		return false;
		//	}
	}
	return true;
}

//void referee_send_client_graphic(ext_id_t target_id, graphic_data_struct_t* graphic_draw) {

//	static ext_robot_graphic_data_t robot_data;

//	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
//	robot_data.header.seq++;
//	robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
//	Append_CRC8_Check_Sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));

//	robot_data.cmd_id = student_interactive_header;
//	robot_data.data_id = 0x0101;
//	robot_data.sender_id = MY_ROBOT_ID;
//	robot_data.robot_id = target_id;
//	// memcpy(&robot_data.graphic_data ,(uint8_t*)&graphic_draw, sizeof(robot_data.graphic_data));
//	robot_data.graphic_data = *graphic_draw;
//	Append_CRC16_Check_Sum((uint8_t*)&robot_data, sizeof(robot_data));
//	//spDMA.mem2mem.copy((uint32_t)robot_data.data, (uint32_t)graphic_draw, size);
///* Send out data */
////		return spDMA.controller.start(spDMA_USART6_tx_stream, 
////        (uint32_t)&robot_data, (uint32_t)USART6->DR, sizeof(robot_data));
//	memcpy(Personal_Data, (u8*)&robot_data, sizeof(robot_data));
//	RefereeSend(sizeof(robot_data));
//}

//void referee_send_client_character(ext_id_t target_id, ext_client_custom_character_t* character_data) {


//	static ext_robot_character_data_t robot_data;

//	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
//	robot_data.header.seq++;
//	robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
//	Append_CRC8_Check_Sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));

//	robot_data.cmd_id = student_interactive_header;
//	robot_data.data_id = 0x0110;
//	robot_data.sender_id = MY_ROBOT_ID;
//	robot_data.robot_id = target_id;
//	// memcpy(&robot_data.graphic_data ,(uint8_t*)&graphic_draw, sizeof(robot_data.graphic_data));
//	robot_data.character_data = *character_data;
//	Append_CRC16_Check_Sum((uint8_t*)&robot_data, sizeof(robot_data));
//	//spDMA.mem2mem.copy((uint32_t)robot_data.data, (uint32_t)graphic_draw, size);
///* Send out data */
////		return spDMA.controller.start(spDMA_USART6_tx_stream, 
////        (uint32_t)&robot_data, (uint32_t)USART6->DR, sizeof(robot_data));
//	memcpy(Personal_Data, (u8*)&robot_data, sizeof(robot_data));
//	RefereeSend(sizeof(robot_data));
//}

//void referee_send_multi_graphic(ext_id_t target_id, ext_client_custom_graphic_seven_t* graphic_draw) {

//	static ext_robot_sev_graphic_data_t robot_data;

//	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
//	robot_data.header.seq++;
//	robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
//	Append_CRC8_Check_Sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));

//	robot_data.cmd_id = student_interactive_header;
//	robot_data.data_id = 0x0104;
//	robot_data.sender_id = MY_ROBOT_ID;
//	robot_data.robot_id = target_id;
//	// memcpy(&robot_data.graphic_data ,(uint8_t*)&graphic_draw, sizeof(robot_data.graphic_data));
//	robot_data.graphic_data = *graphic_draw;
//	Append_CRC16_Check_Sum((uint8_t*)&robot_data, sizeof(robot_data));
//	//spDMA.mem2mem.copy((uint32_t)robot_data.data, (uint32_t)graphic_draw, size);
///* Send out data */
////		return spDMA.controller.start(spDMA_USART6_tx_stream, 
////        (uint32_t)&robot_data, (uint32_t)USART6->DR, sizeof(robot_data));
//	memcpy(Personal_Data, (u8*)&robot_data, sizeof(robot_data));
//	RefereeSend(sizeof(robot_data));
//}

//static bool first_cap_draw = true;

//void send_cap_graphic(int residue_power)
//{
//	graphic_data_struct_t graphic_draw;
//	//float res = (float)residue_power/1800.0*100.0;
//	int res = residue_power;
//	res = res < 0 ? 0 : (res > 100 ? 100 : res);
//	char* name = "c";
//	graphic_draw.graphic_tpye = 0;

//	if (first_cap_draw) {
//		graphic_draw.operate_tpye = 1;
//		first_cap_draw = false;
//	}
//	else {
//		graphic_draw.operate_tpye = 2;
//	}
//	//graphic_draw.operate_tpye = 1;
//	graphic_draw.layer = 0;
//	if (res > 80)
//	{
//		graphic_draw.color = 2;
//	}
//	else if (res > 50)
//	{
//		graphic_draw.color = 1;
//	}
//	else if (res > 20)
//	{
//		graphic_draw.color = 3;
//	}
//	else
//	{
//		graphic_draw.color = 5;
//	}
//	graphic_draw.width = 30;
//	//graphic_draw.start_angle=10;
//	//graphic_draw.end_angle=10;
//	graphic_draw.start_x = 800;
//	graphic_draw.start_y = 100;
//	graphic_draw.end_x = graphic_draw.start_x + res + 5;
//	graphic_draw.end_y = graphic_draw.start_y;
//	//graphic_draw.radius = 30;
//	//graphic_draw.text_lenght = strlen(value);
//	memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));

//	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
//}

//void send_pitch_graphic(int type)
//{
//	graphic_data_struct_t graphic_draw;
//	//float res = (float)residue_power/1800.0*100.0;
//	int pitchang = (MIDDLE_PITCH - CAN2_206.Current_position) * 360 / 8192.0;
////	if (frictionState == Friction_ON) {
//	if(1){
//		graphic_draw.color = 1;
//	}
//	else {
//		graphic_draw.color = 4;
//	}
//	char* name = "a";
//	/*	graphic_draw.graphic_tpye = 0;

//		if(first_cap_draw){
//			graphic_draw.operate_tpye = 1;
//			first_cap_draw = false;
//		}
//		else{
//			graphic_draw.operate_tpye = 2;
//		}*/
//	graphic_draw.operate_tpye = type;
//	graphic_draw.layer = 0;


//	graphic_draw.width = 10;
//	//graphic_draw.start_angle=10;
//	//graphic_draw.end_angle=10;
//	graphic_draw.start_x = 1600;
//	graphic_draw.start_y = 70;
//	graphic_draw.end_x = graphic_draw.start_x;
//	graphic_draw.end_y = graphic_draw.start_y + 10;//+pitchang*10;
//	//graphic_draw.radius = 30;
//	//graphic_draw.text_lenght = strlen(value);
//	memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));

//	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
//}

//void send_single_icon(char* name, int x, int y, int type, int color)
//{
//	graphic_data_struct_t graphic_draw;
//	//float res = (float)residue_power/1800.0*100.0;
////	int pitchang=(MIDDLE_PITCH-CAN2_206.Current_position)*360/8192.0;
//	graphic_draw.color = color;

//	/*	graphic_draw.graphic_tpye = 0;

//		if(first_cap_draw){
//			graphic_draw.operate_tpye = 1;
//			first_cap_draw = false;
//		}
//		else{
//			graphic_draw.operate_tpye = 2;
//		}*/
//	graphic_draw.operate_tpye = type;
//	graphic_draw.layer = 0;


//	graphic_draw.width = 20;
//	//graphic_draw.start_angle=10;
//	//graphic_draw.end_angle=10;
//	graphic_draw.start_x = x;
//	graphic_draw.start_y = y;
//	graphic_draw.end_x = graphic_draw.start_x;
//	graphic_draw.end_y = graphic_draw.start_y + 20;//+pitchang*10;
//	//graphic_draw.radius = 30;
//	//graphic_draw.text_lenght = strlen(value);
//	memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));

//	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
//}


static bool first_multi_draw = true;
//void send_multi_graphic()
//{
//	ext_client_custom_graphic_seven_t graphic_draw;
//	//graph 1
//	char* name1 = "c";
//	graphic_draw.graphic_data_struct[0].graphic_tpye = 0;
//	if (first_multi_draw)
//	{
//		static bool first_multi_draw = false;
//		graphic_draw.graphic_data_struct[0].operate_tpye = 1;
//		graphic_draw.graphic_data_struct[1].operate_tpye = 1;
//		graphic_draw.graphic_data_struct[2].operate_tpye = 1;
//		graphic_draw.graphic_data_struct[3].operate_tpye = 1;
//		graphic_draw.graphic_data_struct[4].operate_tpye = 1;
//		graphic_draw.graphic_data_struct[5].operate_tpye = 1;
//		graphic_draw.graphic_data_struct[6].operate_tpye = 1;
//	}

//	else
//	{
//		graphic_draw.graphic_data_struct[0].operate_tpye = 2;
//		graphic_draw.graphic_data_struct[1].operate_tpye = 2;
//		graphic_draw.graphic_data_struct[2].operate_tpye = 2;
//		graphic_draw.graphic_data_struct[3].operate_tpye = 2;
//		graphic_draw.graphic_data_struct[4].operate_tpye = 2;
//		graphic_draw.graphic_data_struct[5].operate_tpye = 2;
//		graphic_draw.graphic_data_struct[6].operate_tpye = 2;
//	}

//	graphic_draw.graphic_data_struct[0].layer = 0;
//	graphic_draw.graphic_data_struct[0].color = 1;
//	graphic_draw.graphic_data_struct[0].width = 3;
//	//graphic_draw.graphic_data_struct[0].start_angle=10;
//	//graphic_draw.graphic_data_struct[0].end_angle=10;
//	graphic_draw.graphic_data_struct[0].start_x = 960;
//	graphic_draw.graphic_data_struct[0].start_y = 200;
//	graphic_draw.graphic_data_struct[0].end_x = 960;
//	graphic_draw.graphic_data_struct[0].end_y = 540;
//	//graphic_draw.graphic_data_struct[0].radius = 30;
//	memcpy(graphic_draw.graphic_data_struct[0].graphic_name, (uint8_t*)name1, strlen(name1));
//	//graph 2
//	char* name2 = "d";
//	graphic_draw.graphic_data_struct[1].graphic_tpye = 0;
//	//graphic_draw.graphic_data_struct[1].operate_tpye = 1;
//	graphic_draw.graphic_data_struct[1].layer = 0;
//	graphic_draw.graphic_data_struct[1].color = 1;
//	graphic_draw.graphic_data_struct[1].width = 3;
//	//graphic_draw.graphic_data_struct[1].start_angle=10;
//	//graphic_draw.graphic_data_struct[1].end_angle=10;
//	graphic_draw.graphic_data_struct[1].start_x = 850;
//	graphic_draw.graphic_data_struct[1].start_y = 540;
//	graphic_draw.graphic_data_struct[1].end_x = 1070;
//	graphic_draw.graphic_data_struct[1].end_y = 540;
//	//graphic_draw.graphic_data_struct[1].radius = 30;
//	memcpy(graphic_draw.graphic_data_struct[1].graphic_name, (uint8_t*)name2, strlen(name2));
//	//graph 3
//	char* name3 = "e";
//	graphic_draw.graphic_data_struct[2].graphic_tpye = 0;
//	//graphic_draw.graphic_data_struct[2].operate_tpye = 1;
//	graphic_draw.graphic_data_struct[2].layer = 0;
//	graphic_draw.graphic_data_struct[2].color = 1;
//	graphic_draw.graphic_data_struct[2].width = 3;
//	//graphic_draw.graphic_data_struct[2].start_angle=10;
//	//graphic_draw.graphic_data_struct[2].end_angle=10;
//	graphic_draw.graphic_data_struct[2].start_x = 860;
//	graphic_draw.graphic_data_struct[2].start_y = 520;
//	graphic_draw.graphic_data_struct[2].end_x = 1060;
//	graphic_draw.graphic_data_struct[2].end_y = 520;
//	//graphic_draw.graphic_data_struct[2].radius = 30;
//	memcpy(graphic_draw.graphic_data_struct[2].graphic_name, (uint8_t*)name3, strlen(name3));
//	//graph 4
//	char* name4 = "f";
//	graphic_draw.graphic_data_struct[3].graphic_tpye = 0;
//	//graphic_draw.graphic_data_struct[3].operate_tpye = 1;
//	graphic_draw.graphic_data_struct[3].layer = 0;
//	graphic_draw.graphic_data_struct[3].color = 1;
//	graphic_draw.graphic_data_struct[3].width = 3;
//	//graphic_draw.graphic_data_struct[3].start_angle=10;
//	//graphic_draw.graphic_data_struct[3].end_angle=10;
//	graphic_draw.graphic_data_struct[3].start_x = 890;
//	graphic_draw.graphic_data_struct[3].start_y = 480;
//	graphic_draw.graphic_data_struct[3].end_x = 1030;
//	graphic_draw.graphic_data_struct[3].end_y = 480;
//	//graphic_draw.graphic_data_struct[3].radius = 30;
//	memcpy(graphic_draw.graphic_data_struct[3].graphic_name, (uint8_t*)name4, strlen(name4));
//	//graph 5
//	char* name5 = "g";
//	graphic_draw.graphic_data_struct[4].graphic_tpye = 0;
//	//graphic_draw.graphic_data_struct[4].operate_tpye = 1;
//	graphic_draw.graphic_data_struct[4].layer = 0;
//	graphic_draw.graphic_data_struct[4].color = 1;
//	graphic_draw.graphic_data_struct[4].width = 3;
//	//graphic_draw.graphic_data_struct[4].start_angle=10;
//	//graphic_draw.graphic_data_struct[4].end_angle=10;
//	graphic_draw.graphic_data_struct[4].start_x = 920;
//	graphic_draw.graphic_data_struct[4].start_y = 410;
//	graphic_draw.graphic_data_struct[4].end_x = 1000;
//	graphic_draw.graphic_data_struct[4].end_y = 410;
//	//graphic_draw.graphic_data_struct[4].radius = 30;
//	memcpy(graphic_draw.graphic_data_struct[4].graphic_name, (uint8_t*)name5, strlen(name5));
//	//graph 6
//	char* name6 = "h";
//	graphic_draw.graphic_data_struct[5].graphic_tpye = 0;
//	//graphic_draw.graphic_data_struct[5].operate_tpye = 1;
//	graphic_draw.graphic_data_struct[5].layer = 0;
//	graphic_draw.graphic_data_struct[5].color = 1;
//	graphic_draw.graphic_data_struct[5].width = 3;
//	//graphic_draw.graphic_data_struct[5].start_angle=10;
//	//graphic_draw.graphic_data_struct[5].end_angle=10;
//	graphic_draw.graphic_data_struct[5].start_x = 940;
//	graphic_draw.graphic_data_struct[5].start_y = 300;
//	graphic_draw.graphic_data_struct[5].end_x = 980;
//	graphic_draw.graphic_data_struct[5].end_y = 300;
//	//graphic_draw.graphic_data_struct[5].radius = 30;
//	memcpy(graphic_draw.graphic_data_struct[5].graphic_name, (uint8_t*)name6, strlen(name6));
//	//graph 7
//	char* name7 = "i";
//	graphic_draw.graphic_data_struct[6].graphic_tpye = 0;
//	//graphic_draw.graphic_data_struct[6].operate_tpye = 1;
//	graphic_draw.graphic_data_struct[6].layer = 0;
//	graphic_draw.graphic_data_struct[6].color = 1;
//	graphic_draw.graphic_data_struct[6].width = 3;
//	//graphic_draw.graphic_data_struct[6].start_angle=10;
//	//graphic_draw.graphic_data_struct[6].end_angle=10;
//	graphic_draw.graphic_data_struct[6].start_x = 960;
//	graphic_draw.graphic_data_struct[6].start_y = 541;
//	graphic_draw.graphic_data_struct[6].end_x = 960;
//	graphic_draw.graphic_data_struct[6].end_y = 550;
//	//graphic_draw.graphic_data_struct[6].radius = 30;
//	memcpy(graphic_draw.graphic_data_struct[6].graphic_name, (uint8_t*)name7, strlen(name7));

//	referee_send_multi_graphic(MY_CLIENT_ID, &graphic_draw);
//}


//void Send_Middle_rectangle(int level, int color, int x_length, int y_length)
//{
//	graphic_data_struct_t graphic_draw2;
//	char name_temp = (char)(level + 30);
//	char* name = &name_temp;
//	graphic_draw2.operate_tpye = 1;
//	graphic_draw2.graphic_tpye = 2;
//	graphic_draw2.layer = level;
//	graphic_draw2.color = color;
//	graphic_draw2.width = 2;
//	graphic_draw2.start_x = (int)(960 - x_length / 2.0f);
//	graphic_draw2.start_y = (int)(540 - y_length / 2.0f);
//	graphic_draw2.end_x = (int)(960 + x_length / 2.0f);
//	graphic_draw2.end_y = (int)(540 + y_length / 2.0f);
//	memcpy(graphic_draw2.graphic_name, (uint8_t*)name, strlen(name));
//	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw2);
//}

static bool first_string_draw = true;
//void send_string(char* str, char* name, int x, int y, int upd, int colour)
//{
//	int len = strlen(str) + 1;
//	ext_client_custom_character_t character_data;
//	character_data.graphic_data_struct.graphic_tpye = 7;//string
///*	if(first_string_draw)
//	{
//		character_data.graphic_data_struct.operate_tpye = 1;//add
//		first_string_draw=false;
//	}

//	else*/
//	character_data.graphic_data_struct.operate_tpye = upd;//modify

//	character_data.graphic_data_struct.layer = 0;

//	character_data.graphic_data_struct.color = colour;//green

//	character_data.graphic_data_struct.width = 2;
//	character_data.graphic_data_struct.start_angle = 10;//size
//	character_data.graphic_data_struct.end_angle = len;//length
//	character_data.graphic_data_struct.start_x = x;
//	character_data.graphic_data_struct.start_y = y;

//	memcpy(character_data.data, (uint8_t*)str, len);
//	memcpy(character_data.graphic_data_struct.graphic_name, (uint8_t*)name, strlen(name));

//	referee_send_client_character(MY_CLIENT_ID, &character_data);
//}


//void Send_SOS(void)
//{
//	graphic_data_struct_t graphic_draw;
//	char* name = "s";
//	char* value = "SOS";
//	graphic_draw.operate_tpye = 1;
//	graphic_draw.graphic_tpye = 0;
//	graphic_draw.layer = 0;
//	graphic_draw.color = 5;
//	graphic_draw.width = 5;
//	graphic_draw.start_angle = 0;
//	graphic_draw.end_angle = 0;
//	graphic_draw.start_x = 300;
//	graphic_draw.start_y = 300;
//	graphic_draw.end_x = 600;
//	graphic_draw.end_y = 600;
//	graphic_draw.radius = 70;
//	//graphic_draw.text_lenght = strlen(value);
//	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));
//	//memcpy(graphic_draw.text, (uint8_t*)value, strlen(value));
//	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
//}
