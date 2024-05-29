/*
 * @Author: TJSP_2022_TY
 * @Date:   2021-11-28
 * @Last Modified by: XiaoYoung
 * @Last Modified time: 2023-03-02
 * @brief: port from RM_standard_robot remote_control. uart1_dma_rx has been reconfigured in Cube. use double dma buffer. Added Kalmanfilter for prediction.
 */

#ifndef __AUTOAIM_H__
#define __AUTOAIM_H__

#include "struct_typedef.h"

#define AUTOAIM_EMPTY_FLAG 0
#define AUTOAIM_FIRE_FLAG 1

void autoaim_target_reset(void);
uint8_t get_autoaim_flag(void);
void autoaim_init(void);
void set_autoaim_angle(fp32 *add_yaw_set, fp32 *add_pitch_set, fp32 absolute_yaw_set, fp32 absolute_pitch_set);
void send_to_computer(fp32 absolute_yaw, fp32 absolute_pitch);

#endif
