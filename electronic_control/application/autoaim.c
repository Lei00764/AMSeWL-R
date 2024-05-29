/*
 * @Author: TJSP_2022_TY
 * @Date:   2021-11-28
 * @Last Modified by: XiaoYoung
 * @Last Modified time: 2023-03-02
 * @brief: port from RM_standard_robot remote_control. uart1_dma_rx has been reconfigured in Cube. use double dma buffer. Added Kalmanfilter for prediction.
 */

#include "autoaim.h"
#include "gimbal_task.h"
#include "stm32f4xx_hal.h"
#include "CRC8_CRC16.h"
#include "arm_math.h"
#include "AHRS_middleware.h"
#include "bsp_usart.h"
#include "referee.h"
#include "CAN_receive.h"
#define AUTOAIM_FRAME_RX_LEN 10
#define AUTOAIM_FRAME_RX_BUF AUTOAIM_FRAME_RX_LEN * 2
#define AUTOAIM_FRAME_TX_LEN 11
#define AUTOAIM_FRAME_HEAD 0xf1
#define AUTOAIM_FRAME_TAIL 0xf2

#pragma pack(1)
typedef struct
{
    uint8_t head;       // 1 byte
    int16_t x_in_imu;   // 2 byte
    int16_t y_in_imu;   // 2 byte
    int16_t z_in_imu;   // 2 byte
    uint8_t flag;       // 1 byte
    uint8_t crc8_check; // 1 byte
    uint8_t tail;       // 1 byte
} frame_rx_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    uint8_t head;         // 1 byte
    uint8_t stamp;        // 1 byte
    int16_t yaw;          // 2 byte
    int16_t pitch;        // 2 byte
    int16_t bullet_speed; // 2 byte
    uint8_t flag;         // 1 byte
    uint8_t crc8_check;   // 1 byte
    uint8_t tail;         // 1 byte
} frame_tx_t;
#pragma pack()

typedef struct
{
    fp32 x_in_imu;  // mm
    fp32 y_in_imu;  // mm
    fp32 z_in_imu;  // mm
    uint8_t flag;
    uint8_t outdated_count;
} autoaim_target_t;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t autoaim_frame_tx_buf[AUTOAIM_FRAME_TX_LEN];
uint8_t autoaim_frame_rx_buf[2][AUTOAIM_FRAME_RX_BUF];
frame_rx_t frame_rx;
frame_tx_t frame_tx;
autoaim_target_t autoaim_target;

void usart1_rx_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
uint8_t unpack_frame(uint8_t *autoaim_buf);
void pack_frame(uint8_t *buff, frame_tx_t *frame);

void autoaim_target_reset(void)
{
    autoaim_target.x_in_imu = 0.0f;
    autoaim_target.y_in_imu = 0.0f;
    autoaim_target.z_in_imu = 0.0f;
    autoaim_target.flag = AUTOAIM_EMPTY_FLAG;
    autoaim_target.outdated_count = 100;
}

void autoaim_init(void)
{
    autoaim_target_reset();
    usart1_rx_dma_init(autoaim_frame_rx_buf[0], autoaim_frame_rx_buf[1], AUTOAIM_FRAME_RX_BUF);
}

uint8_t get_autoaim_flag(void)
{
    uint8_t flag = autoaim_target.flag;
    //autoaim_target.flag = AUTOAIM_EMPTY_FLAG;
    return flag;
}

void set_autoaim_angle(fp32 *add_yaw_set, fp32 *add_pitch_set, fp32 absolute_yaw_set, fp32 absolute_pitch_set)
{
    if (autoaim_target.outdated_count < 100)
    {
        autoaim_target.outdated_count++;
        fp32 target_yaw_in_imu = 0.0f;
        fp32 target_pitch_in_imu = 0.0f;

        // 电控角度正方向：
        // yaw：操作手视角下，枪管向左为正方向
        // pitch: 抬枪为正方向
        fp32 xz_length;
        arm_sqrt_f32(autoaim_target.x_in_imu * autoaim_target.x_in_imu + autoaim_target.z_in_imu * autoaim_target.z_in_imu, &xz_length);
        target_yaw_in_imu = -atan2(autoaim_target.x_in_imu, autoaim_target.z_in_imu);
        target_pitch_in_imu = -atan2(autoaim_target.y_in_imu, xz_length);

        *add_yaw_set = target_yaw_in_imu - absolute_yaw_set;
        *add_pitch_set = -(target_pitch_in_imu - absolute_pitch_set);
    }
    else
    {
        *add_yaw_set = 0.0f;
        *add_pitch_set = 0.0f;
				autoaim_target.flag = AUTOAIM_EMPTY_FLAG;
    }
}

void usart1_rx_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // 使能DMA串口接收
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    // 失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);

    // 缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);

    // 缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);

    // 数据长度
    hdma_usart1_rx.Instance->NDTR = dma_buf_num;

    // 使能双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    // 使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}

// 中断服务函数
//void USART1_IRQHandler(void)
//{
//    if (huart1.Instance->SR & UART_FLAG_RXNE) // 接收数据中
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart1);
//    }
//    else if (USART1->SR & UART_FLAG_IDLE) // 数据接收完毕
//    {
//        static uint16_t this_time_fram_len = 0;

//        __HAL_UART_CLEAR_PEFLAG(&huart1);

//        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            // 缓冲区1
//            // 失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            // 获取接收数据长度，长度 = 设定长度 - 剩余长度
//            this_time_fram_len = AUTOAIM_FRAME_RX_BUF - hdma_usart1_rx.Instance->NDTR;

//            // 重新设定数据长度
//            hdma_usart1_rx.Instance->NDTR = AUTOAIM_FRAME_RX_BUF;

//            // 设定缓冲区2
//            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

//            // 使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);


//                unpack_frame(autoaim_frame_rx_buf[0]);

//        }
//        else
//        {
//            // 缓冲区2
//            // 失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            // 获取接收数据长度，长度 = 设定长度 - 剩余长度
//            this_time_fram_len = AUTOAIM_FRAME_RX_BUF - hdma_usart1_rx.Instance->NDTR;

//            // 重新设定数据长度
//            hdma_usart1_rx.Instance->NDTR = AUTOAIM_FRAME_RX_BUF;

//            // 设定缓冲区1
//            hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);

//            // 使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);


//                unpack_frame(autoaim_frame_rx_buf[1]);
//        }
//    }
//}

uint8_t unpack_frame(uint8_t *autoaim_buf)
{
    memcpy((uint8_t *)&frame_rx, autoaim_buf, AUTOAIM_FRAME_RX_LEN);
    uint8_t crc8_check = get_CRC8_check_sum((uint8_t *)&frame_rx, AUTOAIM_FRAME_RX_LEN - 2, 0xff);
    if (frame_rx.head != AUTOAIM_FRAME_HEAD || frame_rx.crc8_check != crc8_check)
    {
        return 0;
    }

    autoaim_target.x_in_imu = (fp32)frame_rx.x_in_imu;
    autoaim_target.y_in_imu = (fp32)frame_rx.y_in_imu;
    autoaim_target.z_in_imu = (fp32)frame_rx.z_in_imu;
    autoaim_target.flag = frame_rx.flag;
    autoaim_target.outdated_count = 0; // 新鲜出炉的数据
    return 1;
}

static uint8_t send_count = 0;
static uint8_t stamp_count = 0;
void send_to_computer(fp32 absolute_yaw, fp32 absolute_pitch)
{
    send_count++;

    // 每3ms向上位机发一次数据
    if (send_count != 3) {
        return;
    }
    
    frame_tx.head = AUTOAIM_FRAME_HEAD;
    frame_tx.tail = AUTOAIM_FRAME_TAIL;
    frame_tx.stamp = stamp_count;
    frame_tx.yaw = (int16_t)(absolute_yaw * 180.f / PI * 1e2f);
    frame_tx.pitch = (int16_t)(absolute_pitch * 180.f / PI * 1e2f);
    frame_tx.bullet_speed = (int16_t)(0 * 1e2f);
    frame_tx.flag = 0;

    if (stamp_count == 255)
        stamp_count = 0;
    else
        stamp_count += 1;

    pack_frame(autoaim_frame_tx_buf, &frame_tx);
    usart1_tx_dma_enable(autoaim_frame_tx_buf, AUTOAIM_FRAME_TX_LEN);

	send_count = 0;
}

void pack_frame(uint8_t *buff, frame_tx_t *frame)
{
    memcpy(buff, frame, AUTOAIM_FRAME_TX_LEN);
    buff[AUTOAIM_FRAME_TX_LEN - 2] = get_CRC8_check_sum((uint8_t *)frame, AUTOAIM_FRAME_TX_LEN - 2, 0xff);
}
