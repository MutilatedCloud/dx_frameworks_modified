#ifndef __CONTROLLER_TASK_H__
#define __CONTROLLER_TASK_H__

#include <stdint.h>
#include "arm_math_types.h"
#include "motor_DJI.h"
#include "uart_api.h"

#define Trans_Angle (360.0f / 8191.0f)

#define Controller_Motor_6020_Joint_0 0
#define Controller_Motor_6020_Joint_1 1

#define Controller_Motor_3508_Joint_2 0
#define Controller_Motor_2006_Joint_3 1
#define Controller_Motor_2006_Joint_4 2
#define Controller_Motor_2006_Joint_5 3

#define Controller_Motor_3508_Joint_2_ID 0X201
#define Controller_Motor_2006_Joint_3_ID 0X202
#define Controller_Motor_2006_Joint_4_ID 0X203
#define Controller_Motor_2006_Joint_5_ID 0X204

#define Controller_Motor_6020_Joint_0_ID 0X205
#define Controller_Motor_6020_Joint_1_ID 0X206

#define Controller_Motor_DJI_All_ID 0x200
#define Controller_Motor_6020_All_ID 0x1FF
#define Controller_Motor_2006_All_ID 0x200

#define CONTROLLER_UART_DATA_LEN 24

#define FRAME_HEADER_LENGTH 5 // 帧头数据长度
#define CMD_ID_LENGTH 2       // 命令码ID数据长度
#define DATA_LENGTH 26        // 数据段长度
#define FRAME_TAIL_LENGTH 2   // 帧尾数据长度

#define DATA_FRAME_LENGTH (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + DATA_LENGTH + FRAME_TAIL_LENGTH) // 整个数据帧的长度

#define CONTROLLER_CMD_ID 0x0302 // 自定义控制器命令码

typedef __packed struct
{
    __packed struct
    {
        uint8_t sof;              // 起始字节，固定值为0xA5
        uint16_t data_length;     // 数据帧中data的长度
        uint8_t seq;              // 包序号
        uint8_t crc8;             // 帧头CRC8校验
    } frame_header;               // 帧头
    __packed uint16_t cmd_id;     // 命令码
    __packed uint8_t data[DATA_LENGTH];    // 自定义控制器的数据帧
    __packed uint16_t frame_tail; // 帧尾CRC16校验
} Controller_t;                   // 自定义控制器数据包

void Controller_Task(void *argument);
void Motor_Init_DJI(DJI_motor_t **Controller_Motor_DJI, DJI_motor_t **Controller_Motor_6020);
void Controller_Angle_Refresh(DJI_motor_t *Controller_Motor_DJI, DJI_motor_t *Controller_Motor_6020, float32_t *Joint_Angle);
void UART_Message_Trans(float32_t float_array[], uint8_t out_buf[]);
uart_status_t UART_Message_Parse(const uint8_t in_buf[], float32_t out_array[]);
void Controller_Uart_tx_init(uart_msg_t *tx_msg, uint8_t *tx_buf);
void Controller_Wait_And_Capture_Zero(DJI_motor_t *Controller_Motor_DJI,DJI_motor_t *Controller_Motor_6020,float32_t *Joint_Angle,float32_t *Angle_Zero_Point);
uint8_t Controller_Motor_Data_Ready(DJI_motor_t *Controller_Motor_DJI, DJI_motor_t *Controller_Motor_6020);

#endif
