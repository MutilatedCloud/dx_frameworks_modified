#ifndef __CONTROLLER_TASK_H__
#define __CONTROLLER_TASK_H__

#include <stdint.h>
#include "arm_math_types.h"

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

/**
 * @brief 控制器发送给外部的关节角调试数据（单位：弧度）。
 */
extern float32_t test_angle[6];

/**
 * @brief 控制器 UART 发送缓冲区（长度为 CONTROLLER_UART_DATA_LEN）。
 */
extern uint8_t Uart_Send_Buffer[CONTROLLER_UART_DATA_LEN];

/**
 * @brief 控制器主任务入口（FreeRTOS 线程函数）。
 */
void Controller_Task(void *argument);

#endif
