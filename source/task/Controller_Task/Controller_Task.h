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

#define CONTROLLER_UART_DATA_LEN 26

extern float32_t test_angle[6];

extern uint8_t Uart_Send_Buffer[CONTROLLER_UART_DATA_LEN];

/* Joint5 sign flag for extended payload byte (0 = non-negative, 1 = negative). */
extern volatile uint8_t g_controller_joint5_sign;

void Controller_Task(void *argument);

#endif
