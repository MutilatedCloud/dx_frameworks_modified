#ifndef __CONTROLLER_MOTOR_H__
#define __CONTROLLER_MOTOR_H__

#include <stdint.h>

#include "arm_math_types.h"
#include "motor_DJI.h"

/**
 * @brief 初始化控制器使用的 DJI 电机结构体参数（配置 CAN ID 等）。
 * @param Controller_Motor_DJI 3508/2006 电机组结构体指针（二级指针，外部已分配内存）。
 * @param Controller_Motor_6020 6020 电机组结构体指针（二级指针，外部已分配内存）。
 */
void Controller_Motor_Init(DJI_motor_t **Controller_Motor_DJI, DJI_motor_t **Controller_Motor_6020);

/**
 * @brief 刷新电机角度信息并写入关节角数组（单位：度）。
 * @param Controller_Motor_DJI 3508/2006 电机组。
 * @param Controller_Motor_6020 6020 电机组。
 * @param Joint_Angle 输出关节角数组，长度为 6。
 */
void Controller_Angle_Refresh(DJI_motor_t *Controller_Motor_DJI,
                             DJI_motor_t *Controller_Motor_6020,
                             float32_t *Joint_Angle);

/**
 * @brief 判断电机数据是否已收到（用于上电后等待 CAN 数据有效）。
 * @param Controller_Motor_DJI 3508/2006 电机组。
 * @param Controller_Motor_6020 6020 电机组。
 * @return 1: 数据已准备好；0: 未准备好。
 */
uint8_t Controller_Motor_Data_Ready(DJI_motor_t *Controller_Motor_DJI, DJI_motor_t *Controller_Motor_6020);

/**
 * @brief 等待电机数据有效并记录零点角度。
 * @param Controller_Motor_DJI 3508/2006 电机组。
 * @param Controller_Motor_6020 6020 电机组。
 * @param Joint_Angle 临时关节角数组。
 * @param Angle_Zero_Point 输出零点角数组，长度为 6。
 */
void Controller_Wait_And_Capture_Zero(DJI_motor_t *Controller_Motor_DJI,
                                     DJI_motor_t *Controller_Motor_6020,
                                     float32_t *Joint_Angle,
                                     float32_t *Angle_Zero_Point);

#endif
