#ifndef __CONTROLLER_MOTOR_H__
#define __CONTROLLER_MOTOR_H__

#include <stdint.h>

#include "arm_math_types.h"
#include "motor_DJI.h"

void Controller_Motor_Init(DJI_motor_t **Controller_Motor_DJI, DJI_motor_t **Controller_Motor_6020);

void Controller_Angle_Refresh(DJI_motor_t *Controller_Motor_DJI,
                             DJI_motor_t *Controller_Motor_6020,
                             float32_t *Joint_Angle);

uint8_t Controller_Motor_Data_Ready(DJI_motor_t *Controller_Motor_DJI, DJI_motor_t *Controller_Motor_6020);

void Controller_Wait_And_Capture_Zero(DJI_motor_t *Controller_Motor_DJI,
                                     DJI_motor_t *Controller_Motor_6020,
                                     float32_t *Joint_Angle,
                                     float32_t *Angle_Zero_Point);

#endif
