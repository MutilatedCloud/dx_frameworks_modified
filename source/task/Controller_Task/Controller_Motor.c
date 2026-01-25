#include "Controller_Motor.h"

#include "cmsis_os2.h"
#include "string.h"

#include "Controller_Task.h"

/**
 * @brief 初始化控制器使用的 DJI 电机结构体参数（配置 CAN port、汇总 ID、各电机 ID 等）。
 * @param Controller_Motor_DJI 3508/2006 电机组（二级指针，要求 *Controller_Motor_DJI 已分配内存）。
 * @param Controller_Motor_6020 6020 电机组（二级指针，要求 *Controller_Motor_6020 已分配内存）。
 * @note 本函数不会分配内存，只对外部传入的结构体做清零与参数初始化。
 * @note 初始化完成后会调用 Motor_DJI_Init() 注册/初始化底层电机接口。
 */
void Controller_Motor_Init(DJI_motor_t **Controller_Motor_DJI, DJI_motor_t **Controller_Motor_6020)
{
    if (Controller_Motor_DJI == NULL || Controller_Motor_6020 == NULL)
    {
        return;
    }

    if (*Controller_Motor_DJI == NULL)
    {
        return;
    }

    memset(*Controller_Motor_DJI, 0, sizeof(DJI_motor_t));
    (*Controller_Motor_DJI)->can_cfg.port = CAN1_PORT;
    (*Controller_Motor_DJI)->can_cfg.id = Controller_Motor_DJI_All_ID;
    (*Controller_Motor_DJI)->motor_msg[Controller_Motor_3508_Joint_2].can_msg.id = Controller_Motor_3508_Joint_2_ID;
    (*Controller_Motor_DJI)->motor_msg[Controller_Motor_2006_Joint_3].can_msg.id = Controller_Motor_2006_Joint_3_ID;
    (*Controller_Motor_DJI)->motor_msg[Controller_Motor_2006_Joint_4].can_msg.id = Controller_Motor_2006_Joint_4_ID;
    (*Controller_Motor_DJI)->motor_msg[Controller_Motor_2006_Joint_5].can_msg.id = Controller_Motor_2006_Joint_5_ID;
    Motor_DJI_Init(*Controller_Motor_DJI);

    if (*Controller_Motor_6020 == NULL)
    {
        return;
    }

    memset(*Controller_Motor_6020, 0, sizeof(DJI_motor_t));
    (*Controller_Motor_6020)->can_cfg.port = CAN1_PORT;
    (*Controller_Motor_6020)->can_cfg.id = Controller_Motor_6020_All_ID;
    (*Controller_Motor_6020)->motor_msg[Controller_Motor_6020_Joint_0].can_msg.id = Controller_Motor_6020_Joint_0_ID;
    (*Controller_Motor_6020)->motor_msg[Controller_Motor_6020_Joint_1].can_msg.id = Controller_Motor_6020_Joint_1_ID;
    Motor_DJI_Init(*Controller_Motor_6020);
}

/**
 * @brief 刷新电机角度信息并写入关节角数组。
 * @param Controller_Motor_DJI 3508/2006 电机组。
 * @param Controller_Motor_6020 6020 电机组。
 * @param Joint_Angle 输出关节角数组，长度为 6。
 * @note 输出单位：度（°）。
 * @note 关节顺序：
 *       - Joint_Angle[0..1]：6020 两个关节
 *       - Joint_Angle[2..5]：3508/2006 四个关节
 * @note 当前映射规则：motor_angle(0~8191) * Trans_Angle(≈0.04395°) + 180。
 */
void Controller_Angle_Refresh(DJI_motor_t *Controller_Motor_DJI, DJI_motor_t *Controller_Motor_6020, float32_t *Joint_Angle)
{
    Motor_DJI_Refresh(Controller_Motor_DJI);
    Motor_DJI_Refresh(Controller_Motor_6020);

    for (int i = 0; i < 4; i++)
    {
        Joint_Angle[i + 2] = Controller_Motor_DJI->motor_msg[i].motor_angle * Trans_Angle + 180;
    }

    for (int i = 0; i < 2; i++)
    {
        Joint_Angle[i] = Controller_Motor_6020->motor_msg[i].motor_angle * Trans_Angle + 180;
    }
}

/**
 * @brief 判断电机数据是否已收到（用于上电后等待 CAN 数据有效）。
 * @param Controller_Motor_DJI 3508/2006 电机组。
 * @param Controller_Motor_6020 6020 电机组。
 * @return 1: 所有电机均至少收到过 1 次报文；0: 仍有电机未收到。
 * @note 判定依据：motor_msg[i].can_msg.cnt 是否大于 0。
 */
uint8_t Controller_Motor_Data_Ready(DJI_motor_t *Controller_Motor_DJI, DJI_motor_t *Controller_Motor_6020)
{
    if (Controller_Motor_DJI == NULL || Controller_Motor_6020 == NULL)
    {
        return 0;
    }

    for (int i = 0; i < 4; i++)
    {
        if (Controller_Motor_DJI->motor_msg[i].can_msg.cnt == 0)
        {
            return 0;
        }
    }

    for (int i = 0; i < 2; i++)
    {
        if (Controller_Motor_6020->motor_msg[i].can_msg.cnt == 0)
        {
            return 0;
        }
    }

    return 1;
}

/**
 * @brief 等待电机数据有效并记录零点角度。
 * @param Controller_Motor_DJI 3508/2006 电机组。
 * @param Controller_Motor_6020 6020 电机组。
 * @param Joint_Angle 临时关节角数组（长度 6，用于中间采样，单位：度）。
 * @param Angle_Zero_Point 输出零点角数组（长度 6，单位：度）。
 * @note 超时时间约 300 * 10ms = 3s；超时后也会强制采样一次作为零点。
 * @note 零点定义：上电后第一次“数据有效时刻”的关节角读数。
 */
void Controller_Wait_And_Capture_Zero(DJI_motor_t *Controller_Motor_DJI,
                                     DJI_motor_t *Controller_Motor_6020,
                                     float32_t *Joint_Angle,
                                     float32_t *Angle_Zero_Point)
{
    for (int t = 0; t < 300; t++)
    {
        Controller_Angle_Refresh(Controller_Motor_DJI, Controller_Motor_6020, Joint_Angle);
        if (Controller_Motor_Data_Ready(Controller_Motor_DJI, Controller_Motor_6020))
        {
            break;
        }
        osDelay(10);
    }

    Controller_Angle_Refresh(Controller_Motor_DJI, Controller_Motor_6020, Joint_Angle);
    for (int i = 0; i < 6; i++)
    {
        Angle_Zero_Point[i] = Joint_Angle[i];
    }
}
