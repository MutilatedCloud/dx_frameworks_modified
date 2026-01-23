#ifndef __CONTROLLER_DEBUGGER_H__
#define __CONTROLLER_DEBUGGER_H__

#include <stdint.h>

#include "arm_math_types.h"

/**
 * @brief 控制器调试数据：六个关节当前相对角度（单位：弧度）。
 * @note 该结构体实例为全局变量，供调试器直接观察。
 */
typedef struct
{
    float32_t joint_angle[6];
} controller_dbg_joint_t;

/**
 * @brief 控制器调试数据：当前待发送的 UART 数据（发送缓冲区的镜像）。
 * @note 该结构体实例为全局变量，供调试器直接观察。
 */
typedef struct
{
    uint8_t tx_buf[24];
} controller_dbg_uart_t;

/**
 * @brief 控制器调试状态：用于判断任务是否运行、调试数据是否在更新。
 * @note 该结构体实例为全局变量，供调试器直接观察。
 */
typedef struct
{
    uint32_t init_done_cnt;
    uint32_t step_cnt;
    uint32_t stage;
} controller_dbg_stat_t;

/**
 * @brief 六关节相对角度调试数据（全局可观测）。
 */
extern volatile controller_dbg_joint_t g_controller_dbg_joint;

/**
 * @brief 控制器 UART 发送数据调试镜像（全局可观测）。
 */
extern volatile controller_dbg_uart_t g_controller_dbg_uart;

/**
 * @brief 控制器调试状态（全局可观测）。
 */
extern volatile controller_dbg_stat_t g_controller_dbg_stat;

/**
 * @brief 更新六关节相对角度调试数据。
 * @param joint_angle 输入数组，长度为 6。
 */
void Controller_Debugger_Update_JointAngle(const float32_t *joint_angle);

/**
 * @brief 更新 UART 发送数据调试镜像。
 * @param tx_buf 输入数组。
 * @param len 输入长度（建议为 CONTROLLER_UART_DATA_LEN）。
 */
void Controller_Debugger_Update_TxBuf(const uint8_t *tx_buf, uint16_t len);

/**
 * @brief 通知：Controller_Task 完成初始化。
 */
void Controller_Debugger_Notify_InitDone(void);

/**
 * @brief 通知：Controller_Task 完成一次主循环。
 */
void Controller_Debugger_Notify_Step(void);

#endif
