#include "Controller_Debugger.h"

/**
 * @brief 六关节相对角度调试数据。
 */
volatile controller_dbg_joint_t g_controller_dbg_joint = {0};

/**
 * @brief 控制器 UART 发送数据调试镜像。
 */
volatile controller_dbg_uart_t g_controller_dbg_uart = {0};

/**
 * @brief 控制器调试状态（全局可观测）。
 */
volatile controller_dbg_stat_t g_controller_dbg_stat = {0};

/**
 * @brief 更新六关节相对角度调试数据。
 */
void Controller_Debugger_Update_JointAngle(const float32_t *joint_angle)
{
    if (joint_angle == NULL)
    {
        return;
    }

    for (int i = 0; i < 6; i++)
    {
        g_controller_dbg_joint.joint_angle[i] = joint_angle[i];
    }
}

/**
 * @brief 更新 UART 发送数据调试镜像。
 */
void Controller_Debugger_Update_TxBuf(const uint8_t *tx_buf, uint16_t len)
{
    if (tx_buf == NULL)
    {
        return;
    }

    if (len > sizeof(g_controller_dbg_uart.tx_buf))
    {
        len = (uint16_t)sizeof(g_controller_dbg_uart.tx_buf);
    }

    for (uint16_t i = 0; i < len; i++)
    {
        g_controller_dbg_uart.tx_buf[i] = tx_buf[i];
    }
}

/**
 * @brief 通知：Controller_Task 完成初始化。
 */
void Controller_Debugger_Notify_InitDone(void)
{
    g_controller_dbg_stat.init_done_cnt++;
}

/**
 * @brief 通知：Controller_Task 完成一次主循环。
 */
void Controller_Debugger_Notify_Step(void)
{
    g_controller_dbg_stat.step_cnt++;
}
