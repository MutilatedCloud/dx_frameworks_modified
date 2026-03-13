#ifndef __CONTROLLER_DEBUGGER_H__
#define __CONTROLLER_DEBUGGER_H__

#include <stdint.h>

#include "arm_math_types.h"
#include "Controller_Task.h"

/* Debug mirror length for the actual transmitted payload (26 bytes data + 4 bytes padding/sign/etc). */
#define CONTROLLER_DEBUG_TX_LEN 30

typedef struct
{
    float32_t joint_angle[6]; /*!< 六关节相对角度，单位：弧度；索引 0~5 对应关节 0~5。 */
} controller_dbg_joint_t;

typedef struct
{
    uint8_t tx_buf[CONTROLLER_DEBUG_TX_LEN]; /*!< UART 发送数据镜像（用于观察完整发送 payload）。 */
} controller_dbg_uart_t;

typedef struct
{
    uint32_t init_done_cnt; /*!< Controller_Task 初始化完成计数（初始化成功后 +1）。 */
    uint32_t step_cnt; /*!< Controller_Task 主循环心跳计数（每次 Step +1）。 */
    uint32_t stage; /*!< Controller_Task 阶段码（用于定位卡在哪个阶段）。 */
} controller_dbg_stat_t;

typedef struct
{
    uint8_t pa0_raw_level; /*!< PA0 原始电平：HAL_GPIO_ReadPin 直接读到的 0/1。 */
    uint8_t pa0_debounced_level; /*!< PA0 去抖后电平：0/1。 */
    uint8_t pa0_irq_pending; /*!< PA0 中断挂起标志：1 表示曾触发中断但任务未清零。 */
    uint8_t sm_state; /*!< 状态机状态：与 controller_sm_state_t 枚举值一致（0=IDLE，1=ACTIVE）。 */
    uint32_t irq_cnt; /*!< PA0 EXTI0 中断回调累计次数。 */
    uint32_t toggle_cnt; /*!< 状态机翻转次数：按“去抖后的下降沿”计数。 */
} controller_dbg_switch_t;

extern volatile controller_dbg_joint_t g_controller_dbg_joint;

extern volatile controller_dbg_uart_t g_controller_dbg_uart;

extern volatile controller_dbg_stat_t g_controller_dbg_stat;

extern volatile controller_dbg_switch_t g_controller_dbg_switch;

void Controller_Debugger_Update_JointAngle(const float32_t *joint_angle);

void Controller_Debugger_Update_TxBuf(const uint8_t *tx_buf, uint16_t len);

void Controller_Debugger_Notify_InitDone(void);

void Controller_Debugger_Notify_Step(void);

#endif
