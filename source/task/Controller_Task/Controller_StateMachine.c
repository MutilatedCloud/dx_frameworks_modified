#include "Controller_StateMachine.h"

#include "Controller_Debugger.h"

#include "gpio.h"

volatile uint32_t g_controller_pa0_irq_cnt = 0;
volatile uint8_t g_controller_pa0_irq_pending = 0;

typedef struct
{
    uint8_t raw_level; /**< PA0 原始电平：HAL_GPIO_ReadPin 直接读取的 0/1。 */
    uint8_t debounced_level; /**< PA0 去抖后电平：稳定后的 0/1。 */
    uint8_t last_debounced_level; /**< 上一次去抖后的电平，用于沿检测。 */
    uint8_t stable_cnt; /**< 去抖稳定计数：原始电平持续与去抖值不同的累计次数。 */
    controller_sm_state_t state; /**< 控制器状态机当前状态。 */
    uint32_t toggle_cnt; /**< 状态翻转计数：按“去抖后的下降沿”计数。 */
} controller_sm_ctx_t;

static controller_sm_ctx_t g_sm_ctx = {0};

/**
 * @brief 初始化控制器开关状态机。
 * @note 该函数会读取一次 PA0 当前电平作为初值，并清零去抖/计数器。
 * @note PA0 默认上拉：未按下为 1，按下接地为 0（下降沿表示按下）。
 */
void Controller_StateMachine_Init(void)
{
    g_sm_ctx.raw_level = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    g_sm_ctx.debounced_level = g_sm_ctx.raw_level;
    g_sm_ctx.last_debounced_level = g_sm_ctx.raw_level;
    g_sm_ctx.stable_cnt = 0;
    g_sm_ctx.state = (g_sm_ctx.debounced_level == 0U) ? CONTROLLER_SM_ACTIVE : CONTROLLER_SM_IDLE;
    g_sm_ctx.toggle_cnt = 0;

    g_controller_dbg_switch.pa0_raw_level = g_sm_ctx.raw_level;
    g_controller_dbg_switch.pa0_debounced_level = g_sm_ctx.debounced_level;
    g_controller_dbg_switch.pa0_irq_pending = g_controller_pa0_irq_pending;
    g_controller_dbg_switch.sm_state = (uint8_t)g_sm_ctx.state;
    g_controller_dbg_switch.irq_cnt = g_controller_pa0_irq_cnt;
    g_controller_dbg_switch.toggle_cnt = g_sm_ctx.toggle_cnt;
}

/**
 * @brief 状态机单步更新：采样 PA0、去抖、检测按键下降沿并切换状态。
 * @note 去抖规则：当原始电平连续 3 次采样与当前去抖值不同，则认为电平翻转生效。
 * @note 触发规则：使用“去抖后的下降沿(1->0)”作为一次按键有效触发，进行 IDLE/ACTIVE 互相切换。
 * @note 副作用：会更新 g_controller_dbg_switch 供调试器观察，并清除 g_controller_pa0_irq_pending。
 */
void Controller_StateMachine_Step(void)
{
    uint8_t raw = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    g_sm_ctx.raw_level = raw;

    if (raw == g_sm_ctx.debounced_level)
    {
        g_sm_ctx.stable_cnt = 0;
    }
    else
    {
        if (g_sm_ctx.stable_cnt < 255)
        {
            g_sm_ctx.stable_cnt++;
        }

        if (g_sm_ctx.stable_cnt >= 3)
        {
            g_sm_ctx.debounced_level = raw;
            g_sm_ctx.stable_cnt = 0;
        }
    }

    controller_sm_state_t new_state = (g_sm_ctx.debounced_level == 0U) ? CONTROLLER_SM_ACTIVE : CONTROLLER_SM_IDLE;
    if (new_state != g_sm_ctx.state)
    {
        g_sm_ctx.state = new_state;
        g_sm_ctx.toggle_cnt++;
    }
    g_sm_ctx.last_debounced_level = g_sm_ctx.debounced_level;

    g_controller_dbg_switch.pa0_raw_level = g_sm_ctx.raw_level;
    g_controller_dbg_switch.pa0_debounced_level = g_sm_ctx.debounced_level;
    g_controller_dbg_switch.pa0_irq_pending = g_controller_pa0_irq_pending;
    g_controller_dbg_switch.sm_state = (uint8_t)g_sm_ctx.state;
    g_controller_dbg_switch.irq_cnt = g_controller_pa0_irq_cnt;
    g_controller_dbg_switch.toggle_cnt = g_sm_ctx.toggle_cnt;

    g_controller_pa0_irq_pending = 0;
}

/**
 * @brief 获取当前状态机状态。
 * @return controller_sm_state_t 当前状态。
 */
controller_sm_state_t Controller_StateMachine_GetState(void)
{
    return g_sm_ctx.state;
}
