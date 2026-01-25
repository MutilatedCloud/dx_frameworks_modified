#ifndef __CONTROLLER_STATE_MACHINE_H__
#define __CONTROLLER_STATE_MACHINE_H__

#include <stdint.h>

typedef enum
{
    CONTROLLER_SM_IDLE = 0, /**< 空闲态（默认态）。 */
    CONTROLLER_SM_ACTIVE = 1, /**< 激活态（由按键触发切换）。 */
} controller_sm_state_t;

extern volatile uint32_t g_controller_pa0_irq_cnt; /**< PA0 EXTI0 中断累计次数（在中断回调中递增）。 */
extern volatile uint8_t g_controller_pa0_irq_pending; /**< PA0 中断挂起标志（在中断回调置 1，在任务中清 0）。 */

void Controller_StateMachine_Init(void);
void Controller_StateMachine_Step(void);
controller_sm_state_t Controller_StateMachine_GetState(void);

#endif
