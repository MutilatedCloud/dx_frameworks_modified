#include "Controller_Task.h"

#include "Controller_Debugger.h"
#include "Controller_Motor.h"
#include "Controller_StateMachine.h"
#include "Controller_Uart.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"

/**
 * @brief 控制器发送给外部的关节角调试数据（单位：弧度）。
 */
float32_t test_angle[6];

/**
 * @brief 控制器 UART 发送缓冲区（长度为 CONTROLLER_UART_DATA_LEN）。
 * @note Referee_Task 会通过 extern 读取此 buffer。
 */
uint8_t Uart_Send_Buffer[CONTROLLER_UART_DATA_LEN] = {0};

typedef struct
{
    DJI_motor_t *motor_dji; /**< 3508/2006 电机组句柄（外部分配内存后交由本任务初始化）。 */
    DJI_motor_t *motor_6020; /**< 6020 电机组句柄（外部分配内存后交由本任务初始化）。 */
    uart_msg_t uart_tx_msg; /**< UART 发送消息结构体：仅维护指针与长度，实际发送目前由 Referee_Task 统一执行。 */
    float32_t joint_angle[6]; /**< 关节角缓存：
                                   - Controller_Angle_Refresh 输出为“度”
                                   - 经过零点补偿/映射后改写为“弧度”，并用于打包发送 */
    float32_t angle_zero_point[6]; /**< 零点角（度）：上电后等待 CAN 数据有效后采样一次作为零点。 */
} controller_task_ctx_t;

/**
 * @brief 控制器任务初始化。
 * @param ctx 控制器任务上下文（由调用者提供栈上对象）。
 * @note 本函数会：
 *       1) 分配电机对象内存；
 *       2) 初始化电机 CAN 配置；
 *       3) 等待 CAN 数据有效并记录零点角（度）；
 *       4) 初始化 UART 发送结构体；
 *       5) 初始化 PA0 状态机。
 * @note 副作用：会更新 g_controller_dbg_stat.stage 供调试定位。
 */
static void Controller_Task_Init(controller_task_ctx_t *ctx)
{
    g_controller_dbg_stat.stage = 2;
    ctx->motor_dji = pvPortMalloc(sizeof(DJI_motor_t));
    ctx->motor_6020 = pvPortMalloc(sizeof(DJI_motor_t));

    Controller_Motor_Init(&ctx->motor_dji, &ctx->motor_6020);
    g_controller_dbg_stat.stage = 3;
    Controller_Wait_And_Capture_Zero(ctx->motor_dji, ctx->motor_6020, ctx->joint_angle, ctx->angle_zero_point);
    g_controller_dbg_stat.stage = 4;
    Controller_Uart_tx_init(&ctx->uart_tx_msg, Uart_Send_Buffer);

    Controller_StateMachine_Init();

    Controller_Debugger_Notify_InitDone();
}

/**
 * @brief 控制器任务单次循环处理。
 * @param ctx 控制器任务上下文。
 * @note 本函数会：
 *       - 执行 PA0 状态机去抖与状态切换；
 *       - 刷新电机角度并做零点补偿；
 *       - 将角度映射到 [0, 2π]（弧度）范围；
 *       - 更新调试镜像与 UART 发送 buffer。
 * @note 输出：
 *       - test_angle[]：供外部调试观察的六关节角（弧度）；
 *       - Uart_Send_Buffer[]：供 Referee_Task 读取并发送的固定长度数据（前 24 字节为角度，后 2 字节为开关状态）。
 */
static void Controller_Task_Step(controller_task_ctx_t *ctx)
{
    g_controller_dbg_stat.stage = 5;

    Controller_StateMachine_Step();
    Controller_Angle_Refresh(ctx->motor_dji, ctx->motor_6020, ctx->joint_angle);

    for (int i = 0; i < 6; i++)
    {
        float32_t angle = ctx->joint_angle[i] - ctx->angle_zero_point[i];
        if (angle < -180)
        {
            angle += 360;
        }
        else if (angle > 180)
        {
            angle -= 360;
        }

        angle += 180;
        angle = angle * 3.1415926f / 180;
        ctx->joint_angle[i] = angle;
        test_angle[i] = angle;
    }

    Controller_Debugger_Update_JointAngle(ctx->joint_angle);

    UART_Message_Trans(ctx->joint_angle, Uart_Send_Buffer);

    Uart_Send_Buffer[24] = (uint8_t)('0');
    Uart_Send_Buffer[25] = (uint8_t)('0' + (g_controller_dbg_switch.sm_state ? 1 : 0));

    Controller_Debugger_Update_TxBuf(Uart_Send_Buffer, CONTROLLER_UART_DATA_LEN);
    ctx->uart_tx_msg.pBuffer = Uart_Send_Buffer;
    ctx->uart_tx_msg.Len = CONTROLLER_UART_DATA_LEN;
    // uart_tx_send_IT(&ctx->uart_tx_msg);

    Controller_Debugger_Notify_Step();
}

/**
 * @brief 控制器主任务：读取电机关节角、零点校准、打包并通过 UART7 发送。
 */
void Controller_Task(void *argument)
{
    /* USER CODE BEGIN Controller_Task */
    UNUSED(argument);
    g_controller_dbg_stat.stage = 1;
    controller_task_ctx_t ctx = {0};
    Controller_Task_Init(&ctx);

    for(;;)
    {
        Controller_Task_Step(&ctx);
        osDelay(4);
    }
}
