#include "Controller_Task.h"

#include "Controller_Debugger.h"
#include "Controller_Motor.h"
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
    DJI_motor_t *motor_dji;
    DJI_motor_t *motor_6020;
    uart_msg_t uart_tx_msg;
    float32_t joint_angle[6];
    float32_t angle_zero_point[6];
} controller_task_ctx_t;

/**
 * @brief 控制器任务初始化：分配电机对象、等待 CAN 数据有效并记录零点、初始化 UART 发送结构体。
 * @param ctx 控制器任务上下文。
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

    Controller_Debugger_Notify_InitDone();
}

/**
 * @brief 控制器任务单次循环：刷新角度、零点补偿、角度映射到弧度并打包发送。
 * @param ctx 控制器任务上下文。
 */
static void Controller_Task_Step(controller_task_ctx_t *ctx)
{
    g_controller_dbg_stat.stage = 5;
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
