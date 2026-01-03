#include "dsp/fast_math_functions.h"
#include "main.h"
#include "usart.h"
#include "uart_api.h"
#include "motor_DM.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>
#include "arm_math.h"
#define JOINT_NUM 6

// ========== 串口协议相关宏定义 ==========
#define CONTROLLER_UART_HEAD0       0xAA
#define CONTROLLER_UART_HEAD1       0x55
#define CONTROLLER_UART_PAYLOAD_LEN 24
#define CONTROLLER_UART_FRAME_LEN   (2 + CONTROLLER_UART_PAYLOAD_LEN + 1 + 2)  // 帧头2 + 数据24 + 校验1 + 结尾2
#define CONTROLLER_RX_BUF_SIZE      512

// 接收缓冲区
static uint8_t Controller_Rx_Buffer[CONTROLLER_RX_BUF_SIZE];
static uint16_t Controller_Rx_Len = 0;

DM_motor_t *joint_motor[JOINT_NUM];
uart_rx_t Angle_msg;
uart_msg_t Angle_rx_msg;
uart_msg_t Angle_tx_msg;
uint8_t Angle_rx_msg_Buffer[256];
float joint_radian[6] = {0};

/**
 * @brief 解析流式数据中的一帧
 * @param stream_buf 缓冲区指针
 * @param stream_len 缓冲区长度
 * @param out_array  输出的6个角度值
 * @param consumed   消耗的字节数
 * @return 0成功, -1失败
 */
static int UART_Message_Parse_Stream(const uint8_t *stream_buf, uint16_t stream_len, float *out_array, uint16_t *consumed)
{
    *consumed = 0;

    for (uint16_t offset = 0; offset + CONTROLLER_UART_FRAME_LEN <= stream_len; offset++)
    {
        if (stream_buf[offset] != CONTROLLER_UART_HEAD0) {
            continue;
        }
        if (stream_buf[offset + 1] != CONTROLLER_UART_HEAD1) {
            continue;
        }

        const uint8_t *frame = &stream_buf[offset];

        // 检查帧尾 \r\n
        if (frame[2 + CONTROLLER_UART_PAYLOAD_LEN + 1] != '\r' ||
            frame[2 + CONTROLLER_UART_PAYLOAD_LEN + 2] != '\n') {
            continue;
        }

        // 校验和
        uint8_t checksum = 0;
        for (int i = 0; i < CONTROLLER_UART_PAYLOAD_LEN; i++)
        {
            checksum = (uint8_t)(checksum + frame[2 + i]);
        }
        if (checksum != frame[2 + CONTROLLER_UART_PAYLOAD_LEN]) {
            continue;
        }

        // 解析6个角度值，每个4字符
        for (int i = 0; i < 6; i++)
        {
            int value = 0;
            for (int j = 0; j < 4; j++)
            {
                uint8_t c = frame[2 + i * 4 + j];
                if (c < '0' || c > '9') {
                    goto next_offset;
                }
                value = value * 10 + (int)(c - '0');
            }
            out_array[i] = ((float)value) / 1000.0f;
        }

        *consumed = offset + CONTROLLER_UART_FRAME_LEN;
        return 0;  // 成功

    next_offset:
        continue;
    }

    // 没找到完整帧，丢弃无法构成帧的前部数据
    if (stream_len >= CONTROLLER_UART_FRAME_LEN) {
        *consumed = stream_len - CONTROLLER_UART_FRAME_LEN + 1;
    } else {
        *consumed = 0;
    }
    return -1;  // 失败
}

// 标记是否有新数据需要处理
static volatile uint8_t Controller_Rx_Flag = 0;

void Angle_Receive_Callback(uint8_t *buf, uint32_t len)
{
    // 中断中只做数据拷贝，不做复杂解析
    if (len > CONTROLLER_RX_BUF_SIZE) {
        len = CONTROLLER_RX_BUF_SIZE;
    }
    if (Controller_Rx_Len + len > CONTROLLER_RX_BUF_SIZE) {
        Controller_Rx_Len = 0;
    }
    memcpy(&Controller_Rx_Buffer[Controller_Rx_Len], buf, (uint16_t)len);
    Controller_Rx_Len += (uint16_t)len;
    Controller_Rx_Flag = 1;
}

// 在任务中调用此函数处理接收到的数据
void Angle_Process_Data(void)
{
    if (!Controller_Rx_Flag) {
        return;
    }
    Controller_Rx_Flag = 0;

    // 进入临界区，防止中断修改缓冲区
    __disable_irq();
    uint16_t local_len = Controller_Rx_Len;
    uint8_t local_buf[CONTROLLER_RX_BUF_SIZE];
    memcpy(local_buf, Controller_Rx_Buffer, local_len);
    Controller_Rx_Len = 0;  // 清空原缓冲区
    __enable_irq();

    uint16_t offset = 0;
    while (local_len - offset >= CONTROLLER_UART_FRAME_LEN)
    {
        uint16_t consumed = 0;
        float angles[6];

        int result = UART_Message_Parse_Stream(
            &local_buf[offset],
            local_len - offset,
            angles,
            &consumed
        );

        if (result == 0)
        {
            for (int i = 0; i < 6; i++) {
                joint_radian[i] = angles[i] - PI;
            }
        }

        if (consumed > 0)
        {
            offset += consumed;
        }
        else
        {
            break;
        }
    }

    // 把未处理完的数据放回缓冲区
    if (offset < local_len)
    {
        __disable_irq();
        uint16_t remaining = local_len - offset;
        memmove(&Controller_Rx_Buffer[Controller_Rx_Len], &local_buf[offset], remaining);
        Controller_Rx_Len += remaining;
        __enable_irq();
    }
}
void angle_msg_rx_init(void)
{
    Angle_msg.rx_msg = &Angle_rx_msg;
    Angle_msg.rx_msg -> pBuffer = Angle_rx_msg_Buffer;
    Angle_msg.rx_msg -> huart = &huart7;
    Angle_msg.rx_msg -> Len = 256;
    uart7_rx_hook = Angle_Receive_Callback;
    uart_rx_init(&Angle_msg);
}
uart_msg_t testUart_tx_msg;
void testUart_tx_init(void){
    testUart_tx_msg.huart= &huart7;
    testUart_tx_msg.pBuffer = (uint8_t *)"hello\r\n";
    testUart_tx_msg.Len = strlen((char *) testUart_tx_msg.pBuffer);
}

void uart_Transmit_Angle(void *argment)
{
    UNUSED(argment);
    osDelay(10);
    angle_msg_rx_init();
    // testUart_tx_init();
    while (1) {
        // uart_tx_send_IT(&testUart_tx_msg);
        Angle_Process_Data();  // 在任务中处理接收数据
        osDelay(2);    
    }
}
void joint_motor_init(void)
{
    for (int joint_index = 0; joint_index < JOINT_NUM; joint_index++) {
        joint_motor[joint_index] = pvPortMalloc(sizeof(DM_motor_t));
        //配置 can1
        joint_motor[joint_index]->can_cfg.port = CAN1_PORT;
        joint_motor[joint_index]->tmp.PMAX = 12.5f;
        joint_motor[joint_index]->tmp.VMAX = 3.0f;
        joint_motor[joint_index]->tmp.TMAX = 1.0f;
        // 配置can id
        joint_motor[joint_index]->can_cfg.id = 0x01 + joint_index;
        joint_motor[joint_index]->motor_msg.can_msg.id = 0x11+ joint_index;
        Motor_DM_Init(joint_motor[joint_index]);
    }
}
float dm_angle_test = 0;
void jointFollowAngle(void *argument)
{
    UNUSED(argument);
    joint_motor_init();
    Motor_DM_Enable(joint_motor[0]);
    while (1) {
        dm_angle_test = joint_motor[0]->motor_msg.motor_angle;
        Motor_DM_Refresh(joint_motor[0]);
        // Motor_DM_Refresh(joint_motor[1]);
        // Motor_DM_Refresh(joint_motor[2]);
        // Motor_DM_Refresh(joint_motor[3]);
        // Motor_DM_Refresh(joint_motor[4]);
        // Motor_DM_Refresh(joint_motor[5]);

        MIT_CtrlMotorDM(joint_motor[0],joint_radian[4], 0, 1, 0, 0);
        // MIT_CtrlMotorDM(joint_motor[1],joint_radian[1], 0, 1, 0, 0);
        // MIT_CtrlMotorDM(joint_motor[2],joint_radian[2], 0, 1, 0, 0);
        // MIT_CtrlMotorDM(joint_motor[3],joint_radian[3], 0, 1, 0, 0);
        // MIT_CtrlMotorDM(joint_motor[4],joint_radian[4], 0, 1, 0, 0);
        // MIT_CtrlMotorDM(joint_motor[5],joint_radian[5], 0, 1, 0, 0);
        osDelay(2);
    }
}
