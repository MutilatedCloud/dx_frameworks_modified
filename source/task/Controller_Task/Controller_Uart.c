#include "Controller_Uart.h"

#include <stdio.h>
#include <string.h>

#include "Controller_Task.h"
#include "usart.h"

/**
 * @brief 将 6 个 float32 数据打包成固定 24 字节的 ASCII 数字数据。
 * @param float_array 输入数组，长度为 6。
 * @param out_buf 输出 buffer，长度至少为 CONTROLLER_UART_DATA_LEN。
 * @note 编码规则：每个通道占 4 字节 ASCII '0'~'9'，等价于把 value*1000 取整后格式化为 %04d。
 * @note 边界处理：为保证协议长度恒定，数值会被钳位到 [0, 9999]。
 * @note 本函数只写入前 24 字节角度数据；若协议在末尾追加扩展字段，需要由调用者在本函数后补写。
 */
void UART_Message_Trans(float32_t float_array[], uint8_t out_buf[])
{
    for (int i = 0; i < 6; i++)
    {
        int scaled = (int)(float_array[i] * 1000.0f);
        if (scaled < 0)
        {
            scaled = 0;
        }
        else if (scaled > 9999)
        {
            scaled = 9999;
        }

        char four_digit[5];
        (void)snprintf(four_digit, sizeof(four_digit), "%04d", scaled);
        memcpy(&out_buf[i * 4], four_digit, 4);
    }
}

/**
 * @brief 将固定 24 字节 ASCII 数字数据解析为 6 个 float32。
 * @param in_buf 输入 buffer，长度至少为 CONTROLLER_UART_DATA_LEN。
 * @param out_array 输出数组，长度为 6。
 * @return UART_OK: 成功；UART_ERROR: 输入包含非数字字符。
 * @note 解码规则：每 4 字节解析为 0~9999 的整数，再除以 1000.0f。
 * @note 本函数只读取前 24 字节角度数据；若协议在末尾追加扩展字段，需要由调用者自行解析。
 */
uart_status_t UART_Message_Parse(const uint8_t in_buf[], float32_t out_array[])
{
    for (int i = 0; i < 6; i++)
    {
        int value = 0;
        for (int j = 0; j < 4; j++)
        {
            uint8_t c = in_buf[i * 4 + j];
            if (c < '0' || c > '9')
            {
                return UART_ERROR;
            }
            value = value * 10 + (int)(c - '0');
        }
        out_array[i] = ((float32_t)value) / 1000.0f;
    }

    return UART_OK;
}

/**
 * @brief 初始化控制器 UART 发送结构体。
 * @param tx_msg UART 发送消息结构体。
 * @param tx_buf 发送 buffer（通常为 Uart_Send_Buffer）。
 * @note 当前默认使用 UART7。
 * @note 本函数只做指针/长度赋值，不会触发发送。
 */
void Controller_Uart_tx_init(uart_msg_t *tx_msg, uint8_t *tx_buf)
{
    tx_msg->huart = &huart7;
    tx_msg->pBuffer = tx_buf;
    tx_msg->Len = CONTROLLER_UART_DATA_LEN;
}
