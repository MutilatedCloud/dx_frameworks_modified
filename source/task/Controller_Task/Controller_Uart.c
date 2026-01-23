#include "Controller_Uart.h"

#include <stdio.h>
#include <string.h>

#include "Controller_Task.h"
#include "usart.h"

/**
 * @brief 将 6 个 float32 数据打包成 ASCII 4 位数字形式（每个数乘 1000）。
 * @note 输出格式：每个通道 4 字节 ASCII '0'~'9'，总长 24 字节。
 * @note 为保证协议长度恒定，数值会被限制在 [0, 9999]。
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
 * @brief 将 ASCII 4 位数字形式的数据解析为 float32（每个数除 1000）。
 * @note 输入必须为 '0'~'9'，否则返回 UART_ERROR。
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
 * @note 当前默认使用 UART7。
 */
void Controller_Uart_tx_init(uart_msg_t *tx_msg, uint8_t *tx_buf)
{
    tx_msg->huart = &huart7;
    tx_msg->pBuffer = tx_buf;
    tx_msg->Len = CONTROLLER_UART_DATA_LEN;
}
