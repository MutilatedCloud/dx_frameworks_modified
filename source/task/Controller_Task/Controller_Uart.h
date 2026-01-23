#ifndef __CONTROLLER_UART_H__
#define __CONTROLLER_UART_H__

#include <stdint.h>

#include "arm_math_types.h"
#include "uart_api.h"

/**
 * @brief 将 6 个 float32 数据打包成 ASCII 4 位数字形式（每个数乘 1000）。
 * @param float_array 输入数组，长度为 6。
 * @param out_buf 输出 buffer，长度至少为 CONTROLLER_UART_DATA_LEN。
 */
void UART_Message_Trans(float32_t float_array[], uint8_t out_buf[]);

/**
 * @brief 将 ASCII 4 位数字形式的数据解析为 float32（每个数除 1000）。
 * @param in_buf 输入 buffer，长度至少为 CONTROLLER_UART_DATA_LEN。
 * @param out_array 输出数组，长度为 6。
 * @return UART_OK/UART_ERROR。
 */
uart_status_t UART_Message_Parse(const uint8_t in_buf[], float32_t out_array[]);

/**
 * @brief 初始化控制器 UART 发送结构体。
 * @param tx_msg UART 发送消息结构体。
 * @param tx_buf 发送 buffer。
 */
void Controller_Uart_tx_init(uart_msg_t *tx_msg, uint8_t *tx_buf);

#endif
