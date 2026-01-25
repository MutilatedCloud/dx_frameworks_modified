#ifndef __CONTROLLER_UART_H__
#define __CONTROLLER_UART_H__

#include <stdint.h>

#include "arm_math_types.h"
#include "uart_api.h"

void UART_Message_Trans(float32_t float_array[], uint8_t out_buf[]);

uart_status_t UART_Message_Parse(const uint8_t in_buf[], float32_t out_array[]);

void Controller_Uart_tx_init(uart_msg_t *tx_msg, uint8_t *tx_buf);

#endif
