#include "arm_math_types.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "Referee_Task.h"
#include <stdio.h>
#include <stdint.h>
#include "uart_api.h"
#include "usart.h"
#include "referee_api.h"
#include "referee_protocol.h"
#include "Controller_Task.h"

extern uint8_t Uart_Send_Buffer[CONTROLLER_UART_DATA_LEN];

void Referee_Task(void *argument)
{
    /* USER CODE BEGIN Referee_Task */
    UNUSED(argument);
    referee_init(&huart7);
    /* Delay enabling UART TX to avoid sending during early startup transients. */
    osDelay(2000);
    for(;;)
    {
        uint8_t Controller_data[30] = {0};
        for(int i = 0;i < CONTROLLER_UART_DATA_LEN;i++)
        {
            Controller_data[i] = Uart_Send_Buffer[i];
        }
        /* Extended byte 27: joint5 sign (0 = non-negative, 1 = negative). */
        Controller_data[CONTROLLER_UART_DATA_LEN] = (uint8_t)g_controller_joint5_sign;
        referee_data_pack_handle(0xA5, 0x0302 , Controller_data, sizeof(Controller_data));
        osDelay(50);
    }
    /* USER CODE END Referee_Task */
}
