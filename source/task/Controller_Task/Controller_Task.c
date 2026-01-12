#include "Controller_Task.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "string.h"
#include "arm_math_types.h"
#include "motor_DJI.h"
#include "uart_api.h"
#include "usart.h"
#include <stdio.h>
 #include <stdint.h>
#include "crc8_crc16.h"

float32_t test_angle[6];
uint8_t testUart_Send_Buffer[DATA_FRAME_LENGTH] = {0};
Controller_t Transmit_Frame_Data = {0};
uart_msg_t crc_send_msg;

void Controller_Task(void *argument)
{
    /* USER CODE BEGIN Controller_Task */
    UNUSED(argument);
    float32_t Joint_Angle[6];
    float32_t Angle_Zero_Point[6] = {0};
    DJI_motor_t *Controller_Motor_DJI;
    DJI_motor_t *Controller_Motor_6020;
    uart_msg_t Controller_Uart_tx_msg;
    uint8_t Uart_Send_Buffer[CONTROLLER_UART_DATA_LEN] = {0};

    Controller_Motor_DJI = pvPortMalloc(sizeof(DJI_motor_t));
    Controller_Motor_6020 = pvPortMalloc(sizeof(DJI_motor_t));
    Motor_Init_DJI(&Controller_Motor_DJI, &Controller_Motor_6020); 
    Controller_Wait_And_Capture_Zero(Controller_Motor_DJI,Controller_Motor_6020,Joint_Angle,Angle_Zero_Point);
    Controller_Uart_tx_init(&Controller_Uart_tx_msg, Uart_Send_Buffer);
    crc_send_msg_init();
    for(;;)
    {
        Controller_Angle_Refresh(Controller_Motor_DJI,  Controller_Motor_6020, Joint_Angle);
        for(int i = 0;i < 6;i++)
        {
            Joint_Angle[i] = Joint_Angle[i] - Angle_Zero_Point[i];
            if(Joint_Angle[i] < -180)
            {
                Joint_Angle[i] += 360;
            }
            else if(Joint_Angle[i] > 180)
            {
                Joint_Angle[i] -= 360;
            }   
            Joint_Angle[i] += 180;
            Joint_Angle[i] = Joint_Angle[i] * 3.1415926 / 180;
            test_angle[i] = Joint_Angle[i];
        }

        UART_Message_Trans(Joint_Angle, Uart_Send_Buffer);

        Data_Concatenation(Uart_Send_Buffer);

        // Controller_Uart_tx_msg.pBuffer = Uart_Send_Buffer;
        // Controller_Uart_tx_msg.Len = CONTROLLER_UART_DATA_LEN;
        // memcpy(testUart_Send_Buffer, Uart_Send_Buffer,CONTROLLER_UART_DATA_LEN);

        // uart_tx_send_IT(&Controller_Uart_tx_msg);
        crc_send_msg.Len = DATA_FRAME_LENGTH;
        crc_send_msg.pBuffer = (uint8_t *)(&Transmit_Frame_Data);
        memcpy(testUart_Send_Buffer, (uint8_t *)(&Transmit_Frame_Data), DATA_FRAME_LENGTH);
        uart_tx_send_IT(&crc_send_msg);
        osDelay(4);
    }
}

uint8_t Controller_Motor_Data_Ready(DJI_motor_t *Controller_Motor_DJI, DJI_motor_t *Controller_Motor_6020)
{
    if (Controller_Motor_DJI == NULL || Controller_Motor_6020 == NULL)
    {
        return 0;
    }

    for (int i = 0; i < 4; i++)
    {
        if (Controller_Motor_DJI->motor_msg[i].can_msg.cnt == 0)
        {
            return 0;
        }
    }
    for (int i = 0; i < 2; i++)
    {
        if (Controller_Motor_6020->motor_msg[i].can_msg.cnt == 0)
        {
            return 0;
        }
    }
    return 1;
}

void Controller_Wait_And_Capture_Zero(DJI_motor_t *Controller_Motor_DJI,DJI_motor_t *Controller_Motor_6020,float32_t *Joint_Angle,float32_t *Angle_Zero_Point)
{
    for (int t = 0; t < 300; t++)
    {
        Controller_Angle_Refresh(Controller_Motor_DJI, Controller_Motor_6020, Joint_Angle);
        if (Controller_Motor_Data_Ready(Controller_Motor_DJI, Controller_Motor_6020))
        {
            break;
        }
        osDelay(10);
    }

    Controller_Angle_Refresh(Controller_Motor_DJI, Controller_Motor_6020, Joint_Angle);
    for (int i = 0; i < 6; i++)
    {
        Angle_Zero_Point[i] = Joint_Angle[i];
    }
}

void Motor_Init_DJI(DJI_motor_t **Controller_Motor_DJI, DJI_motor_t **Controller_Motor_6020) 
{

  if (*Controller_Motor_DJI == NULL) 
  {
    return;
  }
  memset(*Controller_Motor_DJI, 0, sizeof(DJI_motor_t)); // 清零内存
  (*Controller_Motor_DJI)->can_cfg.port = CAN1_PORT;
  (*Controller_Motor_DJI)->can_cfg.id = Controller_Motor_DJI_All_ID;
  (*Controller_Motor_DJI)->motor_msg[Controller_Motor_3508_Joint_2].can_msg.id =Controller_Motor_3508_Joint_2_ID;
  (*Controller_Motor_DJI)->motor_msg[Controller_Motor_2006_Joint_3].can_msg.id =Controller_Motor_2006_Joint_3_ID;
  (*Controller_Motor_DJI)->motor_msg[Controller_Motor_2006_Joint_4].can_msg.id =Controller_Motor_2006_Joint_4_ID;
  (*Controller_Motor_DJI)->motor_msg[Controller_Motor_2006_Joint_5].can_msg.id =Controller_Motor_2006_Joint_5_ID;
  Motor_DJI_Init(*Controller_Motor_DJI);

  if (*Controller_Motor_6020 == NULL) 
  {
    return;
  }
  memset(*Controller_Motor_6020, 0, sizeof(DJI_motor_t)); 
  (*Controller_Motor_6020)->can_cfg.port = CAN1_PORT;
  (*Controller_Motor_6020)->can_cfg.id = Controller_Motor_6020_All_ID;
  (*Controller_Motor_6020)->motor_msg[Controller_Motor_6020_Joint_0].can_msg.id =Controller_Motor_6020_Joint_0_ID;
  (*Controller_Motor_6020)->motor_msg[Controller_Motor_6020_Joint_1].can_msg.id =Controller_Motor_6020_Joint_1_ID;
  Motor_DJI_Init(*Controller_Motor_6020);
}

void Controller_Angle_Refresh(DJI_motor_t *Controller_Motor_DJI, DJI_motor_t *Controller_Motor_6020, float32_t *Joint_Angle)
{
    Motor_DJI_Refresh(Controller_Motor_DJI);
    Motor_DJI_Refresh(Controller_Motor_6020);
    for(int i = 0;i < 4;i++)
    {
        Joint_Angle[i + 2] = Controller_Motor_DJI->motor_msg[i].motor_angle * Trans_Angle + 180;
    }
    for(int i = 0;i < 2;i++)
    {
        Joint_Angle[i] = Controller_Motor_6020->motor_msg[i].motor_angle * Trans_Angle + 180;
    }
}

void UART_Message_Trans(float32_t float_array[], uint8_t out_buf[]) 
{
    for (int i = 0; i < 6; i++)
    {
        int scaled = (int)(float_array[i] * 1000.0f);
        if (scaled < 0) {
            scaled = 0;
        } else if (scaled > 9999) {
            scaled = 9999;
        }

        char four_digit[5];
        (void)snprintf(four_digit, sizeof(four_digit), "%04d", scaled);
        memcpy(&out_buf[i * 4], four_digit, 4);
    }
}

uart_status_t UART_Message_Parse(const uint8_t in_buf[], float32_t out_array[])
{
    for (int i = 0; i < 6; i++)
    {
        int value = 0;
        for (int j = 0; j < 4; j++)
        {
            uint8_t c = in_buf[i * 4 + j];
            if (c < '0' || c > '9') {
                return UART_ERROR;
            }
            value = value * 10 + (int)(c - '0');
        }
        out_array[i] = ((float32_t)value) / 1000.0f;
    }
    return UART_OK;
}


void Controller_Uart_tx_init(uart_msg_t *tx_msg, uint8_t *tx_buf)
{
    tx_msg->huart = &huart7;
    tx_msg->pBuffer = tx_buf;
    tx_msg->Len = CONTROLLER_UART_DATA_LEN;
}

void Data_Concatenation(const uint8_t *pData)
{
    static uint8_t seq = 0;
    uint8_t zero[2]={0,0};
    // 帧头数据
    Transmit_Frame_Data.frame_header.sof = 0xA5;
    Transmit_Frame_Data.frame_header.data_length = DATA_LENGTH;
    Transmit_Frame_Data.frame_header.seq = seq;
    append_CRC8_check_sum((uint8_t *)(&Transmit_Frame_Data.frame_header), 5);
    
    // 命令码ID
    Transmit_Frame_Data.cmd_id = CONTROLLER_CMD_ID;
    // 数据段
    memcpy(Transmit_Frame_Data.data, pData, DATA_LENGTH);
    memcpy(Transmit_Frame_Data.data+DATA_LENGTH-2, zero, 2);
    
    // 帧尾CRC16，整包校验
    append_CRC16_check_sum((uint8_t *)(&Transmit_Frame_Data), DATA_FRAME_LENGTH);

    if (seq == 0xff) 
    {
        seq = 0;
    }
    else 
    {
        seq++;
    }
}

void crc_send_msg_init(void)
{
    crc_send_msg.huart = &huart7;
    crc_send_msg.Len = DATA_FRAME_LENGTH;
}
