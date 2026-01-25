#include "GpioExtiAll.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "Controller_StateMachine.h"

extern osSemaphoreId imuBinarySem01Handle;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_0)
    {
        g_controller_pa0_irq_cnt++;
        g_controller_pa0_irq_pending = 1;
    }
    else if(GPIO_Pin == ACC_INT_Pin)
    {
        osSemaphoreRelease(imuBinarySem01Handle);
    }
    else if(GPIO_Pin == GYRO_INT_Pin)
    {

    }
}