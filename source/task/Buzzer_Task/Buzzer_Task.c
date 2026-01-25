#include "cmsis_os2.h"
#include "referee_protocol.h"
#include "arm_math_types.h"
#include "buzzer_bsp.h"

#define pause1 620

void Buzzer_Task(void *argument)
{
    UNUSED(argument);
    //buzzer_init();
    int16_t tone[]={H3,H2,H1,H2,H3,H4,H3,H2,
        H3,H2,H1,H2,H3,H4,H3,H2,H3,
        H2,H1,H2,H3,H4,H3,H2,H3,
        H2,H1,H2,H3,H4,H3,H2,
        M1,M2,M3,M3,M2,M4,M3,M2,
        M2,M2,M1,M1,M4,M3,M2};
    float32_t pause[]={1,0.5,1,0.5,0.75,0.25,0.5,1.5,1,
        0.5,1,0.5,0.75,0.25,0.5,1.5,1,
        0.5,1,0.5,0.75,0.25,0.5,1.5,
        1,0.5,1,0.5,0.75,0.25,0.5,1,
        0.25,0.25,0.5,0.5,0.5,0.5,0.5,0.5,
        0.5,0.5,0.25,0.25,0.5,0.5,0.5};
    float32_t strenth[]={
        0.6,0.2,0.4,0.2,0.6,0.2,0.4,0.2,
        0.6,0.2,0.4,0.2,0.6,0.2,0.4,0.2,
        0.6,0.2,0.4,0.2,0.6,0.2,0.4,0.2,
        0.6,0.2,0.4,0.2,0.6,0.2,0.4,0.2,
        0.2,0.2,0.6,0.2,0.2,0.4,0.2,0.2,
        0.6,0.2,0.2,0.2,0.4,0.2,0.2};
    for(;;)
    {
        for(int i=16;i<47;i++)
        {
            buzzer_on(tone[i],strenth[i]);
            osDelay(pause[i]*pause1);
        }
        osDelay(1);
    }
}
