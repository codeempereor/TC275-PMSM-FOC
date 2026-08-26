#include "foc.h"

static FOC_Handle_t g_foc;

void FOC_Demo_Init(void)
{
    FOC_Init(&g_foc);
    FOC_SetSpeed(&g_foc, 800.0f);
    FOC_Start(&g_foc);
}

void FOC_Demo_PWM_ISR(void)
{
    FOC_CurrentLoop(&g_foc);
}

void FOC_Demo_Stop(void)
{
    FOC_Stop(&g_foc);
}

float FOC_Demo_GetSpeed(void)
{
    return FOC_GetSpeed(&g_foc);
}

FOC_State_t FOC_Demo_GetState(void)
{
    return FOC_GetState(&g_foc);
}
