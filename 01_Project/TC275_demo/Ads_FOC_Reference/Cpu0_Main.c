#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "FOC_DRV8305.h"
#include "FOC_PWM.h"
#include "IfxPort.h"

IfxCpu_syncEvent g_cpuSyncEvent = 0;

#define PIN_LED1 &MODULE_P00, 5
#define PIN_EN_GATE &MODULE_P02, 6
#define PIN_NFAULT &MODULE_P00, 3

static const float32 sin_lut[64] = {
     0.0000f, 0.0980f, 0.1951f, 0.2903f, 0.3827f, 0.4714f, 0.5556f, 0.6344f,
     0.7071f, 0.7730f, 0.8315f, 0.8819f, 0.9239f, 0.9569f, 0.9808f, 0.9952f,
     1.0000f, 0.9952f, 0.9808f, 0.9569f, 0.9239f, 0.8819f, 0.8315f, 0.7730f,
     0.7071f, 0.6344f, 0.5556f, 0.4714f, 0.3827f, 0.2903f, 0.1951f, 0.0980f,
     0.0000f,-0.0980f,-0.1951f,-0.2903f,-0.3827f,-0.4714f,-0.5556f,-0.6344f,
    -0.7071f,-0.7730f,-0.8315f,-0.8819f,-0.9239f,-0.9569f,-0.9808f,-0.9952f,
    -1.0000f,-0.9952f,-0.9808f,-0.9569f,-0.9239f,-0.8819f,-0.8315f,-0.7730f,
    -0.7071f,-0.6344f,-0.5556f,-0.4714f,-0.3827f,-0.2903f,-0.1951f,-0.0980f
};

#define DUTY_AMPLITUDE   0.20f
#define LOOP_DELAY       300000

int core0_main(void)
{
    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    IfxPort_setPinModeOutput(PIN_LED1, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinHigh(PIN_LED1);
    IfxPort_setPinModeInput(PIN_NFAULT, IfxPort_InputMode_pullUp);

    DRV8305_Init();
    DRV8305_WriteReg(0x07, 0x0FF);
    DRV8305_WriteReg(0x0C, 0x001F);

    IfxPort_setPinModeOutput(PIN_EN_GATE, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinHigh(PIN_EN_GATE);
    for (volatile int j = 0; j < 2000000; j++);

    FOC_PWM_Init();
    IfxPort_setPinModeOutput(&MODULE_P02, 4, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_alt7);
    FOC_PWM_SetDutyPercent(0.5f, 0.5f, 0.5f);
    for (volatile int j = 0; j < 5000000; j++);

    IfxPort_setPinLow(PIN_LED1);

    uint16 idx = 0;
    uint16 idxB = 21;
    uint16 idxC = 42;

    while (1)
    {
        if (IfxPort_getPinState(PIN_NFAULT) == 0) break;

        FOC_PWM_SetDutyPercent(
            0.5f + DUTY_AMPLITUDE * sin_lut[idx],
            0.5f + DUTY_AMPLITUDE * sin_lut[idxB],
            0.5f + DUTY_AMPLITUDE * sin_lut[idxC]
        );
        idx  = (idx  + 1) & 63;
        idxB = (idxB + 1) & 63;
        idxC = (idxC + 1) & 63;

        for (volatile int j = 0; j < LOOP_DELAY; j++);
    }

    /* fault: release motor (all phases 50% = zero line voltage), LED off */
    FOC_PWM_SetDutyPercent(0.5f, 0.5f, 0.5f);
    IfxPort_setPinHigh(PIN_LED1);
    while (1);
    return (1);
}
