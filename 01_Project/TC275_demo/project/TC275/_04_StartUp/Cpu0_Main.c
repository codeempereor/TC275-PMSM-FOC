#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "FOC_PWM.h"
#include "IfxPort.h"

IfxCpu_syncEvent g_cpuSyncEvent = 0;

#define PWM_PERIOD 5000U

int core0_main(void)
{
    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    FOC_PWM_Init();

    // 固定直流磁场：A相80%，B/C相50%
    FOC_PWM_SetDuty(4000, 2500, 2500);

    FOC_PWM_Enable();

    while (1)
    {
    }
    return (1);
}
