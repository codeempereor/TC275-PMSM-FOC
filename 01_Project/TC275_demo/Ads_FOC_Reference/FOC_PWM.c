#include "FOC_PWM.h"
#include "IfxCcu6.h"
#include "IfxCcu6_PwmHl.h"
#include "IfxCcu6_TimerWithTrigger.h"
#include "IfxCcu6_PinMap.h"
#include "IfxPort.h"

/* PWM parameters */
#define PWM_FREQ_HZ     20000.0f
#define PWM_DEADTIME_S  0.000005f   /* 5us dead time */

/* CCU6 objects */
static IfxCcu6_TimerWithTrigger g_timer;
static IfxCcu6_TimerWithTrigger_Config g_timerCfg;
static IfxCcu6_PwmHl g_pwmHl;
static IfxCcu6_PwmHl_Config g_pwmHlCfg;

void FOC_PWM_Init(void)
{
    /* === Step 1: Init TimerWithTrigger (T12) === */
    IfxCcu6_TimerWithTrigger_initConfig(&g_timerCfg, &MODULE_CCU60);
    g_timerCfg.base.frequency = PWM_FREQ_HZ;
    g_timerCfg.base.countDir = IfxStdIf_Timer_CountDir_upAndDown; /* center aligned */
    g_timerCfg.base.startOffset = 0;
    g_timerCfg.base.isrPriority = 0; /* no interrupt */
    g_timerCfg.base.isrProvider = IfxSrc_Tos_cpu0;
    g_timerCfg.triggerOut = NULL_PTR;

    IfxCcu6_TimerWithTrigger_init(&g_timer, &g_timerCfg);

    /* === Step 2: Init PwmHl (3-phase complementary) === */
    IfxCcu6_PwmHl_initConfig(&g_pwmHlCfg);
    g_pwmHlCfg.timer = &g_timer;

    /* Pins (hardware wiring confirmed):
     * A: CC60=P02.0 (high), COUT60=P02.1 (low)
     * B: CC61=P02.7 (high), COUT61=P02.3 (low)
     * C: CC62=P02.4 (high), COUT62=P02.5 (low)
     */
    g_pwmHlCfg.cc0 = (IfxCcu6_Cc60_Out *)&IfxCcu60_CC60_P02_0_OUT;
    g_pwmHlCfg.cout0 = (IfxCcu6_Cout60_Out *)&IfxCcu60_COUT60_P02_1_OUT;
    g_pwmHlCfg.cc1 = (IfxCcu6_Cc61_Out *)&IfxCcu60_CC61_P02_7_OUT;
    g_pwmHlCfg.cout1 = (IfxCcu6_Cout61_Out *)&IfxCcu60_COUT61_P02_3_OUT;
    g_pwmHlCfg.cc2 = (IfxCcu6_Cc62_Out *)&IfxCcu60_CC62_P02_4_OUT;
    g_pwmHlCfg.cout2 = (IfxCcu6_Cout62_Out *)&IfxCcu60_COUT62_P02_5_OUT;

    /* Base config */
    g_pwmHlCfg.base.deadtime = PWM_DEADTIME_S;
    g_pwmHlCfg.base.minPulse = 0.000001f; /* 1us min pulse */
    g_pwmHlCfg.base.channelCount = 3;
    g_pwmHlCfg.base.emergencyEnabled = FALSE;
    g_pwmHlCfg.base.outputMode = IfxPort_OutputMode_pushPull;
    g_pwmHlCfg.base.outputDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    g_pwmHlCfg.base.ccxActiveState = Ifx_ActiveState_high;   /* high side (INHx) active high */
    g_pwmHlCfg.base.coutxActiveState = Ifx_ActiveState_high;  /* low side (INLx) active high */

    /* Start timer FIRST, then init PwmHl (per iLLD note) */
    IfxCcu6_TimerWithTrigger_run(&g_timer);

    IfxCcu6_PwmHl_init(&g_pwmHl, &g_pwmHlCfg);

    /* Set PWM mode to center aligned */
    IfxCcu6_PwmHl_setMode(&g_pwmHl, Ifx_Pwm_Mode_centerAligned);

    /* Set initial duty: 0% all phases (output off) */
    Ifx_TimerValue tOn[3] = {0, 0, 0};
    IfxCcu6_PwmHl_setOnTime(&g_pwmHl, tOn);
    IfxCcu6_TimerWithTrigger_applyUpdate(&g_timer);
}

void FOC_PWM_SetDuty(uint16 dutyA, uint16 dutyB, uint16 dutyC)
{
    /* dutyA/B/C are in ticks (0 ~ period) */
    Ifx_TimerValue tOn[3];
    tOn[0] = (Ifx_TimerValue)dutyA;
    tOn[1] = (Ifx_TimerValue)dutyB;
    tOn[2] = (Ifx_TimerValue)dutyC;
    IfxCcu6_PwmHl_setOnTime(&g_pwmHl, tOn);
    IfxCcu6_TimerWithTrigger_applyUpdate(&g_timer);
}

void FOC_PWM_SetDutyPercent(float32 dutyA, float32 dutyB, float32 dutyC)
{
    Ifx_TimerValue period = IfxCcu6_TimerWithTrigger_getPeriod(&g_timer);
    Ifx_TimerValue tOn[3];
    tOn[0] = (Ifx_TimerValue)(dutyA * (float32)period);
    tOn[1] = (Ifx_TimerValue)(dutyB * (float32)period);
    tOn[2] = (Ifx_TimerValue)(dutyC * (float32)period);
    IfxCcu6_PwmHl_setOnTime(&g_pwmHl, tOn);
    IfxCcu6_TimerWithTrigger_applyUpdate(&g_timer);
}

void FOC_PWM_Enable(void)
{
    /* EN_GATE controlled by main (P02.6), keep here for API symmetry */
}

void FOC_PWM_Disable(void)
{
    /* EN_GATE tied to 3.3V hardware, always enabled */
}
