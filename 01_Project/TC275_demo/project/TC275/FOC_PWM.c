#include "FOC_PWM.h"
#include "IfxGtm.h"
#include "IfxGtm_Cmu.h"
#include "IfxGtm_Atom_Pwm.h"
#include "IfxGtm_PinMap.h"
#include "IfxPort.h"

#define PWM_PERIOD 5000U
#define CLK_FREQ   1000000.0f
#define DEADTIME   50U

static IfxGtm_Atom_Pwm_Config s_cfgAH, s_cfgAL, s_cfgBH, s_cfgBL, s_cfgCH, s_cfgCL;
static IfxGtm_Atom_Pwm_Driver s_drvAH, s_drvAL, s_drvBH, s_drvBL, s_drvCH, s_drvCL;

void FOC_PWM_Init(void)
{
    IfxGtm_enable(&MODULE_GTM);
    IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, CLK_FREQ);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);

    IfxGtm_Atom_Pwm_initConfig(&s_cfgAH, &MODULE_GTM);
    s_cfgAH.atom = IfxGtm_Atom_0;
    s_cfgAH.atomChannel = IfxGtm_Atom_Ch_0;
    s_cfgAH.period = PWM_PERIOD;
    s_cfgAH.dutyCycle = 2500;
    s_cfgAH.pin.outputPin = (IfxGtm_Atom_ToutMap *)&IfxGtm_ATOM0_0_TOUT0_P02_0_OUT;
    s_cfgAH.synchronousUpdateEnabled = TRUE;
    IfxGtm_Atom_Pwm_init(&s_drvAH, &s_cfgAH);
    IfxGtm_Atom_Pwm_start(&s_drvAH, TRUE);

    IfxGtm_Atom_Pwm_initConfig(&s_cfgAL, &MODULE_GTM);
    s_cfgAL.atom = IfxGtm_Atom_0;
    s_cfgAL.atomChannel = IfxGtm_Atom_Ch_1;
    s_cfgAL.period = PWM_PERIOD;
    s_cfgAL.dutyCycle = 2500 + DEADTIME;
    s_cfgAL.pin.outputPin = (IfxGtm_Atom_ToutMap *)&IfxGtm_ATOM0_1_TOUT1_P02_1_OUT;
    s_cfgAL.synchronousUpdateEnabled = TRUE;
    s_cfgAL.signalLevel = Ifx_ActiveState_low;
    IfxGtm_Atom_Pwm_init(&s_drvAL, &s_cfgAL);
    IfxGtm_Atom_Pwm_start(&s_drvAL, TRUE);

    IfxGtm_Atom_Pwm_initConfig(&s_cfgBH, &MODULE_GTM);
    s_cfgBH.atom = IfxGtm_Atom_0;
    s_cfgBH.atomChannel = IfxGtm_Atom_Ch_3;
    s_cfgBH.period = PWM_PERIOD;
    s_cfgBH.dutyCycle = 2500;
    s_cfgBH.pin.outputPin = (IfxGtm_Atom_ToutMap *)&IfxGtm_ATOM0_3_TOUT3_P02_3_OUT;
    s_cfgBH.synchronousUpdateEnabled = TRUE;
    IfxGtm_Atom_Pwm_init(&s_drvBH, &s_cfgBH);
    IfxGtm_Atom_Pwm_start(&s_drvBH, TRUE);

    IfxGtm_Atom_Pwm_initConfig(&s_cfgBL, &MODULE_GTM);
    s_cfgBL.atom = IfxGtm_Atom_0;
    s_cfgBL.atomChannel = IfxGtm_Atom_Ch_4;
    s_cfgBL.period = PWM_PERIOD;
    s_cfgBL.dutyCycle = 2500 + DEADTIME;
    s_cfgBL.pin.outputPin = (IfxGtm_Atom_ToutMap *)&IfxGtm_ATOM0_4_TOUT4_P02_4_OUT;
    s_cfgBL.synchronousUpdateEnabled = TRUE;
    s_cfgBL.signalLevel = Ifx_ActiveState_low;
    IfxGtm_Atom_Pwm_init(&s_drvBL, &s_cfgBL);
    IfxGtm_Atom_Pwm_start(&s_drvBL, TRUE);

    IfxGtm_Atom_Pwm_initConfig(&s_cfgCH, &MODULE_GTM);
    s_cfgCH.atom = IfxGtm_Atom_0;
    s_cfgCH.atomChannel = IfxGtm_Atom_Ch_5;
    s_cfgCH.period = PWM_PERIOD;
    s_cfgCH.dutyCycle = 2500;
    s_cfgCH.pin.outputPin = (IfxGtm_Atom_ToutMap *)&IfxGtm_ATOM0_5_TOUT5_P02_5_OUT;
    s_cfgCH.synchronousUpdateEnabled = TRUE;
    IfxGtm_Atom_Pwm_init(&s_drvCH, &s_cfgCH);
    IfxGtm_Atom_Pwm_start(&s_drvCH, TRUE);

    IfxGtm_Atom_Pwm_initConfig(&s_cfgCL, &MODULE_GTM);
    s_cfgCL.atom = IfxGtm_Atom_0;
    s_cfgCL.atomChannel = IfxGtm_Atom_Ch_6;
    s_cfgCL.period = PWM_PERIOD;
    s_cfgCL.dutyCycle = 2500 + DEADTIME;
    s_cfgCL.pin.outputPin = (IfxGtm_Atom_ToutMap *)&IfxGtm_ATOM0_6_TOUT6_P02_6_OUT;
    s_cfgCL.synchronousUpdateEnabled = TRUE;
    s_cfgCL.signalLevel = Ifx_ActiveState_low;
    IfxGtm_Atom_Pwm_init(&s_drvCL, &s_cfgCL);
    IfxGtm_Atom_Pwm_start(&s_drvCL, TRUE);

    IfxPort_setPinModeOutput(PIN_EN_GATE, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinLow(PIN_EN_GATE);
}

void FOC_PWM_SetDuty(float32 dutyA, float32 dutyB, float32 dutyC)
{
    s_cfgAH.dutyCycle = (uint32)dutyA;
    s_cfgAL.dutyCycle = (uint32)(dutyA + DEADTIME);
    s_cfgBH.dutyCycle = (uint32)dutyB;
    s_cfgBL.dutyCycle = (uint32)(dutyB + DEADTIME);
    s_cfgCH.dutyCycle = (uint32)dutyC;
    s_cfgCL.dutyCycle = (uint32)(dutyC + DEADTIME);

    IfxGtm_Atom_Pwm_init(&s_drvAH, &s_cfgAH);
    IfxGtm_Atom_Pwm_init(&s_drvAL, &s_cfgAL);
    IfxGtm_Atom_Pwm_init(&s_drvBH, &s_cfgBH);
    IfxGtm_Atom_Pwm_init(&s_drvBL, &s_cfgBL);
    IfxGtm_Atom_Pwm_init(&s_drvCH, &s_cfgCH);
    IfxGtm_Atom_Pwm_init(&s_drvCL, &s_cfgCL);
}

void FOC_PWM_Enable(void)
{
    IfxPort_setPinHigh(PIN_EN_GATE);
}

void FOC_PWM_Disable(void)
{
    IfxPort_setPinLow(PIN_EN_GATE);
}
