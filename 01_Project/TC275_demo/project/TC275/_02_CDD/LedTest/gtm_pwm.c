#include "gtm_pwm.h"
#include "IfxGtm_reg.h"
#include "IfxPort_reg.h"
#include "Mcal_WdgLib.h"
#include "uart_test.h"

#define GTM_PWM_PERIOD  10000U

void GtmPwm_Init(void)
{
    uint32 tout;
    UartTest_SendString("GtmPwm_Init start\r\n");

    Mcal_ResetENDINIT();
    MODULE_GTM.CLC.B.DISR = 0u;
    Mcal_SetENDINIT();

    tout = 1000000u;
    while((MODULE_GTM.CLC.B.DISS != 0u) && (tout > 0u)) { tout--; }

    GTM_CMU_CLK_EN.U = (2u << 22);

    GTM_TOM1_CH4_CTRL.U = 0x00000000u;

    GTM_TOM1_CH4_SR0.U = GTM_PWM_PERIOD;
    GTM_TOM1_CH4_SR1.U = GTM_PWM_PERIOD / 2;

    GTM_TOM1_TGC0_FUPD_CTRL.U = (2u << 8) | (2u << 24);
    GTM_TOM1_TGC0_GLB_CTRL.U = (1u << 0);

    GTM_TOM1_TGC0_ENDIS_CTRL.U = (2u << 8);
    GTM_TOM1_TGC0_OUTEN_CTRL.U = (2u << 8);
    GTM_TOM1_TGC0_GLB_CTRL.U = (1u << 0);

    GTM_TOUTSEL0.U = (GTM_TOUTSEL0.U & ~(0x3u << 28)) | (0x1u << 28);

    P00_IOCR4.U = (P00_IOCR4.U & ~(0xFu << 4)) | (0x9u << 4);

    UartTest_SendString("GtmPwm_Init done\r\n");
}

void GtmPwm_SetDuty(uint32 duty)
{
    if(duty > GTM_PWM_PERIOD) duty = GTM_PWM_PERIOD;
    GTM_TOM1_CH4_SR1.U = duty;
    GTM_TOM1_TGC0_FUPD_CTRL.U = (2u << 8) | (2u << 24);
    GTM_TOM1_TGC0_GLB_CTRL.U = (1u << 0);
}

void GtmPwm_SetFreq(uint32 freq)
{
    uint32 period = 100000000u / freq;
    GTM_TOM1_CH4_SR0.U = period;
    GTM_TOM1_TGC0_FUPD_CTRL.U = (2u << 8) | (2u << 24);
    GTM_TOM1_TGC0_GLB_CTRL.U = (1u << 0);
}
