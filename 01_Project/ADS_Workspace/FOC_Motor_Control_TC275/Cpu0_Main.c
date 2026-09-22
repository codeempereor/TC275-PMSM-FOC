#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "FOC_DRV8305.h"
#include "FOC_PWM.h"
#include "IfxPort.h"

IfxCpu_syncEvent g_cpuSyncEvent = 0;

#define PIN_LED1 &MODULE_P00, 5
/* EN_GATE is hard-wired to 3.3 V on the DRV8305 board; no GPIO needed. */
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

/*
 * PWM period ISR runs at 20kHz (same as PWM).
 * Phase is a float in units of sin LUT index (0..64). Each ISR we add
 * PHASE_INC, so the electrical angle rotates continuously (no steps).
 *   PHASE_INC = 1.0 / PWM_ISR_DIV
 *   elec freq = 20000 * PHASE_INC / 64
 * DIV=150 -> 133 LUT-updates/sec -> elec 2.08Hz -> ~18rpm
 */
#define PWM_ISR_DIV      150.0f
#define PHASE_INC        (1.0f / PWM_ISR_DIV)

/* 120 deg offset in LUT units (64/3 = 21.333) */
#define OFF_B            21.3333f
#define OFF_C            42.6667f

static inline float32 sin_lut_interp(float32 phase)
{
    if (phase >= 64.0f) phase -= 64.0f;
    uint8 idx = (uint8)phase;
    uint8 idx1 = (idx + 1) & 63;
    float32 frac = phase - (float32)idx;
    return sin_lut[idx] + frac * (sin_lut[idx1] - sin_lut[idx]);
}

/* Volatile state shared between main loop and ISR */
static volatile uint8   g_running = 0;       /* set 1 by main to start spin, 0 on fault */
static volatile float32 g_phase = 0.0f;      /* electrical angle, 0..64 LUT units */

IFX_INTERRUPT(FOC_PWM_ISR, 0, 1);
void FOC_PWM_ISR(void)
{
    if (!FOC_PWM_AckIrq()) return;
    if (!g_running) return;

    g_phase += PHASE_INC;
    if (g_phase >= 64.0f) g_phase -= 64.0f;

    FOC_PWM_SetDutyPercent(
        0.5f + DUTY_AMPLITUDE * sin_lut_interp(g_phase),
        0.5f + DUTY_AMPLITUDE * sin_lut_interp(g_phase + OFF_B),
        0.5f + DUTY_AMPLITUDE * sin_lut_interp(g_phase + OFF_C)
    );
}

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

    /*
     * SPI self-test: writes reg 0x07 = 0x090 then reads it back.
     *   LED solid ON  -> SPI link OK (registers actually reach DRV8305)
     *   LED blinks 5x -> SPI link FAIL (check SCLK/SDI/SCS/SDO wiring)
     * After this, we reprogram 0x07 to the real 3xPWM value below.
     */
    DRV8305_TestSPI();

    /*
     * Gate Drive Control (reg 0x07):
     *   bit 9   = 1   COMM_OPTION (default, active freewheel)
     *   bit 8:7 = 01  PWM_MODE = 3 independent inputs
     *                (INLx ignored; DRV8305 generates complementary low-side)
     *   bit 6:4 = 110 DEAD_TIME = 3520 ns (was 100=880ns, too small -> VDS shoot-through)
     *   bit 3:2 = 01  TBLANK (default)
     *   bit 1:0 = 10  TVDS (default)
     *   => 0x2E6
     */
    DRV8305_WriteReg(0x07, 0x2E6);
    DRV8305_WriteReg(0x0C, 0x001F);

    /* Clear latched fault bits from power-up sequencing (CLR_FLTS=1, self-clearing).
     * 0x09 default = 0x020 (WD_DLY=01); set bit1 to clear faults. */
    DRV8305_WriteReg(0x09, 0x022);
    for (volatile int j = 0; j < 100000; j++);

    /* EN_GATE is hard-wired to 3.3 V on the DRV8305 board; no GPIO needed. */
    for (volatile int j = 0; j < 2000000; j++);

    FOC_PWM_Init();
    IfxPort_setPinModeOutput(&MODULE_P02, 4, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_alt7);
    FOC_PWM_SetDutyPercent(0.5f, 0.5f, 0.5f);
    for (volatile int j = 0; j < 5000000; j++);

    IfxPort_setPinLow(PIN_LED1);

    /* Start the open-loop spin from the ISR. Before this, g_running=0 so the
     * ISR just acks and returns, PWM stays at 50% (symmetric, motor still). */
    g_running = 1;

    while (1)
    {
        if (IfxPort_getPinState(PIN_NFAULT) == 0)
        {
            g_running = 0;
            break;
        }
    }

    /* fault: release motor, then blink LED to report fault source */
    FOC_PWM_SetDutyPercent(0.5f, 0.5f, 0.5f);

    uint16 vds_fault = DRV8305_ReadReg(0x02);   /* VDS overcurrent faults */
    uint16 ic_fault  = DRV8305_ReadReg(0x03);   /* IC faults (UVLO/OTSD/etc) */
    uint16 vgs_fault = DRV8305_ReadReg(0x04);   /* VGS gate drive faults */

    int blink_count = 4;                          /* default: unknown fault */

    if (vds_fault != 0)
        blink_count = 2;                          /* VDS overcurrent (shoot-through) */
    else if (ic_fault != 0)
    {
        /* decode which IC fault bit is set */
        if      (ic_fault & (1 << 10)) blink_count = 1;   /* PVDD_UVLO2 */
        else if (ic_fault & (1 << 9))  blink_count = 2;   /* WD_FAULT */
        else if (ic_fault & (1 << 8))  blink_count = 3;   /* OTSD */
        else if (ic_fault & (1 << 6))  blink_count = 4;   /* VREG_UV */
        else if (ic_fault & (1 << 5))  blink_count = 5;   /* AVDD_UVLO */
        else if (ic_fault & (1 << 4))  blink_count = 6;   /* VCP_LSD_UVLO2 */
        else if (ic_fault & (1 << 2))  blink_count = 7;   /* VCPH_UVLO2 */
        else                           blink_count = 8;   /* other IC fault */
    }
    else if (vgs_fault != 0)
        blink_count = 9;                          /* VGS gate drive fault */

    for (int i = 0; i < blink_count; i++)
    {
        IfxPort_setPinLow(PIN_LED1);
        for (volatile int j = 0; j < 5000000; j++);
        IfxPort_setPinHigh(PIN_LED1);
        for (volatile int j = 0; j < 5000000; j++);
    }
    while (1);
    return (1);
}
