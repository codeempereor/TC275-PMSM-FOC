#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "FOC_DRV8305.h"
#include "FOC_PWM.h"
#include "FOC_UART.h"
#include "FOC_ADC.h"
#include "FOC_Algorithm.h"
#include "FOC_SPI.h"
#include "IfxPort.h"
#include "Bsp.h"

IfxCpu_syncEvent g_cpuSyncEvent = 0;

#define PIN_LED1 &MODULE_P00, 5
/* EN_GATE is hard-wired to 3.3 V on the DRV8305 board; no GPIO needed. */
#define PIN_NFAULT &MODULE_P00, 3

/* ==============================================
 * 软件配置开关（直接改这两个值，不用动硬件）
 * ============================================== */
#define CURRENT_DIR_A   -1  /* A相电流极性：1=正，-1=反 */
#define CURRENT_DIR_B   -1  /* B相电流极性：1=正，-1=反 */
#define MOTOR_DIR       -1  /* 电机方向：1=正序，-1=反序 */

/* ==============================================
 * 快速多项式sin/cos（抄SguanFOC，比标准库快几十倍）
 * ============================================== */
#define VALUE_PI  3.14159265358979f

static inline float fast_sin(float x)
{
    int si = (int)(x * 0.31830988f);
    x = x - (float)si * VALUE_PI;
    if (si & 1) { x = x > 0.0f ? x - VALUE_PI : x + VALUE_PI; }
    float u = x * x;
    return x * (1.0f + u * (-0.16666666f + u * (0.0083333179f + u * (-0.00019840381f + u * (2.7532926e-06f + u * (-2.4703144e-08f + u * 1.3528548e-10f))))));
}
static inline float fast_cos(float x)
{
    int si = (int)(x * 0.31830988f);
    x = x - (float)si * VALUE_PI;
    if (si & 1) { x = x > 0.0f ? x - VALUE_PI : x + VALUE_PI; }
    float u = x * x;
    return 1.0f + u * (-0.49999991f + u * (0.041666519f + u * (-0.0013887906f + u * (2.4771643e-05f + u * (-2.7093486e-07f + u * 1.7290616e-09f)))));
}

#define DUTY_AMPLITUDE   0.15f
#define MIN_DIV  3000.0f
#define MAX_DIV  150.0f
#define RAMP_STEP 0.01f
#define SQRT3_2  0.8660254037844386f

/* Volatile state shared between main loop and ISR */
static volatile uint8   g_running = 0;
static volatile float32 g_phase = 0.0f;      /* electrical angle, 0..2π radians */
static volatile float32 g_phase_inc = 0.0f;

/* Zero current offset */
static uint16 g_offA = 2050;
static uint16 g_offB = 2044;
static uint16 g_offC = 2040;

/* Debug variables (ISR writes, main loop reads) */
static volatile float32 g_id = 0;
static volatile float32 g_iq = 0;
static volatile float32 g_ialpha = 0;
static volatile float32 g_ibeta = 0;

IFX_INTERRUPT(FOC_PWM_ISR, 0, 1);
void FOC_PWM_ISR(void)
{
    if (!FOC_PWM_AckIrq()) return;
    if (!g_running) return;

    /* 电角度累加 */
    g_phase += g_phase_inc;
    if (g_phase >= 2.0f * VALUE_PI) g_phase -= 2.0f * VALUE_PI;
    if (g_phase < 0.0f) g_phase += 2.0f * VALUE_PI;

    /* 同步采样电流 */
    uint16 rawA = FOC_ADC_ReadRaw(7);
    uint16 rawB = FOC_ADC_ReadRaw(6);
    uint16 rawC = FOC_ADC_ReadRaw(5);

    /* 减零偏，加极性配置 */
    float32 ia = (float32)(rawA - g_offA) * CURRENT_DIR_A;
    float32 ib = (float32)(rawB - g_offB) * CURRENT_DIR_B;
    float32 ic = -(ia + ib);  /* 第三相用基尔霍夫算，和SguanFOC一致 */

    /* 电机方向配置：反序则交换B/C */
    if (MOTOR_DIR == -1) {
        float32 tmp = ib;
        ib = ic;
        ic = tmp;
    }

    /* Clarke变换 */
    float32 ialpha = ia;
    float32 ibeta = (ia + 2.0f * ib) / 1.7320508075688772f;

    /* Park变换，用快速三角函数，角度反向试一下 */
    float32 sin_theta = fast_sin(-g_phase);
    float32 cos_theta = fast_cos(-g_phase);
    float32 id = ialpha * cos_theta + ibeta * sin_theta;
    float32 iq = ibeta * cos_theta - ialpha * sin_theta;

    /* 存调试变量 */
    g_ialpha = ialpha;
    g_ibeta = ibeta;
    g_id = id;
    g_iq = iq;

    /* 开环V/f输出，用快速三角函数 */
    float32 sine = fast_sin(g_phase);
    float32 cosine = fast_cos(g_phase);
    float32 dutyA = 0.5f + DUTY_AMPLITUDE * sine;
    float32 dutyB = 0.5f + DUTY_AMPLITUDE * (sine * (-0.5f) + cosine * SQRT3_2);
    float32 dutyC = 0.5f + DUTY_AMPLITUDE * (sine * (-0.5f) - cosine * SQRT3_2);

    /* 限幅 */
    if (dutyA > 0.8f) dutyA = 0.8f;
    if (dutyA < 0.2f) dutyA = 0.2f;
    if (dutyB > 0.8f) dutyB = 0.8f;
    if (dutyB < 0.2f) dutyB = 0.2f;
    if (dutyC > 0.8f) dutyC = 0.8f;
    if (dutyC < 0.2f) dutyC = 0.2f;

    FOC_PWM_SetDutyPercent(dutyA, dutyB, dutyC);
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

    FOC_UART_Init();
    FOC_UART_Print("\r\nFOC Motor Control Started\r\n");

    FOC_ADC_Init();
    FOC_UART_Print("ADC Ready\r\n");

    /* Zero-current offset calibration: motor is still (g_running=0, PWM=50% symmetric).
     * Read N samples of each phase, average them as the zero offset. */
    FOC_UART_Print("Calibrating zero current offset...\r\n");
    uint32 sumA = 0, sumB = 0, sumC = 0;
    #define CALIB_SAMPLES 1000
    for (uint32 i = 0; i < CALIB_SAMPLES; i++)
    {
        sumA += FOC_ADC_ReadRaw(7);
        sumB += FOC_ADC_ReadRaw(6);
        sumC += FOC_ADC_ReadRaw(5);
    }
    uint16 offA = sumA / CALIB_SAMPLES;
    uint16 offB = sumB / CALIB_SAMPLES;
    uint16 offC = sumC / CALIB_SAMPLES;
    g_offA = offA;
    g_offB = offB;
    g_offC = offC;
    FOC_UART_Print("Zero offset: A="); FOC_UART_PrintInt(offA);
    FOC_UART_Print(" B="); FOC_UART_PrintInt(offB);
    FOC_UART_Print(" C="); FOC_UART_PrintInt(offC);
    FOC_UART_Print("\r\n");

    /* 初始化AS5047P编码器，用现成的硬件QSPI1驱动 */
    FOC_SPI_Init();
    FOC_UART_Print("Encoder Ready\r\n");

    /* 先不启动电机，PWM保持50%，打印编码器读数验证 */
    g_running = 0;
    FOC_PWM_SetDutyPercent(0.5f, 0.5f, 0.5f);
    FOC_UART_Print("Turn motor by hand to verify encoder:\r\n");

    uint32 tick = 0;
    while (1)
    {
        /* 调试阶段：每 500ms 打一次 AS5047P 原始帧 + EF/PARD/角度。
         * 手转电机，angle 应在 0..16383 间单调变化；EF 必须恒为 0。 */
        tick++;
        if (tick >= 500000)
        {
            tick = 0;
            FOC_SPI_DumpDebug();
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

