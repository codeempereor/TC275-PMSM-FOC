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
#define PIN_NFAULT &MODULE_P00, 3

#define CURRENT_DIR_A   -1
#define CURRENT_DIR_B   -1
#define MOTOR_DIR       -1

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

static volatile uint8   g_running = 0;
static volatile float32 g_elec_angle = 0.0f;

static PID_t g_pid_d;
static PID_t g_pid_q;
static volatile float32 g_id_ref = 0.0f;
static volatile float32 g_iq_ref = 0.0f;

static uint16 g_offA = 2050;
static uint16 g_offB = 2044;
static uint16 g_offC = 2040;

static volatile float32 g_id = 0, g_iq = 0, g_vd = 0, g_vq = 0;

IFX_INTERRUPT(FOC_PWM_ISR, 0, 1);
void FOC_PWM_ISR(void)
{
    if (!FOC_PWM_AckIrq()) return;
    if (!g_running) return;

    uint16 rawA = FOC_ADC_ReadRaw(7);
    uint16 rawB = FOC_ADC_ReadRaw(6);
    uint16 rawC = FOC_ADC_ReadRaw(5);

    float32 ia = (float32)(rawA - g_offA) * CURRENT_DIR_A;
    float32 ib = (float32)(rawB - g_offB) * CURRENT_DIR_B;
    float32 ic = -(ia + ib);

    if (MOTOR_DIR == -1) { float32 t = ib; ib = ic; ic = t; }

    float32 ialpha = ia;
    float32 ibeta = (ia + 2.0f * ib) / 1.7320508075688772f;

    float32 sin_e = fast_sin(g_elec_angle);
    float32 cos_e = fast_cos(g_elec_angle);
    float32 id  =  ialpha * cos_e + ibeta * sin_e;
    float32 iq  = -ialpha * sin_e + ibeta * cos_e;

    g_id = id;
    g_iq = iq;

    float32 vd = PID_Calc(&g_pid_d, g_id_ref, id);
    float32 vq = PID_Calc(&g_pid_q, g_iq_ref, iq);
    g_vd = vd;
    g_vq = vq;

    float32 valpha = vd * cos_e - vq * sin_e;
    float32 vbeta  = vd * sin_e + vq * cos_e;

    float32 dutyA, dutyB, dutyC;
    FOC_SVPWM(valpha, vbeta, &dutyA, &dutyB, &dutyC);

    dutyA = 0.5f + (dutyA - 0.5f) * 0.3f;
    dutyB = 0.5f + (dutyB - 0.5f) * 0.3f;
    dutyC = 0.5f + (dutyC - 0.5f) * 0.3f;

    if (dutyA > 0.8f) dutyA = 0.8f; if (dutyA < 0.2f) dutyA = 0.2f;
    if (dutyB > 0.8f) dutyB = 0.8f; if (dutyB < 0.2f) dutyB = 0.2f;
    if (dutyC > 0.8f) dutyC = 0.8f; if (dutyC < 0.2f) dutyC = 0.2f;

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
    DRV8305_TestSPI();
    DRV8305_WriteReg(0x07, 0x2E6);
    DRV8305_WriteReg(0x0C, 0x001F);
    DRV8305_WriteReg(0x09, 0x022);
    for (volatile int j = 0; j < 100000; j++);
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

    FOC_UART_Print("Calibrating zero current offset...\r\n");
    uint32 sumA = 0, sumB = 0, sumC = 0;
    #define CALIB_SAMPLES 1000
    for (uint32 i = 0; i < CALIB_SAMPLES; i++)
    {
        sumA += FOC_ADC_ReadRaw(7);
        sumB += FOC_ADC_ReadRaw(6);
        sumC += FOC_ADC_ReadRaw(5);
    }
    g_offA = sumA / CALIB_SAMPLES;
    g_offB = sumB / CALIB_SAMPLES;
    g_offC = sumC / CALIB_SAMPLES;
    FOC_UART_Print("Zero offset: A="); FOC_UART_PrintInt(g_offA);
    FOC_UART_Print(" B="); FOC_UART_PrintInt(g_offB);
    FOC_UART_Print(" C="); FOC_UART_PrintInt(g_offC);
    FOC_UART_Print("\r\n");

    FOC_SPI_Init();
    FOC_UART_Print("Encoder Ready\r\n");

    g_running = 0;
    FOC_UART_Print("Aligning encoder zero...\r\n");
    FOC_PWM_SetDutyPercent(0.65f, 0.425f, 0.425f);
    for (volatile int j = 0; j < 5000000; j++);

    uint32 sum_enc = 0;
    #define ENC_CALIB_N 100
    for (uint32 i = 0; i < ENC_CALIB_N; i++)
        sum_enc += FOC_SPI_GetAngleRaw();
    uint16 zero_offset = (uint16)(sum_enc / ENC_CALIB_N);

    FOC_PWM_SetDutyPercent(0.5f, 0.5f, 0.5f);
    for (volatile int j = 0; j < 500000; j++);

    FOC_UART_Print("Zero offset = ");
    FOC_UART_PrintInt((sint32)zero_offset);
    FOC_UART_Print(" (using ENCODER_ZERO_OFFSET=8726)\r\n");

    PID_Init(&g_pid_d, 0.0001f, 0.0f, 0.0f, 0.3f);
    PID_Init(&g_pid_q, 0.0001f, 0.0f, 0.0f, 0.3f);
    g_id_ref = 0.0f;
    g_iq_ref = 0.0f;

    FOC_UART_Print("Current loop starting. iq_ref=0\r\n");
    for (volatile int j = 0; j < 5000000; j++);
    g_running = 1;

    uint32 print_cnt = 0;
    while (1)
    {
        uint16 enc_raw = FOC_SPI_GetAngleRaw();
        sint32 mech = (sint32)enc_raw - (sint32)ENCODER_ZERO_OFFSET;
        if (mech < 0) mech += 16384;
        g_elec_angle = (float32)mech * TWO_PI / 16384.0f * (float32)MOTOR_POLE_PAIRS;
        while (g_elec_angle >= TWO_PI) g_elec_angle -= TWO_PI;

        print_cnt++;
        if (print_cnt >= 200)
        {
            print_cnt = 0;
            FOC_UART_Print("id="); FOC_UART_PrintInt((sint32)g_id);
            FOC_UART_Print(" iq="); FOC_UART_PrintInt((sint32)g_iq);
            FOC_UART_Print(" vd="); FOC_UART_PrintInt((sint32)g_vd);
            FOC_UART_Print(" vq="); FOC_UART_PrintInt((sint32)g_vq);
            FOC_UART_Print("\r\n");
        }
        for (volatile int j = 0; j < 5000; j++);
    }
}