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
#include <math.h>

IfxCpu_syncEvent g_cpuSyncEvent = 0;

#define PIN_LED1 &MODULE_P00, 5
#define PIN_NFAULT &MODULE_P00, 3

#define CURRENT_DIR_A   1
#define CURRENT_DIR_B   1
#define CURRENT_DIR_C   1
#define MOTOR_DIR       1

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

volatile uint8   g_running = 0;
volatile float32 g_elec_angle = 0.0f;

static PID_t g_pid_d;
static PID_t g_pid_q;
static volatile float32 g_id_ref = 0.0f;
static volatile float32 g_iq_ref = -200.0f;

static uint16 g_offA = 2050;
static uint16 g_offB = 2044;
static uint16 g_offC = 2040;
uint16 g_zero_offset = 10372;
static sint32 g_mech = 0;
static uint16 g_lastEnc = 0;

static volatile float32 g_id = 0, g_iq = 0, g_vd = 0, g_vq = 0;
static volatile uint16 g_rawA = 0, g_rawB = 0;
static float32 s_dA = 0.25f, s_dB = 0.25f, s_dC = 0.25f;

IFX_INTERRUPT(FOC_PWM_ISR, 0, 1);
void FOC_PWM_ISR(void)
{
    if (!FOC_PWM_AckIrq()) return;
    if (!g_running) return;

    FOC_ADC_StartSync();
    float32 ia, ib, ic;

    {
        uint16 ra = FOC_ADC_ReadRaw(7);
        uint16 rb = FOC_ADC_ReadRaw(6);
        g_rawA = ra; g_rawB = rb;
        ia = (float32)(ra - g_offA) * CURRENT_DIR_A;
        ib = (float32)(rb - g_offB) * CURRENT_DIR_B;
        ic = -(ia + ib);
    }

    if (MOTOR_DIR == -1) { float32 t = ib; ib = ic; ic = t; }

    float32 ialpha = ia;
    float32 ibeta = (ia + 2.0f * ib) / 1.7320508075688772f;

    float32 sin_e = -sinf(g_elec_angle);
    float32 cos_e =  cosf(g_elec_angle);
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

    if (dutyA > 0.45f) dutyA = 0.45f; if (dutyA < 0.05f) dutyA = 0.05f;
    if (dutyB > 0.45f) dutyB = 0.45f; if (dutyB < 0.05f) dutyB = 0.05f;
    if (dutyC > 0.45f) dutyC = 0.45f; if (dutyC < 0.05f) dutyC = 0.05f;

    s_dA = dutyA; s_dB = dutyB; s_dC = dutyC;

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
        FOC_ADC_StartSync();
        sumA += FOC_ADC_ReadRaw(7);
        sumB += FOC_ADC_ReadRaw(6);
        sumC += FOC_ADC_ReadRaw(5);
    }
    g_offA = (uint16)(sumA / CALIB_SAMPLES);
    g_offB = (uint16)(sumB / CALIB_SAMPLES);
    g_offC = (uint16)(sumC / CALIB_SAMPLES);
    FOC_UART_Print("Zero offset: A="); FOC_UART_PrintInt(g_offA);
    FOC_UART_Print(" B="); FOC_UART_PrintInt(g_offB);
    FOC_UART_Print(" C="); FOC_UART_PrintInt(g_offC);
    FOC_UART_Print("\r\n");

    FOC_SPI_Init();
    FOC_UART_Print("Encoder Ready\r\n");

    g_running = 1;
    FOC_UART_Print("Aligning encoder zero...\r\n");
    FOC_PWM_SetDutyPercent(0.9f, 0.45f, 0.45f);
    for (volatile int j = 0; j < 5000000; j++);

    uint32 sum_enc = 0;
    #define ENC_CALIB_N 100
    for (uint32 i = 0; i < ENC_CALIB_N; i++)
        sum_enc += FOC_SPI_GetAngleRaw();
    uint16 zero_offset = (uint16)(sum_enc / ENC_CALIB_N);
    g_zero_offset = zero_offset;
    g_lastEnc = zero_offset;
    g_mech = 0;

    FOC_PWM_SetDutyPercent(0.25f, 0.25f, 0.25f);
    for (volatile int j = 0; j < 500000; j++);

    FOC_UART_Print("Zero offset = ");
    FOC_UART_PrintInt((sint32)zero_offset);
    FOC_UART_Print("\r\n");

    PID_Init(&g_pid_d, 0.0005f, 0.0002f, 0.0f, 0.5f);
    PID_Init(&g_pid_q, 0.0005f, 0.0002f, 0.0f, 0.5f);
    g_id_ref = 0.0f;
    g_iq_ref = -50.0f;

    FOC_UART_Print("Current loop starting. iq_ref=-200\r\n");
    for (volatile int j = 0; j < 5000000; j++);
    g_running = 1;

    uint32 print_cnt = 0;
    while (1)
    {
        uint16 enc_raw = FOC_SPI_GetAngleRaw();
        sint32 delta = (sint32)enc_raw - (sint32)g_lastEnc;
        if (delta > (sint32)ENCODER_RESOLUTION / 2) delta -= (sint32)ENCODER_RESOLUTION;
        if (delta < -(sint32)ENCODER_RESOLUTION / 2) delta += (sint32)ENCODER_RESOLUTION;
        g_lastEnc = enc_raw;
        g_mech += delta;
        g_elec_angle = (float32)g_mech * TWO_PI / ENCODER_RESOLUTION * (float32)MOTOR_POLE_PAIRS;

        print_cnt++;
        if (print_cnt >= 500)
        {
            print_cnt = 0;
            FOC_UART_Print("ang="); FOC_UART_PrintInt((sint32)(g_elec_angle * 1000.0f));
            FOC_UART_Print(" id="); FOC_UART_PrintInt((sint32)g_id);
            FOC_UART_Print(" iq="); FOC_UART_PrintInt((sint32)g_iq);
            FOC_UART_Print(" vd="); FOC_UART_PrintInt((sint32)(g_vd * 1000.0f));
            FOC_UART_Print(" vq="); FOC_UART_PrintInt((sint32)(g_vq * 1000.0f));
            FOC_UART_Print(" ra="); FOC_UART_PrintInt((sint32)g_rawA);
            FOC_UART_Print(" rb="); FOC_UART_PrintInt((sint32)g_rawB);
            FOC_UART_Print("\r\n");
        }
    }
}
