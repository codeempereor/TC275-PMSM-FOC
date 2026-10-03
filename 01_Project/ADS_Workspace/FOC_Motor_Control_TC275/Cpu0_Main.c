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

static volatile float32 g_speed_ref = 0.0f;     /* 目标转速 机械rad/s（旋钮给定），负=逆时针，打印×100 */
static float32 g_speed_ref_filt = 0.0f;         /* sref 低通滤波，防旋钮跳变 */
static uint16 g_pot_raw = 0;                    /* 电位器原始 12bit ADC 值 */
static float32 g_speed_meas = 0.0f;
static float32 speed_integral = 0.0f;
static sint32 speed_delta_win = 0;
static uint32 speed_t_start = 0;
static uint8 speed_updated = 0;
static volatile uint32 g_isr_cnt = 0;

static volatile uint8  g_sw_dbg = 0;              /* 切换瞬间诊断快照 */
static volatile float32 g_sw_th = 0.0f;
static volatile float32 g_sw_el = 0.0f;

static volatile uint8 g_dir_chk = 0;              /* 磁极方向确认：0=未切，1=确认中，2=已确认 */
static sint32 dir_accum = 0;
static volatile uint32 dir_t_start = 0;

/* 堵转脱困（方案A：电压开环冲击 kick） */
static volatile uint8  g_kick = 0;             /* 1=冲击中 */
static uint16 g_kick_cnt = 0;                  /* 冲击剩余窗数(5ms) */
static uint16 g_stall_cnt = 0;                 /* 堵转连续窗计数 */
static uint8  g_kick_fail = 0;                 /* 连续冲击失败次数 */
static volatile float32 s_duty_hi = 0.45f;     /* 动态 duty 上限，kick 时放宽到 0.9 */
#define STALL_SPD_LIM  0.5f    /* 堵转判定：|spd|<0.5 rad/s */
#define STALL_SREF_MIN 0.5f    /* 堵转判定：|sref|>0.5 rad/s 才判（仅挡零位；低速给定同样需要脱困） */
#define STALL_WINDOWS  40      /* 持续 200ms 判堵转 */
#define KICK_WINDOWS   40      /* 冲击 200ms：长时间推力累积角动量冲出齿槽 */
#define KICK_VOLTAGE   1.0f    /* 冲击电压：满调制（duty 上限 0.95 → 相电压约 13V） */
#define KICK_SPD_EXIT  2.0f    /* 冲击成功判定：spd>2 rad/s */
#define KICK_FAIL_MAX  3       /* 连续失败 3 次停止冲击，防持续大电流 */

static volatile uint8  g_foc_mode = 0;   /* 0=预定位 1=开环加速 2=闭环 */
static volatile float32 g_theta_i = 0.0f;
static volatile float32 g_omega_i = 0.0f;
static volatile uint16 g_prepos_cnt = 0;
#define I_START      200.0f   /* 启动电流幅值(码, 2.3A) */
#define OMEGA_MIN    2.0f     /* 起始电频率 rad/s */
#define OMEGA_MAX    60.0f    /* 切换电频率 rad/s (≈1rev/s 机械) */
#define OMEGA_RAMP   100.0f   /* 频率斜坡 rad/s² */
#define PREPOS_SAMPLES 6000   /* 预定位 200ms 斜坡 + 100ms 稳定 (20kHz) */

IFX_INTERRUPT(FOC_PWM_ISR, 0, 1);
void FOC_PWM_ISR(void)
{
    if (!FOC_PWM_AckIrq()) return;
    if (!g_running) return;
    g_isr_cnt++;

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

    if (g_foc_mode == 0)
    {
        g_theta_i = 0.0f;
        g_prepos_cnt++;
        g_id_ref = (float32)g_prepos_cnt * (I_START / 4000.0f);
        if (g_id_ref > I_START) g_id_ref = I_START;
        g_iq_ref = 0.0f;
        if (g_prepos_cnt > PREPOS_SAMPLES)
        {
            g_prepos_cnt = 0;
            g_foc_mode = 1;
            g_omega_i = OMEGA_MIN;
        }
    }
    else if (g_foc_mode == 1)
    {
        g_omega_i += OMEGA_RAMP * 0.00005f;
        if (g_omega_i > OMEGA_MAX) g_omega_i = OMEGA_MAX;
        g_theta_i += g_omega_i * 0.00005f;
        g_iq_ref = 0.0f;
        float32 err = g_theta_i - g_elec_angle;
        while (err > PI) err -= TWO_PI;
        while (err < -PI) err += TWO_PI;
        if (g_omega_i >= OMEGA_MAX && fabsf(err) < 0.3f)
        {
            g_foc_mode = 2;
            g_id_ref = 0.0f;
            g_iq_ref = -50.0f;
            g_sw_dbg = 1;
            g_sw_th = g_theta_i;
            g_sw_el = g_elec_angle;
            g_dir_chk = 1;
            dir_t_start = g_isr_cnt;
        }
    }

    float32 ang_use = (g_foc_mode == 2) ? g_elec_angle : g_theta_i;
    float32 sin_e = -sinf(ang_use);
    float32 cos_e =  cosf(ang_use);
    float32 id  =  ialpha * cos_e + ibeta * sin_e;
    float32 iq  = -ialpha * sin_e + ibeta * cos_e;

    g_id = id;
    g_iq = iq;

    float32 vd, vq, valpha, vbeta;
    if (g_kick)
    {
        /* 堵转脱困：电压开环冲击（不闭环，采样失效无碍），方向跟随 sref */
        vd = 0.0f;
        vq = (g_speed_ref < 0.0f) ? -KICK_VOLTAGE : KICK_VOLTAGE;
        valpha = vd * cos_e - vq * sin_e;
        vbeta  = vd * sin_e + vq * cos_e;
        g_vd = vd;
        g_vq = vq;
    }
    else
    {
        vd = PID_Calc(&g_pid_d, g_id_ref, id);
        vq = PID_Calc(&g_pid_q, g_iq_ref, iq);
        valpha = vd * cos_e - vq * sin_e;
        vbeta  = vd * sin_e + vq * cos_e;
        g_vd = vd;
        g_vq = vq;
    }

    float32 dutyA, dutyB, dutyC;
    FOC_SVPWM(valpha, vbeta, &dutyA, &dutyB, &dutyC);

    if (dutyA > s_duty_hi) dutyA = s_duty_hi; if (dutyA < 0.05f) dutyA = 0.05f;
    if (dutyB > s_duty_hi) dutyB = s_duty_hi; if (dutyB < 0.05f) dutyB = 0.05f;
    if (dutyC > s_duty_hi) dutyC = s_duty_hi; if (dutyC < 0.05f) dutyC = 0.05f;

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

    PID_Init(&g_pid_d, 0.0005f, 0.0002f, 0.0f, 0.2f);  /* vd 限幅 0.2：防电压矢量被拉偏致卡死 */
    PID_Init(&g_pid_q, 0.0005f, 0.0002f, 0.0f, 0.5f);
    g_id_ref = 0.0f;
    g_iq_ref = -50.0f;

    FOC_UART_Print("I/f startup: prepos->ramp->closed\r\n");
    for (volatile int j = 0; j < 5000000; j++);
    g_running = 1;

    uint32 print_cnt = 0;
    while (1)
    {
        uint16 enc_raw = FOC_SPI_GetAngleRaw();
        sint32 delta = (sint32)enc_raw - (sint32)g_lastEnc;
        if (delta > (sint32)ENCODER_RESOLUTION / 2) delta -= (sint32)ENCODER_RESOLUTION;
        if (delta < -(sint32)ENCODER_RESOLUTION / 2) delta += (sint32)ENCODER_RESOLUTION;
        if (delta > 4096 || delta < -4096)   /* 坏帧过滤：单次位置跳变超限丢弃 */
        {
            g_lastEnc = enc_raw;
            continue;
        }
        g_lastEnc = enc_raw;
        g_mech += delta;
        speed_delta_win += delta;
        g_elec_angle = (float32)g_mech * TWO_PI / ENCODER_RESOLUTION * (float32)MOTOR_POLE_PAIRS;

        if (g_isr_cnt - speed_t_start >= 100)   /* 100 ISR = 5ms @20kHz */
        {
            float32 dt_s = (float32)(g_isr_cnt - speed_t_start) * 0.00005f;
            if (dt_s > 1e-4f && dt_s < 1.0f)
            {
                float32 raw_speed = (float32)speed_delta_win / ENCODER_RESOLUTION * TWO_PI / dt_s;
                g_speed_meas = g_speed_meas * 0.8f + raw_speed * 0.2f;
                speed_updated = 1;
            }
            speed_delta_win = 0;
            speed_t_start = g_isr_cnt;
        }

        if (g_dir_chk == 1)
        {
            dir_accum += delta;
            if (dir_accum > 400)                   /* 顺转 0.15 rad → 磁极反 π → 翻转 */
            {
                g_mech += (sint32)ENCODER_RESOLUTION / MOTOR_POLE_PAIRS / 2;
                dir_accum = 0;
                g_dir_chk = 2;
            }
            else if (dir_accum < -400)             /* 逆转 0.15 rad → 磁极对，放行 */
            {
                dir_accum = 0;
                g_dir_chk = 2;
            }
            else if (g_isr_cnt - dir_t_start >= 40000)  /* 超时兜底：不判定则放行 */
            {
                dir_accum = 0;
                g_dir_chk = 2;
            }
        }

        /* 旋钮调速：每 100 次主循环读一次电位器（G0CH0=AN0，12bit 0-4095）→ sref 0..-12 rad/s */
        static uint16 pot_cnt = 0;
        if (++pot_cnt >= 100)
        {
            pot_cnt = 0;
            FOC_ADC_TriggerPot();
            g_pot_raw = FOC_ADC_ReadRaw(0);
            float32 target = (g_pot_raw < 40) ? 0.0f
                            : -(float32)(g_pot_raw - 40) / 4055.0f * 12.0f;  /* 死区40码后线性升到 -12 */
            g_speed_ref_filt = g_speed_ref_filt * 0.9f + target * 0.1f;       /* 一阶低通防跳变 */
            g_speed_ref = g_speed_ref_filt;
            if (fabsf(g_speed_ref) < 0.5f) { speed_integral = 0.0f; g_kick_fail = 0; }  /* 停转位清积分/失败计数：旋钮回零必须能停 */
        }

        if (speed_updated && g_foc_mode == 2 && g_dir_chk == 2)
        {
            if (g_kick)
            {
                /* 冲击进行中：每 5ms 检查一次，转起来/超时/旋钮回零即退出 */
                speed_integral = 0.0f;
                if (--g_kick_cnt == 0)
                {
                    g_kick = 0;
                    s_duty_hi = 0.45f;
                    if (++g_kick_fail >= KICK_FAIL_MAX)
                    {
                        g_kick_fail = KICK_FAIL_MAX;
                        g_iq_ref = -50.0f;   /* 3次失败：降回低力矩等待（防持续大电流发热），旋钮回零后复位 */
                    }
                }
                else if (fabsf(g_speed_meas) > KICK_SPD_EXIT || fabsf(g_speed_ref) < 1.0f)
                {
                    g_kick = 0;
                    s_duty_hi = 0.45f;
                    g_kick_fail = 0;   /* 冲击成功/旋钮回零：失败计数清零 */
                }
                speed_updated = 0;
            }
            else
            {
                if (g_speed_meas * g_speed_ref < -30.0f && fabsf(g_speed_meas) > 3.0f)
                {
                    g_mech += (sint32)ENCODER_RESOLUTION / MOTOR_POLE_PAIRS / 2;  /* 磁极反 → 翻 π */
                    speed_integral = 0.0f;
                    g_iq_ref = -50.0f;
                    speed_updated = 0;
                    continue;
                }
                float32 err_speed = g_speed_ref - g_speed_meas;
                sint32 windup = (err_speed > 0.0f && g_iq_ref >= 149.0f) ||
                               (err_speed < 0.0f && g_iq_ref <= -149.0f);
                if (!windup) speed_integral += err_speed;
                if (speed_integral > 3000.0f) speed_integral = 3000.0f;
                if (speed_integral < -3000.0f) speed_integral = -3000.0f;
                float32 new_iq = 0.5f * err_speed + 0.05f * speed_integral;
                float32 dq = new_iq - g_iq_ref;
                if (dq > 6.0f) dq = 6.0f;
                if (dq < -6.0f) dq = -6.0f;
                g_iq_ref += dq;
                if (g_iq_ref > 150.0f) g_iq_ref = 150.0f;
                if (g_iq_ref < -150.0f) g_iq_ref = -150.0f;

                /* 堵转检测：spd≈0 且 有给定（不依赖 iqr 饱和——kick 失败后 iqr 回落到 -6 附近不饱和，
                 * 若靠饱和判据则 kick 永远无法重触发）→ 持续 200ms 触发冲击脱困 */
                if (fabsf(g_speed_meas) < STALL_SPD_LIM && fabsf(g_speed_ref) > STALL_SREF_MIN
                    && g_kick_fail < KICK_FAIL_MAX)
                {
                    if (++g_stall_cnt >= STALL_WINDOWS)
                    {
                        g_stall_cnt = 0;
                        g_kick = 1;
                        g_kick_cnt = KICK_WINDOWS;
                        s_duty_hi = 0.95f;
                        speed_integral = 0.0f;
                    }
                }
                else
                {
                    g_stall_cnt = 0;
                }
                speed_updated = 0;
            }
        }

        if (g_sw_dbg)
        {
            g_sw_dbg = 0;
            FOC_UART_Print("SW th="); FOC_UART_PrintInt((sint32)(g_sw_th * 1000.0f));
            FOC_UART_Print(" el="); FOC_UART_PrintInt((sint32)(g_sw_el * 1000.0f));
            FOC_UART_Print("\r\n");
        }

        print_cnt++;
        if (print_cnt >= 500)
        {
            print_cnt = 0;
            FOC_UART_Print("md="); FOC_UART_PrintInt((sint32)g_foc_mode);
            FOC_UART_Print(" ang="); FOC_UART_PrintInt((sint32)(g_elec_angle * 1000.0f));
            FOC_UART_Print(" id="); FOC_UART_PrintInt((sint32)g_id);
            FOC_UART_Print(" iq="); FOC_UART_PrintInt((sint32)g_iq);
            FOC_UART_Print(" vd="); FOC_UART_PrintInt((sint32)(g_vd * 1000.0f));
            FOC_UART_Print(" vq="); FOC_UART_PrintInt((sint32)(g_vq * 1000.0f));
            FOC_UART_Print(" ra="); FOC_UART_PrintInt((sint32)g_rawA);
            FOC_UART_Print(" rb="); FOC_UART_PrintInt((sint32)g_rawB);
            FOC_UART_Print(" spd="); FOC_UART_PrintInt((sint32)(g_speed_meas * 100.0f));
            FOC_UART_Print(" iqr="); FOC_UART_PrintInt((sint32)g_iq_ref);
            FOC_UART_Print(" sref="); FOC_UART_PrintInt((sint32)(g_speed_ref * 100.0f));
            FOC_UART_Print(" pot="); FOC_UART_PrintInt((sint32)g_pot_raw);
            FOC_UART_Print(" k="); FOC_UART_PrintInt((sint32)g_kick);
            FOC_UART_Print("\r\n");
        }
    }
}
