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
static volatile float32 g_id_filt = 0, g_iq_filt = 0;  /* 电流测量一阶低通：堵转大电流工况 duty 摆动致采样噪声大，
                                                        * 20 倍电流环增益会放大成 vd/vq 饱和振荡，必须先滤波 */
static volatile uint16 g_rawA = 0, g_rawB = 0;
static float32 s_dA = 0.25f, s_dB = 0.25f, s_dC = 0.25f;
static volatile uint16 g_smp_bad_cnt = 0;   /* 采样超限连续窗计数（保护触发 100 窗） */
static volatile uint16 g_smp_ok_cnt = 0;    /* 保护后正常窗计数（恢复 200 窗） */
static volatile uint8  g_smp_prot = 0;      /* 采样保护锁存：1 → duty 上限降 0.45 */

static volatile float32 g_speed_ref = 0.0f;     /* 目标转速 机械rad/s（旋钮给定），负=逆时针，打印×100 */
static float32 g_speed_ref_filt = 0.0f;         /* sref 低通滤波，防旋钮跳变 */
static volatile float32 g_spd_ramp = 0.0f;      /* 速度环目标斜坡：5ms 爬 0.1 rad/s(20 rad/s²)，
                                                * 0.4s 平滑升到 8，防启动即大误差→力矩饱和→狂加速过冲→反向振荡("一卡一卡") */
static uint16 g_pot_raw = 0;                    /* 电位器原始 12bit ADC 值 */
static float32 g_speed_meas = 0.0f;
static float32 g_spd_prev = 0.0f;   /* 速度环 D 项（微分阻尼）：上一窗滤波转速 */
/* I/f 同步校验（防失步/反向切环→失控高速）：转子电角增量须与磁场方向一致 */
static uint32 g_if_dt_cnt = 0;      /* 5ms 方向校验窗口计数 */
static float32 g_el_prev_if = 0.0f; /* 上一校验窗口电角 */
static uint32 g_if_sync_cnt = 0;    /* 连续同步窗口数（≥8 才允许切环） */
static uint32 g_if_stall_cnt = 0;   /* 同步失败连续窗口数（≥20 → 磁极反，翻 π 重试） */
static uint32 g_if_flip_cnt = 0;    /* 翻 π 重试次数（限 3） */
static uint32 g_ovspd_cnt = 0;      /* 闭环超速失控窗口计数（≥20 → 停机翻 π 重启） */
static uint32 g_boost_cnt = 0;      /* 切环力矩 boost 剩余主循环窗数：150ms 冲出齿槽 */
static float32 speed_integral = 0.0f;
static sint32 speed_delta_win = 0;
static uint32 speed_t_start = 0;
static uint8 speed_updated = 0;
static uint8 g_dir_flip = 0;        /* 磁极/参考系方向自检：切环瞬间 Park 参考系与转子
                                     * 电角可能差 π(负载角/切环相位随机) → iq 负可能产生
                                     * 正力矩(实测 42a18b6 负向 vs b8efcb6 正转 5.7 圈)。
                                     * spd 与 sref 明显反向 → 磁极反 → iq 输出全部取反 */
static volatile uint32 g_isr_cnt = 0;

static volatile uint8  g_sw_dbg = 0;              /* 切换瞬间诊断快照 */
static volatile float32 g_sw_th = 0.0f;
static volatile float32 g_sw_el = 0.0f;

static volatile uint8 g_dir_chk = 0;              /* 磁极方向确认：0=未切，1=确认中，2=已确认 */
static sint32 dir_accum = 0;
static volatile uint32 dir_t_start = 0;

/* 堵转脱困+低速拖动（开环旋转磁通，低电流限流，3A 电源友好）
 * mode0=低速拖动：|sref|<LOW_SPD_MAX 时磁场斜坡加速到目标电转速，平滑起步
 * mode1=堵转脱困：|sref|≥LOW_SPD_MAX 且 spd≈0 时慢速扫角翻越齿槽 */
static volatile uint8  g_kick = 0;             /* 1=开环拖动/脱困中 */
static float32 g_sweep_angle = 0.0f;           /* 开环当前电角度 */
static float32 g_sweep_start = 0.0f;           /* 开环起始电角度 */
static float32 g_sweep_dir  = 0.0f;            /* 方向 ±1（跟随 sref） */
static float32 g_sweep_iq   = 0.0f;            /* q 电流给定（码值，限流） */
static float32 g_sweep_spd  = 0.0f;            /* 当前开环电角速度 rad/s */
static uint8  g_sweep_mode = 0;                /* 0=低速拖动 1=堵转脱困 */
static uint16 g_stall_cnt = 0;                 /* 堵转连续窗计数 */
static uint16 g_torq_pump_cnt = 0;             /* 力矩预充连续窗计数（起步/堵转瞬间满力矩冲齿槽） */
static uint8  g_kick_fail = 0;                 /* 连续失败次数 */
static uint16 g_fail_cool_cnt = 0;             /* 失败冷却窗计数（自动重试） */
static uint16 g_kick_timer = 0;                /* 堵转直推计时：1.7A 垂直直推 2s(400 窗)
                                               * 转子仍不动 → 失败冷却重试（超强齿槽物理极限） */
static uint16 g_pole_chk_cnt = 0;              /* 磁极方向自检连续窗计数 */
#define STALL_SPD_LIM  0.5f    /* 堵转判定：|spd|<0.5 rad/s */
#define STALL_SREF_MIN 0.5f    /* 有给定（仅挡零位） */
#define STALL_WINDOWS  40      /* 持续 200ms 判堵转 */
#define LOW_SPD_MAX    4.0f    /* 低速区上限：|sref|<4 rad/s 直接开环拖动（平滑起步） */
#define SWEEP_SPD      2.5f    /* 堵转脱困扫角速度（电 rad/s）：1.5 慢速扫角 1.5A 在最强
                                * 齿槽位推不动(实测 -854k 位 iqr+100 卡)；2.5 转子惯性能量
                                * ∝v² → 冲过强齿槽，扫角后速度环接管 */
#define SWEEP_RAMP     100.0f  /* 低速拖动加速斜坡 rad/s²（0→目标电转速平滑） */
#define SWEEP_ANG_MAX  6.283f  /* 最大扫过 2π 电角（36° 机械，必翻越齿槽） */
#define SWEEP_IREF     150.0f  /* 拖动/脱困 q 电流 ≈1.7A（3A 电源内，单相 1.7A<3A 安全）。
                                * 130(1.5A) 在最强齿槽位(实测 -213k/-873k)拖停后扫角
                                * 需 1.5s 恢复；150 冲力更大一次冲过概率高 → 卡位更短 */
#define SWEEP_EXIT_SPD 0.5f    /* 脱困成功判定：转子动起来 */
#define KICK_FAIL_MAX  3       /* 连续失败 3 次 → 冷却 2 秒自动重试（无需回零） */
#define FAIL_COOL_WINDOWS 400  /* 冷却窗 = 400×5ms = 2s */
#define BASE_SPEED     8.0f    /* 旋钮 0% 基础转速 rad/s（≈76 RPM，闭环稳定区起步） */
#define MAX_SPEED      12.0f   /* 旋钮 100% 顶速 rad/s（duty 0.80 电压 19.2V，EMF 余量充足） */

/* === 架构改造（duty 0.45→0.80 + 采样保护）2026-10-03 ===
 * 根因：DRV8305 低侧分流采样，采样点在 PWM 周期端点（T12 OneMatch @ 0us，
 * 低侧导通窗口起点）。duty 0.45 窗口 13.75us 三相采样安全；duty 放开后窗口
 * 收窄（0.80→4us、0.90→1.5us）→ 三相转换(~2us)出窗口 → C 相失效。
 * 改造：①ADC 只扫 A/B 两相（C 重构）→ 转换时间减 1/3；
 *       ②duty 上限 0.45→0.80（窗口 4us，两相转换 ~1.2-2us 安全余量 2us）；
 *       ③电压标幺 0.5→0.80（PID outMax 同步）→ 电流环不再饱和 → 齿槽
 *         扰动被压住 → 转速恒定；
 *       ④采样原始值范围保护：raw 持续超限 500ms → 自动降 duty 至 0.45 +
 *         UART 告警，恢复 1s 后自动复原（防高 duty 采样失效失控，兜底）。 */
#define DUTY_MAX       0.80f   /* duty 钳位上限（低侧窗口 4us，两相采样安全值） */
#define VOLT_MAX       0.80f   /* 电流环电压标幺上限（PID outMax，SVPWM duty 同步） */
#define RAW_LIM_HI     3500    /* 采样 raw 合理上限（2048±1548，超限=采样失效） */
#define RAW_LIM_LO     500     /* 采样 raw 合理下限 */
#define SMP_BAD_WIN    100     /* 连续 100 窗(500ms) 采样超限 → 降级保护 */
#define SMP_REC_WIN    200     /* 保护后连续 200 窗(1s) 正常 → 自动恢复 */

static volatile uint8  g_foc_mode = 0;   /* 0=预定位 1=开环加速 2=闭环 */
static volatile float32 g_theta_i = 0.0f;
static volatile float32 g_omega_i = 0.0f;
static volatile uint16 g_prepos_cnt = 0;
#define I_START      200.0f   /* 预定位强拉电流(码, 2.3A)：定位必须足强克服齿槽 → 磁极方向唯一 */
#define IF_ID        130.0f   /* I/f 拖动力矩(码, 1.5A)：150 时转子跟上磁场(7.5 vs 8.5) ✓
                               * 旧架构(duty 0.45)电压需求高 → duty 顶 0.45 边界 → 采样失效(尖峰)，
                               * 故取 130 居中。新架构(duty 0.80+两相采样)电压余量大 →
                               * 150 不再顶边界，130 保守保持（启动相关，非本次改造点） */
#define OMEGA_MIN    2.0f     /* 起始电频率 rad/s */
#define OMEGA_MAX    20.0f    /* 切换电频率 rad/s (≈2 机械 rad/s)：35(3.5) 超 V/f 开环拖动
                               * 同步能力——实测 vd=0.7V 转子稳定 2 rad/s 连续转 6.7 圈但磁场
                               * 3.5 追不上 → 失步 → sync 永远凑不齐 → 永不切环("只有1没有2")。
                               * 20=转子实测同步速度 → V/f 同步 → sync 累计 → 切环 → 闭环爬 8 */
#define OMEGA_RAMP   25.0f    /* 频率斜坡 rad/s²（3.4s 到 85）：云台电机惯量大，
                               * 100 rad/s² 加速过快会丢步（转子没跟上就切闭环→启动全靠预充→
                               * 预充冲出过头→速度环刹车过头→走走停停→卡死） */
#define PREPOS_SAMPLES 12000  /* 预定位 400ms 斜坡 + 200ms 稳定 (20kHz)：云台电机齿槽强，
                               * 6000 窗(300ms)不够转子到位 → 每次上电停在不同齿槽位 →
                               * 磁极方向随机（本次转子反向爬一整圈=方向判定反的实证） */

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
        float32 dir_if = (g_speed_ref < 0.0f) ? -1.0f : 1.0f;   /* I/f 启动方向跟随旋钮给定：
                                                                * 切闭环后速度环方向一致，无反向掰转速的堵转 */
        g_theta_i += dir_if * g_omega_i * 0.00005f;
        g_iq_ref = 0.0f;
        g_id_ref = IF_ID;   /* I/f 励磁 1.7A：200 会 vd 饱和电流建立不足（拖动力矩弱 → 丢步），
                             * 150 进线性区电流实际建立 */
        float32 err = g_theta_i - g_elec_angle;
        while (err > PI) err -= TWO_PI;
        while (err < -PI) err += TWO_PI;
        /* 同步校验：每 100 窗(5ms)累计一次电角增量，方向与磁场一致才算同步。
         * 实测无校验时：转子反向微动/失步也会切环（角度差每 2π 重合）→ 切环方向错 →
         * 失控加速到 1000 RPM（反电动势>驱动电压刹不住）→ 旋钮调速无效 */
        if (++g_if_dt_cnt >= 100)
        {
            g_if_dt_cnt = 0;
            float32 el_delta = g_elec_angle - g_el_prev_if;
            g_el_prev_if = g_elec_angle;
            if (el_delta > PI) el_delta -= TWO_PI;
            if (el_delta < -PI) el_delta += TWO_PI;
            if (dir_if * el_delta > 0.001f)
            {
                if (g_if_sync_cnt < 50) g_if_sync_cnt++;   /* 累计制：抖动时偶发反向不再清零，
                                                            * 否则连续 8 窗永远凑不齐 → 切环死锁
                                                            * （实测转子在转但 22s 不切环，一直开环抖） */
                g_if_stall_cnt = 0;
            }
            else if (g_omega_i >= OMEGA_MAX)
            {
                g_if_stall_cnt++;
            }
        }
        /* 切环：sync≥20(100ms 方向确认)即切。去掉 err<0.3 与 spd>1.0 两个判据——
         * ①err=θ_i-el 在 V/f 同步态=负载角，实测常 >0.3 → 永不满足 → 永不切环；
         * ②spd 由 5ms 编码器增量测出，转子 2 rad/s 时窗内仅 ~26 码，噪声使
         * 读数频繁 <1.0 → 同样永卡。sync 累计本身已证明转子同步且在转，
         * 是唯一可靠判据；切环有 boost+预充+ramp 锁目标保护过渡 */
        if (g_omega_i >= OMEGA_MAX && g_if_sync_cnt >= 20)
        {
            g_foc_mode = 2;
            g_id_ref = 0.0f;
            g_iq_ref = -20.0f;   /* 切闭环直接给稳态力矩（0.23A）：I/f 8.5 → 目标 8 只需微减速，
                                  * 不经过转速零点 → 不卡齿槽。任何高于稳态的初值回落时都会拉崩转速 */
            /* 积分器预充：让速度环首拍输出接近稳态力矩（-20 码）。
             * 用温和 ±100 而非公式 -400：深负偏置会压死超速刹车（0.04×(-400)=-16 抵消 Kp），
             * 造成冲出-掉速-预充循环；±100 下超速分支（纯比例）可正常刹住 */
            speed_integral = (g_speed_ref < 0.0f) ? -100.0f : 100.0f;
            if (speed_integral > 3000.0f) speed_integral = 3000.0f;
            if (speed_integral < -3000.0f) speed_integral = -3000.0f;
            g_sw_dbg = 1;
            g_sw_th = g_theta_i;
            g_sw_el = g_elec_angle;
            g_dir_chk = 2;   /* I/f 方向已跟随旋钮 → 切闭环时方向已知，跳过方向确认窗，
                              * 速度环/力矩预充立即接管（否则 iqr=-20 原地抖 2 秒等超时） */
            g_dir_flip = 0;  /* 方向自检清零：切环后检测到反向再翻转 */
            g_spd_ramp = g_speed_ref;  /* 斜坡直接锁目标（不归零重爬）：boost 结束速度环首拍即稳态区 */
            g_boost_cnt = 40;          /* 切环力矩 boost：200ms 1.15A 冲出齿槽势阱。43e62b0
                                        * 验证 1.15A 是唯一够冲的力矩；100ms 在强齿槽切环位
                                        * (实测 -6047 位)冲不出 → 扫角接管启动延迟 ~3s；
                                        * 200ms 冲量翻倍强位也能冲出，最差不劣化 */
            dir_t_start = g_isr_cnt;
        }
        else if (g_omega_i >= OMEGA_MAX && g_if_stall_cnt >= 20 && g_if_flip_cnt < 3)
        {
            /* 磁场转满速后 1s 转子仍不同步（磁极反）→ 翻 π 重置 I/f 重试（限 3 次） */
            g_if_flip_cnt++;
            g_mech += (sint32)ENCODER_RESOLUTION / MOTOR_POLE_PAIRS / 2;
            g_theta_i = g_elec_angle;
            g_omega_i = OMEGA_MIN;
            g_if_sync_cnt = 0;
            g_if_stall_cnt = 0;
        }
    }

    float32 ang_use;
    if (g_kick)
    {
        if (g_sweep_mode == 0)
        {
            /* 低速拖动：磁场斜坡加速到目标电转速（|sref|×极对数），转子同步平滑起步 */
            float32 tgt = fabsf(g_speed_ref) * (float32)MOTOR_POLE_PAIRS;
            g_sweep_spd += SWEEP_RAMP * 0.00005f;
            if (g_sweep_spd > tgt) g_sweep_spd = tgt;
            g_sweep_angle += g_sweep_dir * g_sweep_spd * 0.00005f;  /* 开环磁通旋转 */
            ang_use = g_sweep_angle;
        }
        else
        {
            /* 堵转脱困 → 编码器定向直推：Park 锁定编码器实时角（垂直转子）+ iq=±SWEEP_IREF
             * = 最大冲出力矩。磁场旋转扫角在超强齿槽位无效（实测 641a8a1：磁场转 7.5 电rad
             * 转子 3s 不动 -6128→-6638）——转子不动时磁场力臂斜、有效推力小；
             * 编码器定向始终垂直转子 → 1.7A 全程最大推力，转子一动（spd>2）闭环接管 */
            ang_use = g_elec_angle;
        }
    }
    else
    {
        ang_use = (g_foc_mode == 2) ? g_elec_angle : g_theta_i;
    }
    float32 sin_e = -sinf(ang_use);
    float32 cos_e =  cosf(ang_use);
    float32 id  =  ialpha * cos_e + ibeta * sin_e;
    float32 iq  = -ialpha * sin_e + ibeta * cos_e;

    g_id = id;
    g_iq = iq;
    g_id_filt = g_id_filt * 0.85f + id * 0.15f;
    g_iq_filt = g_iq_filt * 0.85f + iq * 0.15f;   /* 强滤波：高速下固定中点采样偶发尖峰
                                                  * （实测 iq 瞬时 ±654=7.5A、ra/rb 1433/2480），
                                                  * 0.7/0.3 太轻 → 电流环被打飞 → vd/vq 饱和乱控 → 超级抖 */

    float32 vd, vq, valpha, vbeta;
    if (g_kick)
    {
        /* 扫角脱困：d 轴控 0，q 轴恒定小电流给定（电流环限流，电压不会饱和超流），
         * 磁通随 g_sweep_angle 旋转，转子被磁场拉着逐步翻越齿槽 */
        vd = PID_Calc(&g_pid_d, g_id_ref, g_id_filt);
        vq = PID_Calc(&g_pid_q, g_sweep_iq, g_iq_filt);
        valpha = vd * cos_e - vq * sin_e;
        vbeta  = vd * sin_e + vq * cos_e;
        g_vd = vd;
        g_vq = vq;
    }
    else
    {
        /* I/f 段开环励磁电压直驱（V/f）：vd 直给 0.5V 不跑 d 轴电流环——
         * 闭环建立励磁被采样污染不可靠（实测 id 只 ~0.6-1.1A、vd 恒饱和，
         * 拖动力矩时有时无 → 转子在齿槽位摆，启动成败靠运气）。
         * 开环：转子不动时反电动势≈0 → 电流 (0.5-Rs)/R≈1.7A 足力矩拖动；
         * 转子转起后反电动势升 → 电流自动回落（天然恒流）→ 可靠拖动。
         * vq=0 无 iq 扰动（iq 环追 0 会被采样尖峰打飞 → 抖） */
        vd = (g_foc_mode == 1) ? 0.7f : PID_Calc(&g_pid_d, g_id_ref, g_id_filt);
        vq = (g_foc_mode == 1) ? 0.0f : PID_Calc(&g_pid_q, g_iq_ref, g_iq_filt);
        valpha = vd * cos_e - vq * sin_e;
        vbeta  = vd * sin_e + vq * cos_e;
        g_vd = vd;
        g_vq = vq;
    }

    float32 dutyA, dutyB, dutyC;
    FOC_SVPWM(valpha, vbeta, &dutyA, &dutyB, &dutyC);

    float32 duty_max = g_smp_prot ? 0.45f : DUTY_MAX;   /* 采样保护触发 → 降回 0.45 安全区 */
    if (dutyA > duty_max) dutyA = duty_max; if (dutyA < 0.05f) dutyA = 0.05f;
    if (dutyB > duty_max) dutyB = duty_max; if (dutyB < 0.05f) dutyB = 0.05f;
    if (dutyC > duty_max) dutyC = duty_max; if (dutyC < 0.05f) dutyC = 0.05f;

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

    /* 电流环：Kp=0.01/Ki=0.001（高带宽，1 码=0.0115A → 150 码误差立即输出 1.5V 饱和电压，
     * 大电流参考 ~1.7A 可在 ~1ms 内建立）。旧 Kp=0.0005 时大电流误差输出仅 0.075V、
     * 积分爬满才 0.2V → 1.7A 参考永远建立不起来 → 堵转时力矩≈0 → 电机"滋滋声抖动但不转" */
    PID_Init(&g_pid_d, 0.01f, 0.001f, 0.0f, VOLT_MAX);
    PID_Init(&g_pid_q, 0.01f, 0.001f, 0.0f, VOLT_MAX);
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
        if (delta > 800) delta = 800;   /* 单窗增量钳位（±800 码=0.05圈）：切环瞬间电磁噪声
                                         * 误读大跳被钳成小假值——丢弃会冻结 g_mech（角度卡死），
                                         * 钳位让真实运动(<800码/窗)不受影响、假角度有限、
                                         * spd 计算同源被钳 → 速度环不被假值打飞 */
        else if (delta < -800) delta = -800;
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
                float32 prev_spd = g_speed_meas;
                /* 单窗速度最多变化 2 rad/s：切环瞬间编码器误读尖峰(±50)被钳住，
                 * 但真实加速能跟随（每窗+2 爬升，不锁死——保持旧值会让速度环
                 * 永远看到旧速度 → 刹不住 → 转子带着错误目标转飞） */
                if (raw_speed > prev_spd + 2.0f) raw_speed = prev_spd + 2.0f;
                else if (raw_speed < prev_spd - 2.0f) raw_speed = prev_spd - 2.0f;
                g_speed_meas = prev_spd * 0.92f + raw_speed * 0.08f;   /* 低通 0.9/0.1→0.92/0.08：
                                                                       * 稳定段 spd ±15% 波动
                                                                       * 再收一点(实测 -760~-1000) */
                speed_updated = 1;
            }
            speed_delta_win = 0;
            speed_t_start = g_isr_cnt;
        }

        /* 采样质量保护（每窗 5ms 检查）：raw 超限 = 采样失效（低侧窗口出界/ADC 错位）。
         * 持续 500ms → duty 上限降回 0.45（ISR 读 g_smp_prot 生效）+ 串口告警；
         * 恢复正常 1s → 自动复原。防高 duty 放开后偶发采样失效失控。 */
        if (g_rawA > RAW_LIM_HI || g_rawA < RAW_LIM_LO ||
            g_rawB > RAW_LIM_HI || g_rawB < RAW_LIM_LO)
        {
            g_smp_bad_cnt++;
            g_smp_ok_cnt = 0;
            if (g_smp_bad_cnt >= SMP_BAD_WIN && !g_smp_prot)
            {
                g_smp_prot = 1;
                FOC_UART_Print("\r\nSMP_PROT duty->0.45\r\n");
            }
        }
        else
        {
            if (g_smp_bad_cnt > 0) g_smp_bad_cnt--;
            if (g_smp_prot)
            {
                if (++g_smp_ok_cnt >= SMP_REC_WIN)
                {
                    g_smp_ok_cnt = 0;
                    g_smp_prot = 0;
                    FOC_UART_Print("\r\nSMP_REC duty->0.80\r\n");
                }
            }
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
            /* 旋钮映射（新控制律）：0% → BASE_SPEED 稳定高速，100% → MAX_SPEED 顶速。
             * 全程工作点落在闭环稳定高速区，绕开低速齿槽爬行/堵转振荡 */
            float32 pot_norm = (float32)(g_pot_raw - 40) / 4055.0f;
            if (pot_norm < 0.0f) pot_norm = 0.0f;
            if (pot_norm > 1.0f) pot_norm = 1.0f;
            float32 target = -(BASE_SPEED + pot_norm * (MAX_SPEED - BASE_SPEED));
            g_speed_ref_filt = g_speed_ref_filt * 0.9f + target * 0.1f;       /* 一阶低通防跳变 */
            g_speed_ref = g_speed_ref_filt;
            if (fabsf(g_speed_ref) < 0.5f) { speed_integral = 0.0f; g_kick_fail = 0; }  /* 停转位清积分/失败计数：旋钮回零必须能停 */
        }

        if (speed_updated && g_foc_mode == 2 && g_dir_chk == 2)
        {
            if (g_kick)
            {
                /* 开环拖动进行中：每 5ms 检查——mode0 升速/回零退出，mode1 脱困成功/失败退出 */
                speed_integral = 0.0f;
                float32 swept = fabsf(g_sweep_angle - g_sweep_start);
                if (g_sweep_mode == 0)
                {
                    if (fabsf(g_speed_ref) >= LOW_SPD_MAX || fabsf(g_speed_ref) < 0.5f)
                    {
                        g_kick = 0;
                        g_kick_fail = 0;   /* 升到中高速切闭环 / 旋钮回零 */
                        g_fail_cool_cnt = 0;
                        g_spd_ramp = g_speed_ref;  /* 斜坡锁定目标，防从 0 重爬丢力矩 */
                    }
                    else if (fabsf(g_speed_meas) < 0.3f && swept >= SWEEP_ANG_MAX)
                    {
                        g_kick = 0;   /* 磁场转完 2π 转子没跟 → 拖不动（负载过重） */
                        g_spd_ramp = g_speed_ref;
                        if (++g_kick_fail >= KICK_FAIL_MAX)
                        {
                            g_kick_fail = KICK_FAIL_MAX;
                            g_iq_ref = -50.0f;   /* 3次失败：冷却等待，旋钮回零后复位 */
                        }
                    }
                }
                else
                {
                    if (fabsf(g_speed_meas) > SWEEP_EXIT_SPD)
                    {
                        g_kick = 0;
                        g_kick_fail = 0;   /* 脱困成功：转子动起来，闭环接管 */
                        g_fail_cool_cnt = 0;
                        g_spd_ramp = g_speed_ref;  /* 斜坡直接锁定目标：防退出后从 0 重爬，
                                                    * 积分清零+力矩丢失 → 又掉回爬行-卡死循环 */
                    }
                    else if (fabsf(g_speed_ref) < LOW_SPD_MAX && fabsf(g_speed_ref) > 0.5f)
                    {
                        g_sweep_mode = 0;   /* 旋钮拧回低速区：无缝切换为低速拖动（重新斜坡到新目标） */
                        g_sweep_spd = 0.0f;
                        g_sweep_start = g_sweep_angle;
                        g_sweep_iq = (g_speed_ref < 0.0f) ? -SWEEP_IREF : SWEEP_IREF;
                        g_sweep_dir = (g_speed_ref < 0.0f) ? -1.0f : 1.0f;
                    }
                    else if (++g_kick_timer >= 400)   /* 直推 2s 转子仍不动 → 超强齿槽，
                                                       * 1.7A 是单相上限 → 失败冷却重试 */
                    {
                        g_kick = 0;
                        g_spd_ramp = g_speed_ref;
                        if (++g_kick_fail >= KICK_FAIL_MAX)
                        {
                            g_kick_fail = KICK_FAIL_MAX;
                            g_iq_ref = -50.0f;
                        }
                    }
                    else if (fabsf(g_speed_ref) < 0.5f)
                    {
                        g_kick = 0;
                        g_kick_fail = 0;   /* 旋钮回零 */
                        g_spd_ramp = g_speed_ref;
                    }
                }
                speed_updated = 0;
            }
            else
            {
                /* 失败冷却：3 次脱困失败 → 静置 2 秒自动清零重试（无需旋钮回零） */
                if (g_kick_fail >= KICK_FAIL_MAX)
                {
                    if (fabsf(g_speed_ref) < 0.5f)
                    {
                        g_kick_fail = 0;   /* 旋钮回零立即复位 */
                        g_iq_ref = 0.0f;
                    }
                    else if (++g_fail_cool_cnt >= FAIL_COOL_WINDOWS)
                    {
                        g_fail_cool_cnt = 0;
                        g_kick_fail = 0;   /* 冷却完成：自动重试 */
                        g_iq_ref = 0.0f;
                    }
                    speed_updated = 0;
                    continue;
                }
                /* 切环力矩 boost：150ms 延续拖动力矩，防切环瞬间力矩骤降（iqr -20=0.23A<齿槽）失速。
                 * 1.15A(-100) 曾致电磁噪声打飞编码器（spd 假值 +5430）→ 降 0.46A 温柔过渡；
                 * 但 0.46A 在齿槽谷底势阱位推不动（实测转子 1 rad/s 起步蠕动 0.17 rad 卡停）→
                 * 恢复 1.15A + 延长 150ms 冲出势阱。编码器增量钳位 ±800 已兜住打飞风险 */
                if (g_boost_cnt > 0)
                {
                    /* boost 用实时 el（编码器持续更新）：θ_i 定向在负载角大时(实测 1.59 rad)
                     * 同样方向错(正冲 +20.2)；冻结推不动；1.15A+实时 el 是唯一能转 4 圈的
                     * 组合(43e62b0)。噪声只在切环瞬间几 ms，之后编码器恢复干净 */
                    g_iq_ref = (g_speed_ref < 0.0f) ? -100.0f : 100.0f;
                    if (g_dir_flip) g_iq_ref = -g_iq_ref;   /* 磁极反：boost 力矩同步取反 */
                    g_boost_cnt--;
                    g_spd_prev = g_speed_meas;
                    g_spd_ramp = g_speed_ref;
                    speed_updated = 0;
                    continue;
                }
                /* 低速区（0.5<|sref|<LOW_SPD_MAX）：直接开环拖动起步——磁场斜坡加速，
                 * 恒定电流拖着转子平滑转，替代速度环（低速速度环会在齿槽间振荡） */
                if (fabsf(g_speed_ref) < LOW_SPD_MAX && fabsf(g_speed_ref) > STALL_SREF_MIN
                    && g_kick_fail < KICK_FAIL_MAX)
                {
                    g_kick = 1;
                    g_sweep_mode = 0;
                    g_sweep_angle = g_elec_angle;
                    g_sweep_start = g_elec_angle;
                    g_sweep_dir = (g_speed_ref < 0.0f) ? -1.0f : 1.0f;
                    g_sweep_iq = (g_speed_ref < 0.0f) ? -SWEEP_IREF : SWEEP_IREF;
                    g_sweep_spd = 0.0f;
                    speed_integral = 0.0f;
                    speed_updated = 0;
                    continue;
                }
                /* 磁极方向自检（防误翻）：反向且 |spd|>6 持续 20 窗(100ms)才判定磁极反，
                 * 否则中速齿槽爬行时 spd 瞬时反向抖动会误翻 π → 方向横跳"中速顺时针/高速逆时针" */
                if (g_speed_meas * g_speed_ref < -80.0f && fabsf(g_speed_meas) > 6.0f)
                {
                    if (++g_pole_chk_cnt >= 20)
                    {
                        g_pole_chk_cnt = 0;
                        g_mech += (sint32)ENCODER_RESOLUTION / MOTOR_POLE_PAIRS / 2;  /* 磁极反 → 翻 π */
                        speed_integral = 0.0f;
                        g_iq_ref = -50.0f;
                    }
                }
                else
                {
                    g_pole_chk_cnt = 0;
                }
                /* 超速失控兜底：|spd|>25 rad/s（目标 8 的 3 倍）持续 100ms → 停机翻 π 重启。
                 * 实测失控时反电动势>驱动电压，iqr 恒 +150 饱和也刹不住（稳定 1000 RPM），
                 * 只能翻 π + 回预定位重新 I/f 纠正磁极方向 */
                if (fabsf(g_speed_meas) > 2500.0f)
                {
                    if (++g_ovspd_cnt >= 20)
                    {
                        g_ovspd_cnt = 0;
                        g_foc_mode = 0;
                        g_prepos_cnt = 0;
                        g_id_ref = 0.0f;
                        g_iq_ref = 0.0f;
                        g_mech += (sint32)ENCODER_RESOLUTION / MOTOR_POLE_PAIRS / 2;
                        g_if_flip_cnt = 0;
                        g_if_sync_cnt = 0;
                        g_if_stall_cnt = 0;
                        g_spd_ramp = g_speed_ref;
                    }
                }
                else
                {
                    g_ovspd_cnt = 0;
                }
                /* 速度环目标斜坡：每 5ms 爬 0.15 rad/s（30 rad/s²），目标 8 需 0.27s。
                 * 启动时误差始终小 → 力矩温和 → 不过冲不反向振荡 */
                float32 ramp_delta = g_speed_ref - g_spd_ramp;
                if (ramp_delta > 0.15f) g_spd_ramp += 0.15f;
                else if (ramp_delta < -0.15f) g_spd_ramp -= 0.15f;
                else g_spd_ramp = g_speed_ref;
                float32 err_speed = g_spd_ramp - g_speed_meas;
                sint32 windup = (err_speed > 0.0f && g_iq_ref >= 149.0f) ||
                               (err_speed < 0.0f && g_iq_ref <= -149.0f);
                /* 斜坡未到位前积分清零：启动爬坡段 err 一路负会累积负积分偏置，
                 * 到达目标后 new_iq 仍为负 → 稳态速度被顶到 ~17 rad/s（目标 8）且排不掉 → 超速+卡顿 */
                if (fabsf(g_spd_ramp - g_speed_ref) < 0.05f && !windup) speed_integral += err_speed;
                else if (fabsf(g_spd_ramp - g_speed_ref) >= 0.05f) speed_integral = 0.0f;
                if (speed_integral > 800.0f) speed_integral = 800.0f;
                if (speed_integral < -800.0f) speed_integral = -800.0f;
                /* D 项（微分阻尼）：spd 快速变化时产生反向阻尼力矩，压摆动极限环。
                 * 实测无 D 项时摆动期 err 巨大 → 输出每次打满 ±30 限幅 → 满力矩来回摆
                 * （spd ±400 交替、vq 恒 -500 饱和，0.2~0.4s 周期），无收敛 → "左右晃幅度大" */
                float32 spd_delta = g_speed_meas - g_spd_prev;
                g_spd_prev = g_speed_meas;
                /* 方向自检：spd 与 sref 明显反向且转子在转 → 参考系/磁极反 180° →
                 * iq 负产生正力矩（实测 b8efcb6 正转 5.7 圈）。检测到即翻转全部力矩输出，
                 * 转子被拉回正确方向，稳态自动维持（翻转不解除） */
                if (g_speed_meas * g_speed_ref < -20.0f && fabsf(g_speed_meas) > 1.5f)
                    g_dir_flip = 1;   /* 阈值 2.0→1.5：切环正冲 +316 约 0.3s(实测)，早 1 窗
                                       * 翻转 → 正冲窗口更短，纠正更快 */
                float32 new_iq;
                /* 超速（同向超出目标 0.5）：纯比例强刹 + D 阻尼，防深负积分抵消刹车。
                 * 此前积分深负（-400，0.04×(-400)=-16）压过 Kp(+3) → 超速还加速 →
                 * 冲出-滑行-掉速-预充循环（"一卡一卡"，实测超速 iqr=-9~-12 不刹车） */
                if ((g_spd_ramp < 0.0f && g_speed_meas < g_spd_ramp - 0.5f) ||
                    (g_spd_ramp > 0.0f && g_speed_meas > g_spd_ramp + 0.5f))
                {
                    new_iq = 1.5f * err_speed - 0.3f * spd_delta;
                }
                else
                {
                    new_iq = 1.0f * err_speed + 0.08f * speed_integral - 0.3f * spd_delta
                           + g_speed_ref * 6.0f;   /* 稳态前馈维持力矩：云台齿槽力矩随位置
                                                    * 大幅变化(0.3~1A+ 等效)——sref×2.75(-22
                                                    * 码 0.25A)只在弱齿槽位维持住，硬位掉速停
                                                    * (实测 -16.4k/-1390k/-140k 摆)。sref=-8
                                                    * → -48 码 0.55A 覆盖中弱齿槽；超速分支
                                                    * 保持纯 P 刹，前馈不干扰刹车。
                                                    * Kp 2.0→1.0：方向翻转后 spd -800~-1600
                                                    * 振荡(实测)系增益过高过冲 */
                }

                /* 力矩预充：spd 跌到低速且有给定 → 40ms 后满力矩冲齿槽。
                 * 速度环 Kp 项在静止起步时只有 ~0.07A，积分爬满需 >1s，齿槽等不及；
                 * 预充让起步/偶发堵转瞬间就有大电流，冲出后速度环立即接管。
                 * -100→-150(1.15A→1.7A)：实测切环超强位(-6024)齿槽>1.15A——
                 * boost 1.15A 2s 推不动、直推 1.7A 磨 2s 只到势垒肩部、最后预充
                 * 1.15A 顺势冲出(总 4s)。1.7A 预充=松动+冲出一步到位，切环超强位
                 * 4s→~1s。历史"-150 冲出过猛摆动"属无翻转时代，翻转兜底后安全。
                 * 阈值 0.5→2.0：掉速初期即冲(实测 -156k 位 spd -752→-153 掉 0.5s
                 * iqr -56~-80 不够)，等跌到 0.5 已全停→"抖一抖"；spd<2 即冲→0.1s */
                if (fabsf(g_speed_meas) < 2.0f && fabsf(g_speed_ref) >= LOW_SPD_MAX
                    && g_kick_fail < KICK_FAIL_MAX)
                {
                    if (++g_torq_pump_cnt >= 8)
                    {
                        speed_integral = 0.0f;   /* 冲出前清积分：冲出后超速分支纯比例刹，回落积分从 0 重建，
                                                  * 不残留掉速段的负偏置（否则冲出后 0.04×(-偏置) 抵消刹车） */
                        new_iq = (g_speed_ref < 0.0f) ? -150.0f : 150.0f;
                    }
                }
                else
                {
                    g_torq_pump_cnt = 0;
                }

                if (g_dir_flip) new_iq = -new_iq;   /* 磁极反：速度环/预充输出统一取反 →
                                                     * 力矩反向 → 转子拉回正确方向 */
                float32 dq = new_iq - g_iq_ref;
                if (dq > 30.0f) dq = 30.0f;
                if (dq < -30.0f) dq = -30.0f;
                g_iq_ref += dq;
                if (g_iq_ref > 150.0f) g_iq_ref = 150.0f;
                if (g_iq_ref < -150.0f) g_iq_ref = -150.0f;

                /* 堵转检测（仅中高速区 |sref|≥LOW_SPD_MAX；低速区已直接开环拖动）：
                 * spd≈0 且有给定 → 持续 200ms 触发慢速扫角脱困 */
                if (fabsf(g_speed_meas) < STALL_SPD_LIM && fabsf(g_speed_ref) >= LOW_SPD_MAX
                    && g_kick_fail < KICK_FAIL_MAX)
                {
                    if (++g_stall_cnt >= STALL_WINDOWS)
                    {
                        g_stall_cnt = 0;
                        g_kick = 1;
                        g_sweep_mode = 1;
                        g_sweep_angle = g_elec_angle;
                        g_sweep_start = g_elec_angle;
                        g_sweep_dir = (g_speed_ref < 0.0f) ? -1.0f : 1.0f;
                        g_sweep_iq = (g_speed_ref < 0.0f) ? -SWEEP_IREF : SWEEP_IREF;
                        g_sweep_spd = SWEEP_SPD;
                        g_kick_timer = 0;
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
        if (print_cnt >= 2000)   /* 每 2000 次主循环 ≈100ms/行，降串口负载防助手崩溃（原 500≈25ms/行） */
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
            FOC_UART_Print(" iqr="); FOC_UART_PrintInt((sint32)(g_kick ? g_sweep_iq : g_iq_ref));
            FOC_UART_Print(" sref="); FOC_UART_PrintInt((sint32)(g_speed_ref * 100.0f));
            FOC_UART_Print(" pot="); FOC_UART_PrintInt((sint32)g_pot_raw);
            FOC_UART_Print(" k="); FOC_UART_PrintInt((sint32)g_kick);
            FOC_UART_Print("\r\n");
        }
    }
}
