#include "FOC_ADC.h"
#include "IfxVadc_Adc.h"
#include "IfxVadc_PinMap.h"
#include "IfxPort.h"

static IfxVadc_Adc         s_vadc;
static IfxVadc_Adc_Group   s_group0;   /* 电位器旋钮组 (AN0) */
static IfxVadc_Adc_Group   s_group4;   /* 三相电流采样组 */
static IfxVadc_Adc_Channel s_chA;
static IfxVadc_Adc_Channel s_chB;
static IfxVadc_Adc_Channel s_chC;
static IfxVadc_Adc_Channel s_chPot;  /* potentiometer (knob) channel */

void FOC_ADC_Init(void)
{
    /* 电流采样引脚配置为模拟输入（无上下拉）：P40.7/8/9 = AN37/38/39 = Group4 Ch5/6/7 */
    IfxPort_setPinModeInput(&MODULE_P40, 7, IfxPort_InputMode_noPullDevice);  /* ISEN_C = P40.7 */
    IfxPort_setPinModeInput(&MODULE_P40, 8, IfxPort_InputMode_noPullDevice);  /* ISEN_B = P40.8 */
    IfxPort_setPinModeInput(&MODULE_P40, 9, IfxPort_InputMode_noPullDevice);  /* ISEN_A = P40.9 */
    /* 电位器 AN0 是纯模拟输入（数据手册引脚表无端口控制器，iLLD G0_0_AN0 = NULL_PTR），无需 GPIO 配置 */

    IfxVadc_Adc_Config modCfg;
    IfxVadc_Adc_initModuleConfig(&modCfg, &MODULE_VADC);
    IfxVadc_Adc_initModule(&s_vadc, &modCfg);

    /* --- Group0：电位器旋钮 AN0 = G0CH0，单次扫描 + 主循环按需软件触发 ---
     * 不用后台扫描/自动扫描：结果走 GLOBRES 或持续占用仲裁，都会干扰/复杂化；
     * 电位器是慢变量，主循环每 100 次才读一次，触发一次扫描仅 ~1us，对 Group4 电流采样无影响。 */
    IfxVadc_Adc_GroupConfig grpCfg0;
    IfxVadc_Adc_initGroupConfig(&grpCfg0, &s_vadc);
    grpCfg0.groupId = IfxVadc_GroupId_0;
    grpCfg0.master = IfxVadc_GroupId_0;
    grpCfg0.scanRequest.autoscanEnabled = FALSE;   /* 单次扫描 */
    grpCfg0.scanRequest.triggerConfig.gatingMode = IfxVadc_GatingMode_always;
    grpCfg0.arbiter.requestSlotScanEnabled = TRUE;
    IfxVadc_Adc_initGroup(&s_group0, &grpCfg0);

    IfxVadc_Adc_ChannelConfig chCfg0;
    IfxVadc_Adc_initChannelConfig(&chCfg0, &s_group0);
    chCfg0.channelId = IfxVadc_ChannelId_0;        /* AN0 = 板载电位器 */
    chCfg0.resultRegister = IfxVadc_ChannelResult_0;
    IfxVadc_Adc_initChannel(&s_chPot, &chCfg0);
    IfxVadc_Adc_setScan(&s_group0, (1 << 0), (1 << 0));
    IfxVadc_Adc_startScan(&s_group0);

    /* --- Group4：三相电流采样，PWM ISR 软件触发同步扫描 --- */
    IfxVadc_Adc_GroupConfig grpCfg;
    IfxVadc_Adc_initGroupConfig(&grpCfg, &s_vadc);
    grpCfg.groupId = IfxVadc_GroupId_4;
    grpCfg.master = IfxVadc_GroupId_4;
    grpCfg.scanRequest.autoscanEnabled = FALSE;   /* 单次扫描 + 软件触发：PWM ISR 起点同步采样 */
    grpCfg.scanRequest.triggerConfig.gatingMode = IfxVadc_GatingMode_always;
    grpCfg.arbiter.requestSlotScanEnabled = TRUE;
    IfxVadc_Adc_initGroup(&s_group4, &grpCfg);

    IfxVadc_Adc_ChannelConfig chCfg;
    IfxVadc_Adc_initChannelConfig(&chCfg, &s_group4);

    /* A相: P40.9 = AN39 = Group4 Ch7 */
    chCfg.channelId = IfxVadc_ChannelId_7;
    chCfg.resultRegister = IfxVadc_ChannelResult_7;
    IfxVadc_Adc_initChannel(&s_chA, &chCfg);

    /* B相: P40.8 = AN38 = Group4 Ch6 */
    chCfg.channelId = IfxVadc_ChannelId_6;
    chCfg.resultRegister = IfxVadc_ChannelResult_6;
    IfxVadc_Adc_initChannel(&s_chB, &chCfg);

    /* C相: P40.7 = AN37 = Group4 Ch5 */
    chCfg.channelId = IfxVadc_ChannelId_5;
    chCfg.resultRegister = IfxVadc_ChannelResult_5;
    IfxVadc_Adc_initChannel(&s_chC, &chCfg);

    /* 仅扫描电流通道 6/7（A/B 两相，C 相在主 ISR 重构 ic=-(ia+ib)）：
     * 去掉通道 5 使三相顺序转换变两相 → 转换时间减 1/3 → 高 duty 下
     * 低侧分流窗口（duty 0.80 → 4us）内容纳得下转换（2 通道 ~1.2-2us）。
     * 旧 mask 含 5/6/7：第三通道白白转换 0.6-1us → duty 0.45 窗口 13.75us
     * 无感，但 duty 放开后窗口收窄即采样出界（历史 0.9 失效根因）。 */
    /* 【诊断版临时】恢复三相扫描 mask（5/6/7）：
     * 10/4 改两相（6/7）后 A 相(ch7)采样点提前 ~1µs（少一个通道转换时间），
     * 若 A 相读数系统性偏小（9/30 三相时代 ra≈2600/1.3A，10/4 两相时代
     * ra≈2050/0.3A，同电压差 4 倍）→ 采样点错位实锤 → 恢复本行即可定位。
     * 验证后恢复两相需改回：(1 << 6) | (1 << 7) */
    uint32 channelMask = (1 << 5) | (1 << 6) | (1 << 7);
    IfxVadc_Adc_setScan(&s_group4, channelMask, channelMask);

    IfxVadc_Adc_startScan(&s_group4);
}

/* PWM ISR 起点调用：软件触发一次扫描，采样点固定在当前 PWM 周期中点 */
void FOC_ADC_StartSync(void)
{
    IfxVadc_Adc_startScan(&s_group4);
}

/* 主循环读电位器前调用：软件触发一次 Group0 扫描（~1us，不影响 Group4 仲裁） */
void FOC_ADC_TriggerPot(void)
{
    IfxVadc_Adc_startScan(&s_group0);
}

uint16 FOC_ADC_ReadRaw(uint8 ch)
{
    IfxVadc_Adc_Channel *channel;
    switch (ch) {
        case 0: channel = &s_chPot; break;
        case 7: channel = &s_chA; break;
        case 6: channel = &s_chB; break;
        case 5: channel = &s_chC; break;
        default: return 0;
    }
    Ifx_VADC_RES result;
    do { result = IfxVadc_Adc_getResult(channel); } while (!result.B.VF);
    return result.B.RESULT;
}

void FOC_ADC_ReadAll(float32 *ia, float32 *ib, float32 *ic)
{
    FOC_ADC_StartSync();
    uint16 rawA = FOC_ADC_ReadRaw(7);
    uint16 rawB = FOC_ADC_ReadRaw(6);

    *ia = ((float32)rawA - 2048.0f) * ADC_CURRENT_SCALE;
    *ib = ((float32)rawB - 2048.0f) * ADC_CURRENT_SCALE;
    *ic = -(*ia + *ib);   /* C 相重构（扫描 mask 已去通道 5，勿读 ReadRaw(5) → VF 死等） */
}
