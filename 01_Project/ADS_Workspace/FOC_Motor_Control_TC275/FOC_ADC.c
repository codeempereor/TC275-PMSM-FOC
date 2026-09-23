#include "FOC_ADC.h"
#include "IfxVadc_Adc.h"
#include "IfxVadc_PinMap.h"

static IfxVadc_Adc         s_vadc;
static IfxVadc_Adc_Group   s_group4;
static IfxVadc_Adc_Channel s_chA;
static IfxVadc_Adc_Channel s_chB;
static IfxVadc_Adc_Channel s_chC;

void FOC_ADC_Init(void)
{
    IfxVadc_Adc_Config modCfg;
    IfxVadc_Adc_initModuleConfig(&modCfg, &MODULE_VADC);
    IfxVadc_Adc_initModule(&s_vadc, &modCfg);

    IfxVadc_Adc_GroupConfig grpCfg;
    IfxVadc_Adc_initGroupConfig(&grpCfg, &s_vadc);
    grpCfg.groupId = IfxVadc_GroupId_4;
    grpCfg.master = IfxVadc_GroupId_4;
    grpCfg.scanRequest.autoscanEnabled = TRUE;
    grpCfg.scanRequest.triggerConfig.gatingMode = IfxVadc_GatingMode_always;
    grpCfg.arbiter.requestSlotScanEnabled = TRUE;
    IfxVadc_Adc_initGroup(&s_group4, &grpCfg);

    IfxVadc_Adc_ChannelConfig chCfg;
    IfxVadc_Adc_initChannelConfig(&chCfg, &s_group4);

    /* A相: P40.9 = AN39 = Group4 Ch9 */
    chCfg.channelId = IfxVadc_ChannelId_9;
    chCfg.resultRegister = IfxVadc_ChannelResult_9;
    IfxVadc_Adc_initChannel(&s_chA, &chCfg);

    /* B相: P40.8 = AN38 = Group4 Ch8 */
    chCfg.channelId = IfxVadc_ChannelId_8;
    chCfg.resultRegister = IfxVadc_ChannelResult_8;
    IfxVadc_Adc_initChannel(&s_chB, &chCfg);

    /* C相: P40.7 = AN37 = Group4 Ch7 */
    chCfg.channelId = IfxVadc_ChannelId_7;
    chCfg.resultRegister = IfxVadc_ChannelResult_7;
    IfxVadc_Adc_initChannel(&s_chC, &chCfg);

    IfxVadc_Adc_startScan(&s_group4);
}

uint16 FOC_ADC_ReadRaw(uint8 ch)
{
    IfxVadc_Adc_Channel *channel;
    switch (ch) {
        case 9: channel = &s_chA; break;
        case 8: channel = &s_chB; break;
        case 7: channel = &s_chC; break;
        default: return 0;
    }
    Ifx_VADC_RES result = IfxVadc_Adc_getResult(channel);
    return result.B.RESULT;
}

void FOC_ADC_ReadAll(float32 *ia, float32 *ib, float32 *ic)
{
    uint16 rawA = FOC_ADC_ReadRaw(9);
    uint16 rawB = FOC_ADC_ReadRaw(8);
    uint16 rawC = FOC_ADC_ReadRaw(7);

    *ia = ((float32)rawA - 2048.0f) * ADC_CURRENT_SCALE;
    *ib = ((float32)rawB - 2048.0f) * ADC_CURRENT_SCALE;
    *ic = ((float32)rawC - 2048.0f) * ADC_CURRENT_SCALE;
}
