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

    chCfg.channelId = IfxVadc_ChannelId_7;
    chCfg.resultRegister = IfxVadc_ChannelResult_7;
    IfxVadc_Adc_initChannel(&s_chA, &chCfg);

    chCfg.channelId = IfxVadc_ChannelId_6;
    chCfg.resultRegister = IfxVadc_ChannelResult_6;
    IfxVadc_Adc_initChannel(&s_chB, &chCfg);

    chCfg.channelId = IfxVadc_ChannelId_5;
    chCfg.resultRegister = IfxVadc_ChannelResult_5;
    IfxVadc_Adc_initChannel(&s_chC, &chCfg);

    IfxVadc_Adc_startScan(&s_group4);
}

uint16 FOC_ADC_ReadRaw(uint8 ch)
{
    IfxVadc_Adc_Channel *channel;
    switch (ch) {
        case 7: channel = &s_chA; break;
        case 6: channel = &s_chB; break;
        case 5: channel = &s_chC; break;
        default: return 0;
    }
    Ifx_VADC_RES result = IfxVadc_Adc_getResult(channel);
    return result.B.RESULT;
}

void FOC_ADC_ReadAll(float32 *ia, float32 *ib, float32 *ic)
{
    uint16 rawA = FOC_ADC_ReadRaw(7);
    uint16 rawB = FOC_ADC_ReadRaw(6);
    uint16 rawC = FOC_ADC_ReadRaw(5);

    *ia = ((float32)rawA - 2048.0f) * ADC_CURRENT_SCALE;
    *ib = ((float32)rawB - 2048.0f) * ADC_CURRENT_SCALE;
    *ic = ((float32)rawC - 2048.0f) * ADC_CURRENT_SCALE;
}
