#include "FOC_SPI.h"
#include "IfxQspi_SpiMaster.h"
#include "IfxQspi_PinMap.h"
#include "Bsp.h"

static IfxQspi_SpiMaster         s_spiMaster;
static IfxQspi_SpiMaster_Channel s_spiChannel;

#define SPI_BUFFER_SIZE 4

static uint16 s_txBuffer[SPI_BUFFER_SIZE];
static uint16 s_rxBuffer[SPI_BUFFER_SIZE];

static const IfxQspi_SpiMaster_Pins s_spiPins = {
    &IfxQspi1_SCLK_P10_2_OUT,
    IfxPort_OutputMode_pushPull,
    &IfxQspi1_MTSR_P10_3_OUT,
    IfxPort_OutputMode_pushPull,
    &IfxQspi1_MRSTA_P10_1_IN,
    IfxPort_InputMode_pullDown,
    IfxPort_PadDriver_cmosAutomotiveSpeed3
};

void FOC_SPI_Init(void)
{
    IfxQspi_SpiMaster_Config spiCfg;
    IfxQspi_SpiMaster_initModuleConfig(&spiCfg, &MODULE_QSPI1);
    spiCfg.pins = &s_spiPins;
    spiCfg.base.maximumBaudrate = 10000000;
    IfxQspi_SpiMaster_initModule(&s_spiMaster, &spiCfg);

    IfxQspi_SpiMaster_ChannelConfig chCfg;
    IfxQspi_SpiMaster_initChannelConfig(&chCfg, &s_spiMaster);
    chCfg.base.baudrate = 5000000;
    chCfg.sls.output.pin = &IfxQspi1_SLSO10_P10_0_OUT;
    chCfg.sls.output.mode = IfxPort_OutputMode_pushPull;
    chCfg.sls.output.driver = IfxPort_PadDriver_cmosAutomotiveSpeed3;
    chCfg.mode = IfxQspi_SpiMaster_Mode_long;
    IfxQspi_SpiMaster_initChannel(&s_spiChannel, &chCfg);
}

static uint16 spiTransmit16(uint16 txData)
{
    s_txBuffer[0] = txData;
    s_rxBuffer[0] = 0;
    IfxQspi_SpiMaster_exchange(&s_spiChannel, &s_txBuffer[0], &s_rxBuffer[0], 1);
    while (IfxQspi_SpiMaster_getStatus(&s_spiChannel) == SpiIf_Status_busy) {}
    return s_rxBuffer[0];
}

uint16 FOC_SPI_ReadRegister(uint16 reg)
{
    uint16 cmd = AS5047P_CMD_READ | (reg & 0x3FFF);
    spiTransmit16(cmd);
    return spiTransmit16(0x0000);
}

uint16 FOC_SPI_GetAngleRaw(void)
{
    return FOC_SPI_ReadRegister(AS5047P_REG_ANGLECOM) & 0x3FFF;
}

float32 FOC_SPI_GetAngle(void)
{
    uint16 raw = FOC_SPI_GetAngleRaw();
    return (float32)raw * TWO_PI / ENCODER_RESOLUTION;
}
