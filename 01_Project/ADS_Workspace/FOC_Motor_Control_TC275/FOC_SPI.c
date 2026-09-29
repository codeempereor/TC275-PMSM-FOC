#include "FOC_SPI.h"
#include "FOC_UART.h"
#include "IfxQspi_SpiMaster.h"
#include "IfxQspi_PinMap.h"
#include "Bsp.h"

/*
 * QSPI1 pins (AS5047P encoder):
 *   SCLK  = P10.2
 *   MOSI  = P10.3 (MTSR)
 *   MISO  = P10.1 (MRSTI)
 *   CSN   = P10.0 (SLSO10)
 */
static IfxQspi_SpiMaster         s_spiMaster;
static IfxQspi_SpiMaster_Channel s_spiChannel;

#define SPI_BUFFER_SIZE 4
static uint16 s_txBuffer[SPI_BUFFER_SIZE];
static uint16 s_rxBuffer[SPI_BUFFER_SIZE];

/* Interrupt priorities. PWM ISR already uses priority 1, so use 4/5/6 here. */
#define QSPI1_TX_PRIO  4
#define QSPI1_RX_PRIO  5
#define QSPI1_ER_PRIO  6

IFX_INTERRUPT(FOC_Qspi1TxISR, 0, QSPI1_TX_PRIO)
{
    IfxQspi_SpiMaster_isrTransmit(&s_spiMaster);
}

IFX_INTERRUPT(FOC_Qspi1RxISR, 0, QSPI1_RX_PRIO)
{
    IfxQspi_SpiMaster_isrReceive(&s_spiMaster);
}

IFX_INTERRUPT(FOC_Qspi1ErISR, 0, QSPI1_ER_PRIO)
{
    IfxQspi_SpiMaster_isrError(&s_spiMaster);
}

static const IfxQspi_SpiMaster_Pins s_spiPins = {
    &IfxQspi1_SCLK_P10_2_OUT,
    IfxPort_OutputMode_pushPull,
    &IfxQspi1_MTSR_P10_3_OUT,
    IfxPort_OutputMode_pushPull,
    &IfxQspi1_MRSTA_P10_1_IN,
    IfxPort_InputMode_pullUp,
    IfxPort_PadDriver_cmosAutomotiveSpeed3
};

/*
 * AS5047P SPI frame (16 bit):
 *   bit15 = PARD  even parity over bit14:0
 *   bit14 = RW    1=read, 0=write
 *   bit13:0 = address (command) / data (response)
 * Response frame:
 *   bit15 = PARD, bit14 = EF (command error), bit13:0 = data
 */
static uint16 as5047p_make_read_cmd(uint16 reg)
{
    uint16 cmd = (uint16)(0x4000u | (reg & 0x3FFFu));
    uint16 x = cmd & 0x7FFFu;
    x ^= (uint16)(x >> 8);
    x ^= (uint16)(x >> 4);
    x ^= (uint16)(x >> 2);
    x ^= (uint16)(x >> 1);
    cmd |= (uint16)((x & 0x0001u) << 15);
    return cmd;
}

void FOC_SPI_Init(void)
{
    IfxQspi_SpiMaster_Config spiCfg;
    IfxQspi_SpiMaster_initModuleConfig(&spiCfg, &MODULE_QSPI1);
    spiCfg.pins = &s_spiPins;
    spiCfg.base.maximumBaudrate = 10000000.0f;

    /* QSPI interrupts: required because IfxQspi_SpiMaster_exchange() is async.
     * Without these ISRs onTransfer never clears and getStatus() stays busy forever. */
    spiCfg.base.txPriority   = QSPI1_TX_PRIO;
    spiCfg.base.rxPriority   = QSPI1_RX_PRIO;
    spiCfg.base.erPriority   = QSPI1_ER_PRIO;
    spiCfg.base.isrProvider  = IfxSrc_Tos_cpu0;

    IfxQspi_SpiMaster_initModule(&s_spiMaster, &spiCfg);

    /* IFX_INTERRUPT macros auto-register ISRs to the correct vector slot;
     * no installInterruptHandler call needed (IfxCpu_Irq.c not in project). */

    IfxQspi_SpiMaster_ChannelConfig chCfg;
    IfxQspi_SpiMaster_initChannelConfig(&chCfg, &s_spiMaster);
    chCfg.base.baudrate = 5000000.0f;

    /* AS5047P: SPI Mode 1 = CPOL=0 (idle low), CPHA=1 (sample on trailing edge). */
    chCfg.base.mode.clockPolarity = SpiIf_ClockPolarity_idleLow;
    chCfg.base.mode.shiftClock    = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge;
    chCfg.base.mode.dataWidth     = 16;
    chCfg.base.mode.dataHeading   = SpiIf_DataHeading_msbFirst;
    chCfg.base.mode.csActiveLevel = Ifx_ActiveState_low;

    /* Keep default short mode (do NOT use Mode_long, needs packLongModeBuffer). */
    chCfg.sls.output.pin    = &IfxQspi1_SLSO10_P10_0_OUT;
    chCfg.sls.output.mode   = IfxPort_OutputMode_pushPull;
    chCfg.sls.output.driver = IfxPort_PadDriver_cmosAutomotiveSpeed3;

    IfxQspi_SpiMaster_initChannel(&s_spiChannel, &chCfg);
}

/* Blocking 16-bit full-duplex exchange.
 * exchange() starts the transfer and returns immediately; the RX ISR clears
 * onTransfer when data lands, so the busy-wait below then exits. */
static uint16 spiTransmit16(uint16 txData)
{
    s_txBuffer[0] = txData;
    s_rxBuffer[0] = 0;
    while (IfxQspi_SpiMaster_getStatus(&s_spiChannel) == SpiIf_Status_busy) { }
    IfxQspi_SpiMaster_exchange(&s_spiChannel, &s_txBuffer[0], &s_rxBuffer[0], 1);
    while (IfxQspi_SpiMaster_getStatus(&s_spiChannel) == SpiIf_Status_busy) { }
    return s_rxBuffer[0];
}

/*
 * AS5047P read is a 2-frame handshake:
 *   frame 1: send read command -> MISO returns previous frame's data (dummy)
 *   frame 2: send dummy 0x0000 -> MISO returns the requested register
 */
uint16 FOC_SPI_ReadRegisterRaw(uint16 reg)
{
    uint16 cmd = as5047p_make_read_cmd(reg);
    spiTransmit16(cmd);
    return spiTransmit16(0x0000u);
}

uint16 FOC_SPI_GetAngleRaw(void)
{
    return (uint16)(FOC_SPI_ReadRegisterRaw(AS5047P_REG_ANGLECOM) & 0x3FFFu);
}

float32 FOC_SPI_GetAngle(void)
{
    uint16 raw = FOC_SPI_GetAngleRaw();
    return (float32)raw * TWO_PI / (float32)ENCODER_RESOLUTION;
}

void FOC_SPI_DumpDebug(void)
{
    uint16 frame = FOC_SPI_ReadRegisterRaw(AS5047P_REG_ANGLECOM);
    uint16 ef     = (uint16)((frame >> 14) & 0x1u);
    uint16 pard   = (uint16)((frame >> 15) & 0x1u);
    uint16 angle  = (uint16)(frame & 0x3FFFu);

    FOC_UART_Print("ENC: frame="); FOC_UART_PrintInt((sint32)frame);
    FOC_UART_Print(" EF=");       FOC_UART_PrintInt((sint32)ef);
    FOC_UART_Print(" PARD=");     FOC_UART_PrintInt((sint32)pard);
    FOC_UART_Print(" angle=");    FOC_UART_PrintInt((sint32)angle);
    FOC_UART_Print("\r\n");
}
