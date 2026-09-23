#include "FOC_UART.h"
#include "IfxAsclin_Asc.h"
#include "IfxCpu_Irq.h"

static IfxAsclin_Asc s_asc;
/* txBuffer must be at least txBufferSize + sizeof(Ifx_Fifo) + 8 */
#define ASC_TX_BUFFER_SIZE 64
static uint8 s_txBuffer[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];

#define INTPRIO_ASCLIN0_TX 19

IFX_INTERRUPT(asclin0_Tx_ISR, 0, INTPRIO_ASCLIN0_TX);
void asclin0_Tx_ISR(void)
{
    IfxAsclin_Asc_isrTransmit(&s_asc);
}

void FOC_UART_Init(void)
{
    IfxAsclin_Asc_Config cfg;
    IfxAsclin_Asc_initModuleConfig(&cfg, &MODULE_ASCLIN0);

    cfg.baudrate.baudrate = 115200.0f;

    cfg.interrupt.txPriority = INTPRIO_ASCLIN0_TX;
    cfg.interrupt.typeOfService = IfxCpu_Irq_getTos(IfxCpu_getCoreIndex());

    cfg.txBufferSize = ASC_TX_BUFFER_SIZE;
    cfg.txBuffer = s_txBuffer;

    const IfxAsclin_Asc_Pins pins =
    {
        NULL_PTR,                        IfxPort_InputMode_pullUp,     /* CTS pin not used     */
        &IfxAsclin0_RXA_P14_1_IN,        IfxPort_InputMode_pullUp,     /* RX pin               */
        NULL_PTR,                        IfxPort_OutputMode_pushPull,  /* RTS pin not used     */
        &IfxAsclin0_TX_P14_0_OUT,        IfxPort_OutputMode_pushPull,  /* TX pin               */
        IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    cfg.pins = &pins;

    IfxAsclin_Asc_initModule(&s_asc, &cfg);
}

void FOC_UART_SendChar(char c)
{
    IfxAsclin_Asc_blockingWrite(&s_asc, (uint8)c);
}

void FOC_UART_Print(const char *str)
{
    while (*str)
    {
        if (*str == '\n')
            FOC_UART_SendChar('\r');
        FOC_UART_SendChar(*str++);
    }
}

void FOC_UART_PrintInt(sint32 val)
{
    char buf[12];
    sint32 i = 0;
    sint32 negative = 0;

    if (val < 0) {
        negative = 1;
        val = -val;
    }

    if (val == 0) {
        FOC_UART_Print("0");
        return;
    }

    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }

    if (negative)
        FOC_UART_SendChar('-');

    while (i > 0)
        FOC_UART_SendChar(buf[--i]);
}

void FOC_UART_PrintFloat(float32 val, sint32 decimals)
{
    sint32 intPart;
    sint32 fracPart;
    sint32 i;

    if (val < 0) {
        FOC_UART_SendChar('-');
        val = -val;
    }

    intPart = (sint32)val;
    fracPart = (sint32)((val - (float32)intPart) * 10000.0f);

    FOC_UART_PrintInt(intPart);
    FOC_UART_SendChar('.');

    for (i = 0; i < decimals; i++) {
        sint32 divisor = 1000;
        sint32 digit;
        if (i >= 4) break;
        digit = (fracPart / divisor) % 10;
        FOC_UART_SendChar('0' + digit);
        divisor /= 10;
    }
}