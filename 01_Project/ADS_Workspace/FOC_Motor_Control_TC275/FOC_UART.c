#include "FOC_UART.h"
#include "IfxAscl_Asc.h"
#include "IfxAscl_reg.h"

static IfxAscl_Asc_Channel s_asc;

void FOC_UART_Init(void)
{
    IfxAscl_Asc_Config cfg;
    IfxAscl_Asc_initModuleConfig(&cfg, &MODULE_ASCLIN0);

    cfg.baudrate = 115200;
    cfg.txPin = &IfxAscl_Tx_P14_0;
    cfg.rxPin = &IfxAscl_Rx_P14_1;
    cfg.rxPinMode = IfxPort_InputMode_pullUp;
    cfg.txPinMode = IfxPort_OutputMode_pushPull;
    cfg.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;

    IfxAscl_Asc_initModule(&s_asc, &cfg);
}

void FOC_UART_SendChar(char c)
{
    IfxAscl_Asc_send(&s_asc, (uint8)c);
}

void FOC_UART_Print(const char *str)
{
    while (*str)
    {
        if (*str == '\n')
            IfxAscl_Asc_send(&s_asc, (uint8)'\r');
        IfxAscl_Asc_send(&s_asc, (uint8)(*str++));
    }
}

void FOC_UART_PrintInt(int32 val)
{
    char buf[12];
    int i = 0;
    int negative = 0;

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

void FOC_UART_PrintFloat(float32 val, int decimals)
{
    int32 intPart;
    int32 fracPart;
    int i;

    if (val < 0) {
        FOC_UART_SendChar('-');
        val = -val;
    }

    intPart = (int32)val;
    fracPart = (int32)((val - (float32)intPart) * 10000);

    FOC_UART_PrintInt(intPart);
    FOC_UART_SendChar('.');

    for (i = 0; i < decimals; i++) {
        int divisor = 1000;
        int digit;
        if (i >= 4) break;
        digit = (fracPart / divisor) % 10;
        FOC_UART_SendChar('0' + digit);
        divisor /= 10;
    }
}
