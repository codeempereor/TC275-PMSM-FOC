#include "uart_test.h"
#include "IfxAsclin_reg.h"
#include "IfxPort_reg.h"

void UartTest_Init(void)
{
    uint32 iocr;

    ASCLIN0_CLC.U = 0x00000000u;

    ASCLIN0_FRAMECON.U = 0x00000000u;

    ASCLIN0_DATCON.U = 0x00000007u;

    ASCLIN0_BITCON.U = (54u << 0) | (15u << 16);

    ASCLIN0_BRG.U = 0x00000000u;

    ASCLIN0_TXFIFOCON.U = (1u << 0);
    ASCLIN0_RXFIFOCON.U = (1u << 0);

    ASCLIN0_IOCR.U = (1u << 31);

    ASCLIN0_FLAGSCLEAR.U = 0xFFFFFFFFu;

    iocr = P14_IOCR0.U;
    iocr &= ~(0xFu << 0);
    iocr |= (0x9u << 0);
    iocr &= ~(0xFu << 4);
    iocr |= (0x2u << 4);
    P14_IOCR0.U = iocr;
}

void UartTest_SendByte(uint8 data)
{
    while ((ASCLIN0_TXFIFOCON.U >> 16) & 0x1Fu);
    ASCLIN0_TXDATA.U = data;
}

void UartTest_SendString(const char* str)
{
    while (*str)
    {
        UartTest_SendByte((uint8)*str++);
    }
}

void UartTest_SendUint32(uint32 val)
{
    char buf[11];
    sint32 i = 0;
    uint32 tmp = val;

    if (val == 0u)
    {
        UartTest_SendByte('0');
        return;
    }

    while (tmp > 0u)
    {
        buf[i++] = (char)('0' + (tmp % 10u));
        tmp /= 10u;
    }

    while (i > 0)
    {
        UartTest_SendByte((uint8)buf[--i]);
    }
}
