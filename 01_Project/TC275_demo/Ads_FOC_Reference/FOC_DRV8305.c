#include "FOC_DRV8305.h"
#include "IfxPort.h"

#define PIN_SCLK &MODULE_P00, 0
#define PIN_SDI  &MODULE_P00, 1
#define PIN_SCS  &MODULE_P00, 2
#define PIN_SDO  &MODULE_P33, 0

#define PIN_LED1 &MODULE_P00, 5

static void delay_us(uint32 us)
{
    for (volatile uint32 i = 0; i < us * 200; i++);
}

void DRV8305_Init(void)
{
    IfxPort_setPinModeOutput(PIN_SCLK, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(PIN_SDI, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(PIN_SCS, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeInput(PIN_SDO, IfxPort_InputMode_pullUp);

    IfxPort_setPinLow(PIN_SCLK);    /* Mode 0: SCLK空闲低 */
    IfxPort_setPinLow(PIN_SDI);
    IfxPort_setPinHigh(PIN_SCS);

    IfxPort_setPinModeOutput(PIN_LED1, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinHigh(PIN_LED1);
}

static uint16 spi_transfer(uint16 tx)
{
    uint16 rx = 0;

    IfxPort_setPinLow(PIN_SCS);
    delay_us(10);

    for (int i = 15; i >= 0; i--)
    {
        IfxPort_setPinLow(PIN_SCLK);
        delay_us(5);

        if (tx & (1 << i))
            IfxPort_setPinHigh(PIN_SDI);
        else
            IfxPort_setPinLow(PIN_SDI);

        delay_us(5);
        IfxPort_setPinHigh(PIN_SCLK);
        delay_us(5);

        IfxPort_setPinLow(PIN_SCLK);  /* Mode 1: 下降沿读MISO */
        if (IfxPort_getPinState(PIN_SDO))
            rx |= (1 << i);

        delay_us(5);
    }

    delay_us(10);
    IfxPort_setPinHigh(PIN_SCS);

    return rx;
}

void DRV8305_WriteReg(uint8 addr, uint16 data)
{
    uint16 frame = (0 << 15) | ((addr & 0xF) << 11) | (data & 0x7FF);
    spi_transfer(frame);
}

uint16 DRV8305_ReadReg(uint8 addr)
{
    uint16 cmd = (1 << 15) | ((addr & 0xF) << 11) | 0x000;
    spi_transfer(cmd);                       /* 第一帧: 发读命令 */
    return spi_transfer(0x000) & 0x7FF;      /* 第二帧: dummy, 读回数据 */
}

void DRV8305_TestSPI(void)
{
    DRV8305_WriteReg(0x07, 0x090);
    delay_us(1000);
    uint16 val = DRV8305_ReadReg(0x07);

    if (val == 0x090)
    {
        IfxPort_setPinLow(PIN_LED1);  /* 成功: LED常亮 */
    }
    else
    {
        for (int i = 0; i < 5; i++)   /* 失败: 闪5次 */
        {
            IfxPort_setPinLow(PIN_LED1);
            for (volatile int j = 0; j < 5000000; j++);
            IfxPort_setPinHigh(PIN_LED1);
            for (volatile int j = 0; j < 5000000; j++);
        }
    }
}
