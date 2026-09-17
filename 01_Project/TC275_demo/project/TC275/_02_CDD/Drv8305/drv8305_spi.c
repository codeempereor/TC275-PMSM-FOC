#include "drv8305_spi.h"
#include "IfxPort_reg.h"
#include "Mcal_WdgLib.h"

/*
 * GPIO bit-bang SPI for DRV8305
 *
 * 接线:
 *   P00.0 = SCLK  (推挽输出)
 *   P00.1 = MOSI  (推挽输出)
 *   P00.2 = nSCS  (推挽输出, 低有效)
 *   P33.0 = MISO  (浮空输入)
 *
 * DRV8305 SPI Mode: CPOL=0, CPHA=0 (Mode 0)
 *   - SCLK idle = LOW
 *   - MOSI set before rising edge
 *   - MISO read while SCLK is HIGH (after rising edge)
 *   - 16-bit frame, MSB first
 */

/* ---- 引脚操作宏 ---- */
#define SCLK_HIGH()   (MODULE_P00.OMR.U = (1u << 0))
#define SCLK_LOW()    (MODULE_P00.OMR.U = (1u << (16u + 0u)))
#define MOSI_HIGH()   (MODULE_P00.OMR.U = (1u << 1))
#define MOSI_LOW()    (MODULE_P00.OMR.U = (1u << (16u + 1u)))
#define NCS_HIGH()    (MODULE_P00.OMR.U = (1u << 2))
#define NCS_LOW()     (MODULE_P00.OMR.U = (1u << (16u + 2u)))
#define MISO_READ()   (((MODULE_P33.IN.U >> 0) & 1u))

/* 短延迟: 约 50-100ns (200MHz 主频) */
static inline void spi_udelay(void)
{
    volatile uint32 i;
    for(i = 0; i < 10u; i++) { }
}

void Drv8305Spi_Init(void)
{
    Mcal_ResetENDINIT();

    /* P00_IOCR0: CR0=P00.0 [3:0], CR1=P00.1 [7:4]
     * P00.0 SCLK  -> 0x80 推挽输出
     * P00.1 MOSI  -> 0x80 推挽输出 */
    P00_IOCR0.U = (P00_IOCR0.U & ~(0xFFu << 0)) | (0x80u << 0) | (0x80u << 4);

    /* P00_IOCR4: CR0=P00.2 [3:0]
     * P00.2 nSCS  -> 0x80 推挽输出 */
    P00_IOCR4.U = (P00_IOCR4.U & ~(0xFu << 0)) | (0x80u << 0);

    /* P33_IOCR0: CR0=P33.0 [3:0]
     * P33.0 MISO  -> 0x00 输入无上拉 */
    P33_IOCR0.U = (P33_IOCR0.U & ~(0xFu << 0)) | (0x00u << 0);

    Mcal_SetENDINIT();

    /* 空闲状态: SCLK=LOW(Mode0), MOSI=LOW, nSCS=HIGH */
    SCLK_LOW();
    MOSI_LOW();
    NCS_HIGH();
}

uint16 Drv8305Spi_Transfer16(uint16 txData)
{
    uint16 rxData = 0u;
    int i;

    /* 拉低片选 */
    NCS_LOW();
    spi_udelay();

    /* 逐位移位, MSB first
     * Mode 0: SCLK idle=LOW
     *   1. SCLK=LOW, 准备 MOSI
     *   2. SCLK 上升沿 -> 从机锁存 MOSI
     *   3. SCLK=HIGH, 从机驱动 MISO, 主机采样 */
    for(i = 15; i >= 0; i--)
    {
        /* 1. SCLK 拉低 (确保从低开始) */
        SCLK_LOW();
        spi_udelay();

        /* 2. 设置 MOSI 数据位 */
        if(txData & (1u << i))
        {
            MOSI_HIGH();
        }
        else
        {
            MOSI_LOW();
        }
        spi_udelay();

        /* 3. SCLK 上升沿: 从机锁存 MOSI, 从机输出 MISO */
        SCLK_HIGH();
        spi_udelay();

        /* 4. 在 SCLK 高电平期间采样 MISO */
        rxData <<= 1;
        if(MISO_READ())
        {
            rxData |= 1u;
        }
    }

    /* SCLK 拉低到空闲状态 */
    SCLK_LOW();
    spi_udelay();

    /* 释放片选 */
    NCS_HIGH();
    spi_udelay();

    return rxData;
}

/*
 * 读寄存器: bit15=1, bit14:11=addr, bit10:0=don't care
 * 返回: 16-bit, bit10:0 为寄存器数据
 */
uint16 Drv8305_ReadReg(uint8 regAddr)
{
    uint16 txFrame = (1u << 15) | ((uint16)(regAddr & 0xFu) << 11);
    return Drv8305Spi_Transfer16(txFrame);
}

/*
 * 写寄存器: bit15=0, bit14:11=addr, bit10:0=data
 */
void Drv8305_WriteReg(uint8 regAddr, uint16 data)
{
    uint16 txFrame = ((uint16)(regAddr & 0xFu) << 11) | (data & 0x7FFu);
    (void)Drv8305Spi_Transfer16(txFrame);
}
