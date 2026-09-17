#ifndef DRV8305_SPI_H
#define DRV8305_SPI_H

#include "Platform_Types.h"

/*
 * DRV8305 GPIO bit-bang SPI 驱动
 * 接线（根据 TC275V1.3 原理图）：
 *   SCLK  = P00.0  (推挽输出)
 *   SDI   = P00.1  (推挽输出, TC275->DRV8305 MOSI)
 *   nSCS  = P00.2  (推挽输出, 低有效)
 *   SDO   = P33.0  (浮空输入, DRV8305->TC275 MISO)
 *
 * DRV8305 SPI: 16-bit frame, MSB first
 *   bit15    = R/W (1=read, 0=write)
 *   bit14:11 = register address (4 bit)
 *   bit10:0  = data (11 bit)
 */

void   Drv8305Spi_Init(void);
uint16 Drv8305Spi_Transfer16(uint16 txData);
uint16 Drv8305_ReadReg(uint8 regAddr);
void   Drv8305_WriteReg(uint8 regAddr, uint16 data);

#endif
