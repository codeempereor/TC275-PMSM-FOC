#ifndef FOC_DRV8305_SPI_H
#define FOC_DRV8305_SPI_H

#include "Ifx_Types.h"

void DRV8305_Init(void);
void DRV8305_WriteReg(uint8 addr, uint16 data);
uint16 DRV8305_ReadReg(uint8 addr);
void DRV8305_TestSPI(void);

#endif
