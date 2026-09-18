#ifndef FOC_SPI_H
#define FOC_SPI_H

#include "Ifx_Types.h"
#include "FOC_Config.h"

void FOC_SPI_Init(void);
uint16 FOC_SPI_ReadRegister(uint16 reg);
float32 FOC_SPI_GetAngle(void);
uint16 FOC_SPI_GetAngleRaw(void);

#endif
