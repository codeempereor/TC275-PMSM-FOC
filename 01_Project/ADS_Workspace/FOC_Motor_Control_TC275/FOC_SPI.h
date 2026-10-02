#ifndef FOC_SPI_H
#define FOC_SPI_H

#include "Ifx_Types.h"
#include "FOC_Config.h"

extern volatile float32 g_elec_angle;
extern volatile uint8   g_running;
extern uint16           g_zero_offset;

void     FOC_SPI_Init(void);
uint16   FOC_SPI_ReadRegisterRaw(uint16 reg);
uint16   FOC_SPI_GetAngleRaw(void);
float32  FOC_SPI_GetAngle(void);
void     FOC_SPI_DumpDebug(void);

#endif
