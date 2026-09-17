#ifndef FOC_ADC_H
#define FOC_ADC_H

#include "Ifx_Types.h"
#include "FOC_Config.h"

void FOC_ADC_Init(void);
void FOC_ADC_ReadAll(float32 *ia, float32 *ib, float32 *ic);
uint16 FOC_ADC_ReadRaw(uint8 ch);

#endif
