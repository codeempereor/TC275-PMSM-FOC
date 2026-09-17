#ifndef FOC_PWM_H
#define FOC_PWM_H

#include "Ifx_Types.h"
#include "FOC_Config.h"

void FOC_PWM_Init(void);
void FOC_PWM_SetDuty(float32 dutyA, float32 dutyB, float32 dutyC);
void FOC_PWM_Enable(void);
void FOC_PWM_Disable(void);

#endif
