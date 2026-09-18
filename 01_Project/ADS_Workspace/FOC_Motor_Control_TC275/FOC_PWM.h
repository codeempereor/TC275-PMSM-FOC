#ifndef FOC_PWM_H
#define FOC_PWM_H

#include "Ifx_Types.h"
#include "FOC_Config.h"

void FOC_PWM_Init(void);
void FOC_PWM_SetDuty(uint16 dutyA, uint16 dutyB, uint16 dutyC);
void FOC_PWM_SetDutyPercent(float32 dutyA, float32 dutyB, float32 dutyC); /* 0.0~1.0 */
void FOC_PWM_Enable(void);
void FOC_PWM_Disable(void);

#endif
