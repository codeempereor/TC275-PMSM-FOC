#ifndef GTM_PWM_H
#define GTM_PWM_H

#include "Platform_Types.h"

void GtmPwm_Init(void);
void GtmPwm_SetDuty(uint32 duty);
void GtmPwm_SetFreq(uint32 freq);

#endif
