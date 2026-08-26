#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#include "foc_config.h"
#include "foc_math.h"
#include <stdint.h>

typedef struct {
    float duty_a;
    float duty_b;
    float duty_c;
    uint8_t sector;
} FOC_SVPWM_Output_t;

void FOC_SVPWM_Init(void);
FOC_SVPWM_Output_t FOC_SVPWM_Run(FOC_AB_t *v_ab);

#endif
