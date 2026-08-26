#ifndef FOC_PI_H
#define FOC_PI_H

#include "foc_config.h"

typedef struct {
    float kp;
    float ki;
    float integral;
    float output_max;
    float output_min;
    float prev_error;
} FOC_PI_t;

void FOC_PI_Init(FOC_PI_t *pi, float kp, float ki, float out_max, float out_min);
void FOC_PI_Reset(FOC_PI_t *pi);
float FOC_PI_Run(FOC_PI_t *pi, float error, float dt);

#endif
