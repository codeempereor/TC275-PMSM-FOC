#ifndef FOC_MATH_H
#define FOC_MATH_H

#include "foc_config.h"

typedef struct {
    float alpha;
    float beta;
} FOC_AB_t;

typedef struct {
    float d;
    float q;
} FOC_DQ_t;

typedef struct {
    float a;
    float b;
    float c;
} FOC_ABC_t;

FOC_AB_t FOC_Clarke(FOC_ABC_t *abc);
FOC_DQ_t FOC_Park(FOC_AB_t *ab, float sin_theta, float cos_theta);
FOC_AB_t FOC_InvPark(FOC_DQ_t *dq, float sin_theta, float cos_theta);
float FOC_WrapAngle(float angle);
void FOC_SinCos(float angle, float *sin_val, float *cos_val);

#endif
