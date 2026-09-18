#ifndef FOC_ALGORITHM_H
#define FOC_ALGORITHM_H

#include "Ifx_Types.h"
#include "FOC_Config.h"

typedef struct {
    float32 kp;
    float32 ki;
    float32 kd;
    float32 integral;
    float32 prevError;
    float32 outMax;
} PID_t;

void PID_Init(PID_t *pid, float32 kp, float32 ki, float32 kd, float32 outMax);
float32 PID_Calc(PID_t *pid, float32 target, float32 current);

void FOC_Clarke(float32 ia, float32 ib, float32 ic, float32 *ialpha, float32 *ibeta);
void FOC_Park(float32 ialpha, float32 ibeta, float32 angle, float32 *id, float32 *iq);
void FOC_InvPark(float32 vd, float32 vq, float32 angle, float32 *valpha, float32 *vbeta);
void FOC_SVPWM(float32 valpha, float32 vbeta, float32 *dutyA, float32 *dutyB, float32 *dutyC);

#endif
