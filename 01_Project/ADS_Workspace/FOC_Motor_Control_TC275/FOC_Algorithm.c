#include "FOC_Algorithm.h"
#include <math.h>

#define SQRT3  1.7320508075688772f
#define INV_SQRT3 0.5773502691896258f

void PID_Init(PID_t *pid, float32 kp, float32 ki, float32 kd, float32 outMax)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->outMax = outMax;
}

float32 PID_Calc(PID_t *pid, float32 target, float32 current)
{
    float32 error = target - current;
    float32 derivative = error - pid->prevError;
    pid->integral += error;
    float32 intLimit = (pid->ki > 0.0f) ? (pid->outMax / pid->ki) : 1e9f;
    if (pid->integral > intLimit) pid->integral = intLimit;
    if (pid->integral < -intLimit) pid->integral = -intLimit;
    pid->prevError = error;
    float32 output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    if (output > pid->outMax) output = pid->outMax;
    if (output < -pid->outMax) output = -pid->outMax;
    return output;
}

void FOC_Clarke(float32 ia, float32 ib, float32 ic, float32 *ialpha, float32 *ibeta)
{
    *ialpha = ia;
    *ibeta = (ia + 2.0f * ib) * INV_SQRT3;
}

void FOC_Park(float32 ialpha, float32 ibeta, float32 angle, float32 *id, float32 *iq)
{
    float32 cosA = cosf(angle);
    float32 sinA = sinf(angle);
    *id = ialpha * cosA + ibeta * sinA;
    *iq = -ialpha * sinA + ibeta * cosA;
}

void FOC_InvPark(float32 vd, float32 vq, float32 angle, float32 *valpha, float32 *vbeta)
{
    float32 cosA = cosf(angle);
    float32 sinA = sinf(angle);
    *valpha = vd * cosA - vq * sinA;
    *vbeta = vd * sinA + vq * cosA;
}

void FOC_SVPWM(float32 valpha, float32 vbeta, float32 *dutyA, float32 *dutyB, float32 *dutyC)
{
    float32 va = valpha;
    float32 vb = -0.5f * valpha + 0.8660254f * vbeta;
    float32 vc = -0.5f * valpha - 0.8660254f * vbeta;

    float32 vmax = va;
    if (vb > vmax) vmax = vb;
    if (vc > vmax) vmax = vc;
    float32 vmin = va;
    if (vb < vmin) vmin = vb;
    if (vc < vmin) vmin = vc;

    float32 vzero = (vmax + vmin) * 0.5f;
    va -= vzero;
    vb -= vzero;
    vc -= vzero;

    *dutyA = va * 0.5f + 0.5f;
    *dutyB = vb * 0.5f + 0.5f;
    *dutyC = vc * 0.5f + 0.5f;
}
