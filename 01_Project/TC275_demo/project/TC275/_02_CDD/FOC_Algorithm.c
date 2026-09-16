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
    if (pid->integral > pid->outMax) pid->integral = pid->outMax;
    if (pid->integral < -pid->outMax) pid->integral = -pid->outMax;
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
    float32 t1, t2, t0;
    float32 va, vb, vc;
    float32 x = vbeta;
    float32 y = (SQRT3 * valpha + vbeta) * 0.5f;
    float32 z = (-SQRT3 * valpha + vbeta) * 0.5f;

    uint8 sector = 0;
    if (x > 0) sector += 1;
    if (y > 0) sector += 2;
    if (z > 0) sector += 4;

    switch (sector) {
        case 3:
            t1 = z; t2 = x; break;
        case 1:
            t1 = -y; t2 = -z; break;
        case 5:
            t1 = y; t2 = -x; break;
        case 4:
            t1 = -x; t2 = z; break;
        case 6:
            t1 = -z; t2 = y; break;
        case 2:
            t1 = x; t2 = -y; break;
        default:
            t1 = 0; t2 = 0; break;
    }

    t0 = 1.0f - t1 - t2;
    if (t0 < 0) {
        float32 scale = 1.0f / (t1 + t2);
        t1 *= scale;
        t2 *= scale;
        t0 = 0;
    }

    switch (sector) {
        case 3:
            va = t1 + t2 + t0 * 0.5f;
            vb = t2 + t0 * 0.5f;
            vc = t0 * 0.5f;
            break;
        case 1:
            va = t1 + t0 * 0.5f;
            vb = t1 + t2 + t0 * 0.5f;
            vc = t0 * 0.5f;
            break;
        case 5:
            va = t0 * 0.5f;
            vb = t1 + t2 + t0 * 0.5f;
            vc = t2 + t0 * 0.5f;
            break;
        case 4:
            va = t0 * 0.5f;
            vb = t1 + t0 * 0.5f;
            vc = t1 + t2 + t0 * 0.5f;
            break;
        case 6:
            va = t2 + t0 * 0.5f;
            vb = t0 * 0.5f;
            vc = t1 + t2 + t0 * 0.5f;
            break;
        case 2:
            va = t1 + t2 + t0 * 0.5f;
            vb = t0 * 0.5f;
            vc = t1 + t0 * 0.5f;
            break;
        default:
            va = vb = vc = 0.5f;
            break;
    }

    *dutyA = va;
    *dutyB = vb;
    *dutyC = vc;
}
