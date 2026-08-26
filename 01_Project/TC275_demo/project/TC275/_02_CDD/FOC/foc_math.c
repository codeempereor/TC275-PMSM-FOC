#include "foc_math.h"

FOC_AB_t FOC_Clarke(FOC_ABC_t *abc)
{
    FOC_AB_t ab;
    ab.alpha = abc->a;
    ab.beta = (abc->a + 2.0f * abc->b) * FOC_INV_SQRT3;
    return ab;
}

FOC_DQ_t FOC_Park(FOC_AB_t *ab, float sin_theta, float cos_theta)
{
    FOC_DQ_t dq;
    dq.d = ab->alpha * cos_theta + ab->beta * sin_theta;
    dq.q = -ab->alpha * sin_theta + ab->beta * cos_theta;
    return dq;
}

FOC_AB_t FOC_InvPark(FOC_DQ_t *dq, float sin_theta, float cos_theta)
{
    FOC_AB_t ab;
    ab.alpha = dq->d * cos_theta - dq->q * sin_theta;
    ab.beta = dq->d * sin_theta + dq->q * cos_theta;
    return ab;
}

float FOC_WrapAngle(float angle)
{
    while (angle >= FOC_ELEC_ANGLE_MAX) {
        angle -= FOC_ELEC_ANGLE_MAX;
    }
    while (angle < 0.0f) {
        angle += FOC_ELEC_ANGLE_MAX;
    }
    return angle;
}

void FOC_SinCos(float angle, float *sin_val, float *cos_val)
{
    float sin_table[16] = {
        0.0f, 0.382683f, 0.707107f, 0.923880f,
        1.0f, 0.923880f, 0.707107f, 0.382683f,
        0.0f, -0.382683f, -0.707107f, -0.923880f,
        -1.0f, -0.923880f, -0.707107f, -0.382683f
    };
    float idx_f;
    int idx0, idx1;
    float frac;
    float sin0, sin1;

    angle = FOC_WrapAngle(angle);
    idx_f = angle * (16.0f / FOC_ELEC_ANGLE_MAX);
    idx0 = (int)idx_f;
    frac = idx_f - (float)idx0;
    idx1 = (idx0 + 1) & 0x0F;
    sin0 = sin_table[idx0];
    sin1 = sin_table[idx1];
    *sin_val = sin0 + frac * (sin1 - sin0);
    *cos_val = sin_table[(idx0 + 4) & 0x0F] + frac * (sin_table[(idx1 + 4) & 0x0F] - sin_table[(idx0 + 4) & 0x0F]);
}
