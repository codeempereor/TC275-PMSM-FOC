#include "foc_svpwm.h"
#include <stdint.h>

static float svpwm_ts;
static float svpwm_vdc_inv;

void FOC_SVPWM_Init(void)
{
    svpwm_ts = FOC_PWM_PERIOD_SEC;
    svpwm_vdc_inv = 1.0f / FOC_VBUS_VOLTAGE;
}

FOC_SVPWM_Output_t FOC_SVPWM_Run(FOC_AB_t *v_ab)
{
    FOC_SVPWM_Output_t out;
    float v_alpha, v_beta;
    float t1, t2, t0;
    float ta, tb, tc;
    int sector;
    float x, y, z;

    v_alpha = v_ab->alpha;
    v_beta = v_ab->beta;

    x = v_beta;
    y = (FOC_SQRT3 * v_alpha + v_beta) * 0.5f;
    z = (-FOC_SQRT3 * v_alpha + v_beta) * 0.5f;

    sector = 0;
    if (x > 0.0f) sector += 1;
    if (y > 0.0f) sector += 2;
    if (z > 0.0f) sector += 4;

    switch (sector) {
        case 3:
            t1 = z;
            t2 = x;
            ta = (svpwm_ts - t1 - t2) * 0.5f;
            tb = ta + t1;
            tc = tb + t2;
            out.sector = 1;
            break;
        case 1:
            t1 = -y;
            t2 = -z;
            ta = (svpwm_ts - t1 - t2) * 0.5f;
            tc = ta + t1;
            tb = tc + t2;
            out.sector = 2;
            break;
        case 5:
            t1 = x;
            t2 = y;
            tb = (svpwm_ts - t1 - t2) * 0.5f;
            ta = tb + t1;
            tc = ta + t2;
            out.sector = 3;
            break;
        case 4:
            t1 = -z;
            t2 = -x;
            tb = (svpwm_ts - t1 - t2) * 0.5f;
            tc = tb + t1;
            ta = tc + t2;
            out.sector = 4;
            break;
        case 6:
            t1 = y;
            t2 = z;
            tc = (svpwm_ts - t1 - t2) * 0.5f;
            tb = tc + t1;
            ta = tb + t2;
            out.sector = 5;
            break;
        case 2:
            t1 = -x;
            t2 = -y;
            tc = (svpwm_ts - t1 - t2) * 0.5f;
            ta = tc + t1;
            tb = ta + t2;
            out.sector = 6;
            break;
        default:
            ta = svpwm_ts * 0.5f;
            tb = ta;
            tc = ta;
            out.sector = 0;
            break;
    }

    t0 = svpwm_ts - t1 - t2;
    if (t0 < 0.0f) {
        float scale = svpwm_ts / (t1 + t2);
        ta *= scale;
        tb *= scale;
        tc *= scale;
    }

    out.duty_a = ta * svpwm_vdc_inv / svpwm_ts;
    out.duty_b = tb * svpwm_vdc_inv / svpwm_ts;
    out.duty_c = tc * svpwm_vdc_inv / svpwm_ts;

    if (out.duty_a > FOC_MAX_DUTY) out.duty_a = FOC_MAX_DUTY;
    if (out.duty_a < FOC_MIN_DUTY) out.duty_a = FOC_MIN_DUTY;
    if (out.duty_b > FOC_MAX_DUTY) out.duty_b = FOC_MAX_DUTY;
    if (out.duty_b < FOC_MIN_DUTY) out.duty_b = FOC_MIN_DUTY;
    if (out.duty_c > FOC_MAX_DUTY) out.duty_c = FOC_MAX_DUTY;
    if (out.duty_c < FOC_MIN_DUTY) out.duty_c = FOC_MIN_DUTY;

    return out;
}
