#include "foc_pi.h"

void FOC_PI_Init(FOC_PI_t *pi, float kp, float ki, float out_max, float out_min)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->output_max = out_max;
    pi->output_min = out_min;
    pi->prev_error = 0.0f;
}

void FOC_PI_Reset(FOC_PI_t *pi)
{
    pi->integral = 0.0f;
    pi->prev_error = 0.0f;
}

float FOC_PI_Run(FOC_PI_t *pi, float error, float dt)
{
    float output;
    float integral_new;

    pi->integral += error * dt;
    integral_new = pi->ki * pi->integral;
    output = pi->kp * error + integral_new;

    if (output > pi->output_max) {
        output = pi->output_max;
        if (error > 0.0f) {
            pi->integral -= error * dt;
        }
    } else if (output < pi->output_min) {
        output = pi->output_min;
        if (error < 0.0f) {
            pi->integral -= error * dt;
        }
    }

    pi->prev_error = error;
    return output;
}
