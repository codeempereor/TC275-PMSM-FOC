#include "foc.h"

static FOC_Handle_t *foc_instance;

void FOC_Init(FOC_Handle_t *foc)
{
    foc_instance = foc;
    foc->state = FOC_STATE_IDLE;
    foc->i_abc.a = 0.0f;
    foc->i_abc.b = 0.0f;
    foc->i_abc.c = 0.0f;
    foc->i_ab.alpha = 0.0f;
    foc->i_ab.beta = 0.0f;
    foc->i_dq.d = 0.0f;
    foc->i_dq.q = 0.0f;
    foc->v_dq.d = 0.0f;
    foc->v_dq.q = 0.0f;
    foc->v_ab.alpha = 0.0f;
    foc->v_ab.beta = 0.0f;
    foc->elec_angle = 0.0f;
    foc->mech_angle = 0.0f;
    foc->speed_rpm = 0.0f;
    foc->speed_ref = 0.0f;
    foc->id_ref = 0.0f;
    foc->iq_ref = 0.0f;
    foc->speed_loop_counter = 0;
    foc->pwm_counter = 0;

    FOC_PI_Init(&foc->pi_id, FOC_ID_KP, FOC_ID_KI, FOC_ID_MAX_OUTPUT, FOC_ID_MIN_OUTPUT);
    FOC_PI_Init(&foc->pi_iq, FOC_IQ_KP, FOC_IQ_KI, FOC_IQ_MAX_OUTPUT, FOC_IQ_MIN_OUTPUT);
    FOC_PI_Init(&foc->pi_speed, FOC_SPEED_KP, FOC_SPEED_KI, FOC_SPEED_MAX_OUTPUT, FOC_SPEED_MIN_OUTPUT);
    FOC_SVPWM_Init();
    foc->state = FOC_STATE_READY;
}

void FOC_Start(FOC_Handle_t *foc)
{
    if (foc->state == FOC_STATE_READY) {
        FOC_PI_Reset(&foc->pi_id);
        FOC_PI_Reset(&foc->pi_iq);
        FOC_PI_Reset(&foc->pi_speed);
        foc->state = FOC_STATE_RUNNING;
    }
}

void FOC_Stop(FOC_Handle_t *foc)
{
    foc->speed_ref = 0.0f;
    foc->iq_ref = 0.0f;
    foc->v_dq.d = 0.0f;
    foc->v_dq.q = 0.0f;
    foc->pwm.duty_a = 0.5f;
    foc->pwm.duty_b = 0.5f;
    foc->pwm.duty_c = 0.5f;
    FOC_HAL_WritePWM(&foc->pwm);
    foc->state = FOC_STATE_READY;
}

void FOC_SetSpeed(FOC_Handle_t *foc, float speed_rpm)
{
    foc->speed_ref = speed_rpm;
}

void FOC_CurrentLoop(FOC_Handle_t *foc)
{
    float sin_theta, cos_theta;
    float error_d, error_q;

    if (foc->state != FOC_STATE_RUNNING) {
        return;
    }

    foc->pwm_counter++;
    FOC_HAL_ReadPhaseCurrents(&foc->i_abc);
    foc->mech_angle = FOC_HAL_ReadMechAngle();
    foc->elec_angle = FOC_WrapAngle(foc->mech_angle * (float)FOC_POLE_PAIRS);
    FOC_SinCos(foc->elec_angle, &sin_theta, &cos_theta);

    foc->i_ab = FOC_Clarke(&foc->i_abc);
    foc->i_dq = FOC_Park(&foc->i_ab, sin_theta, cos_theta);

    error_d = foc->id_ref - foc->i_dq.d;
    error_q = foc->iq_ref - foc->i_dq.q;
    foc->v_dq.d = FOC_PI_Run(&foc->pi_id, error_d, FOC_ISR_PERIOD_SEC);
    foc->v_dq.q = FOC_PI_Run(&foc->pi_iq, error_q, FOC_ISR_PERIOD_SEC);

    foc->v_ab = FOC_InvPark(&foc->v_dq, sin_theta, cos_theta);
    foc->pwm = FOC_SVPWM_Run(&foc->v_ab);
    FOC_HAL_WritePWM(&foc->pwm);

    foc->speed_loop_counter++;
    if (foc->speed_loop_counter >= FOC_SPEED_LOOP_DIV) {
        foc->speed_loop_counter = 0;
        FOC_SpeedLoop(foc);
    }
}

void FOC_SpeedLoop(FOC_Handle_t *foc)
{
    float speed_error;

    if (foc->state != FOC_STATE_RUNNING) {
        return;
    }

    foc->speed_rpm = FOC_HAL_ReadSpeed();
    speed_error = foc->speed_ref - foc->speed_rpm;
    foc->iq_ref = FOC_PI_Run(&foc->pi_speed, speed_error, (1.0f / FOC_SPEED_LOOP_FREQ_HZ));
}

float FOC_GetSpeed(FOC_Handle_t *foc)
{
    return foc->speed_rpm;
}

FOC_State_t FOC_GetState(FOC_Handle_t *foc)
{
    return foc->state;
}

__attribute__((weak)) void FOC_HAL_ReadPhaseCurrents(FOC_ABC_t *i_abc)
{
    i_abc->a = 0.0f;
    i_abc->b = 0.0f;
    i_abc->c = 0.0f;
}

__attribute__((weak)) float FOC_HAL_ReadMechAngle(void)
{
    return 0.0f;
}

__attribute__((weak)) float FOC_HAL_ReadSpeed(void)
{
    return 0.0f;
}

__attribute__((weak)) void FOC_HAL_WritePWM(FOC_SVPWM_Output_t *pwm)
{
    (void)pwm;
}
