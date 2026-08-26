#ifndef FOC_H
#define FOC_H

#include "foc_config.h"
#include "foc_math.h"
#include "foc_pi.h"
#include "foc_svpwm.h"
#include <stdint.h>

typedef enum {
    FOC_STATE_IDLE = 0,
    FOC_STATE_READY,
    FOC_STATE_RUNNING,
    FOC_STATE_FAULT
} FOC_State_t;

typedef struct {
    FOC_State_t state;
    FOC_ABC_t i_abc;
    FOC_AB_t i_ab;
    FOC_DQ_t i_dq;
    FOC_DQ_t v_dq;
    FOC_AB_t v_ab;
    FOC_SVPWM_Output_t pwm;
    FOC_PI_t pi_id;
    FOC_PI_t pi_iq;
    FOC_PI_t pi_speed;
    float elec_angle;
    float mech_angle;
    float speed_rpm;
    float speed_ref;
    float id_ref;
    float iq_ref;
    uint32_t speed_loop_counter;
    uint32_t pwm_counter;
} FOC_Handle_t;

void FOC_Init(FOC_Handle_t *foc);
void FOC_Start(FOC_Handle_t *foc);
void FOC_Stop(FOC_Handle_t *foc);
void FOC_SetSpeed(FOC_Handle_t *foc, float speed_rpm);
void FOC_CurrentLoop(FOC_Handle_t *foc);
void FOC_SpeedLoop(FOC_Handle_t *foc);
float FOC_GetSpeed(FOC_Handle_t *foc);
FOC_State_t FOC_GetState(FOC_Handle_t *foc);

void FOC_HAL_ReadPhaseCurrents(FOC_ABC_t *i_abc);
float FOC_HAL_ReadMechAngle(void);
float FOC_HAL_ReadSpeed(void);
void FOC_HAL_WritePWM(FOC_SVPWM_Output_t *pwm);

#endif
