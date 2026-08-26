#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#define FOC_PWM_FREQ_HZ           10000.0f
#define FOC_PWM_PERIOD_SEC        (1.0f / FOC_PWM_FREQ_HZ)
#define FOC_ISR_FREQ_HZ           FOC_PWM_FREQ_HZ
#define FOC_ISR_PERIOD_SEC        FOC_PWM_PERIOD_SEC
#define FOC_SPEED_LOOP_FREQ_HZ    1000.0f
#define FOC_SPEED_LOOP_DIV        (FOC_ISR_FREQ_HZ / FOC_SPEED_LOOP_FREQ_HZ)

#define FOC_POLE_PAIRS            4
#define FOC_ENCODER_LINES         2500
#define FOC_ENCODER_RESOLUTION    (FOC_ENCODER_LINES * 4)

#define FOC_MAX_DUTY              0.95f
#define FOC_MIN_DUTY              0.05f
#define FOC_VBUS_VOLTAGE          24.0f
#define FOC_VOLTAGE_MAX           (FOC_VBUS_VOLTAGE * 0.577350269f)

#define FOC_ID_KP                 0.5f
#define FOC_ID_KI                 20.0f
#define FOC_ID_MAX_OUTPUT         FOC_VOLTAGE_MAX
#define FOC_ID_MIN_OUTPUT         (-FOC_VOLTAGE_MAX)

#define FOC_IQ_KP                 0.5f
#define FOC_IQ_KI                 20.0f
#define FOC_IQ_MAX_OUTPUT         FOC_VOLTAGE_MAX
#define FOC_IQ_MIN_OUTPUT         (-FOC_VOLTAGE_MAX)

#define FOC_SPEED_KP              0.01f
#define FOC_SPEED_KI              0.5f
#define FOC_SPEED_MAX_OUTPUT      10.0f
#define FOC_SPEED_MIN_OUTPUT      (-10.0f)

#define FOC_ADC_VREF_V            3.3f
#define FOC_ADC_RESOLUTION        4095.0f
#define FOC_SHUNT_RESISTOR_OHM    0.01f
#define FOC_AMP_GAIN              10.0f
#define FOC_ADC_TO_AMP            (FOC_ADC_VREF_V / FOC_ADC_RESOLUTION)
#define FOC_AMP_TO_CURRENT        (1.0f / (FOC_SHUNT_RESISTOR_OHM * FOC_AMP_GAIN))
#define FOC_ADC_BIAS              2048.0f

#define FOC_ELEC_ANGLE_MAX        6.283185307f
#define FOC_SQRT3                 1.732050808f
#define FOC_INV_SQRT3             0.577350269f
#define FOC_TWO_DIV_SQRT3         1.154700538f

#endif
