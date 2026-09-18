#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#include "Ifx_Types.h"

/*==================== PWM 引脚定义 (Port 2) ====================*/
#define PIN_INH_A      &MODULE_P02, 0    /* PWM_2  */
#define PIN_INL_A      &MODULE_P02, 1    /* PWM_3  */
#define PIN_INH_B      &MODULE_P02, 3    /* PWM_5  */
#define PIN_INL_B      &MODULE_P02, 4    /* PWM_6  */
#define PIN_INH_C      &MODULE_P02, 5    /* PWM_7  */
#define PIN_INL_C      &MODULE_P02, 6    /* PWM_8  */
#define PIN_EN_GATE    &MODULE_P02, 6    /* PWM_8, 高电平使能 */

/*==================== 故障反馈引脚 ====================*/
#define PIN_NFAULT     &MODULE_P00, 3

/*==================== LED 引脚 ====================*/
#define PIN_LED1       &MODULE_P00, 5    /* 低电平点亮 */

/*==================== ADC 电流采样 (Port 40) ====================*/
#define PIN_ISEN_A     &MODULE_P40, 9    /* AN39 */
#define PIN_ISEN_B     &MODULE_P40, 8    /* AN38 */
#define PIN_ISEN_C     &MODULE_P40, 7    /* AN37 */

#define ADC_CH_A       39
#define ADC_CH_B       38
#define ADC_CH_C       37

/*==================== QSPI1 编码器 (Port 10) ====================*/
#define PIN_SPI_MOSI   &MODULE_P10, 3
#define PIN_SPI_MISO   &MODULE_P10, 1
#define PIN_SPI_SCLK   &MODULE_P10, 2
#define PIN_SPI_CSN    &MODULE_P10, 0    /* SS_MB, Mikrobus排针 */

/*==================== AS5047P 寄存器 ====================*/
#define AS5047P_CMD_READ   0x4000
#define AS5047P_REG_ANGLECOM  0x3FFF
#define AS5047P_REG_ANGLEUNC  0x3FFE
#define AS5047P_REG_MAG       0x3FFD
#define AS5047P_REG_ERRFL     0x0001

/*==================== FOC 参数 ====================*/
#define PWM_FREQ_HZ        20000.0f      /* 20kHz PWM */
#define PWM_DEADTIME_NS    1000.0f       /* 1us 死区 */
#define PWM_PERIOD_TICKS   1250          /* 100MHz/20kHz/2 = 2500, 中心对齐 */
#define PWM_MAX_DUTY       2500

/*==================== PID 参数 (电流环) ====================*/
#define PID_KP             0.5f
#define PID_KI             0.01f
#define PID_KD             0.0f
#define PID_OUT_MAX        1.0f

/*==================== PID 参数 (速度环) ====================*/
#define SPD_KP             0.1f
#define SPD_KI             0.005f
#define SPD_KD             0.0f
#define SPD_OUT_MAX        5.0f

/*==================== 电机参数 ====================*/
#define MOTOR_POLE_PAIRS   7             /* 极对数 */
#define ENCODER_RESOLUTION 16384.0f      /* 14位 */

/*==================== 系统参数 ====================*/
#define ADC_VREF           3.3f
#define ADC_SHUNT_GAIN     10.0f         /* DRV8305 内部增益 */
#define ADC_SHUNT_RESISTOR 0.007f        /* 7mΩ 采样电阻 */
#define ADC_CURRENT_SCALE  (ADC_VREF / 4095.0f / ADC_SHUNT_GAIN / ADC_SHUNT_RESISTOR)

#define SQRT3              1.7320508075688772f
#define PI                 3.141592653589793f
#define TWO_PI             6.283185307179586f

#endif /* FOC_CONFIG_H */
