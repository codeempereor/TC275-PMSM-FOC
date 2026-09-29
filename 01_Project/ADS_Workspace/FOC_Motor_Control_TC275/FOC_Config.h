#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#include "Ifx_Types.h"

/*==================== PWM 引脚定义 (CCU60, Port 2) ====================
 * 实际硬件接线（2026-09-18 实测验证，电机已转）：
 *   A 相高侧 INH_A = P02.0  (CCU60 CC60)
 *   B 相高侧 INH_B = P02.7  (CCU60 CC61)
 *   C 相高侧 INH_C = P02.4  (CCU60 CC62)
 *   A 相低侧 INL_A = P02.1  (CCU60 COUT60) — 悬空（3xPWM 模式）
 *   B 相低侧 INL_B = P02.3  (CCU60 COUT61) — 悬空（3xPWM 模式）
 *   C 相低侧 INL_C = P02.5  (CCU60 COUT62) — 悬空（3xPWM 模式）
 *   EN_GATE      = 硬件直连 3.3V，不占 GPIO
 * DRV8305 已通过 SPI 配置为 3xPWM 模式（reg 0x07=0x2E6），INL 由芯片内部生成互补信号。
 * 注意：这些宏仅作文档对照用；FOC_PWM.c 实际通过 iLLD 引脚表 IfxCcu60_CC6x_P02_x_OUT 路由。
 */
#define PIN_INH_A      &MODULE_P02, 0    /* CC60  */
#define PIN_INL_A      &MODULE_P02, 1    /* COUT60（悬空） */
#define PIN_INH_B      &MODULE_P02, 7    /* CC61  */
#define PIN_INL_B      &MODULE_P02, 3    /* COUT61（悬空） */
#define PIN_INH_C      &MODULE_P02, 4    /* CC62  */
#define PIN_INL_C      &MODULE_P02, 5    /* COUT62（悬空） */

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
#define ENCODER_ZERO_OFFSET 8726          /* 2026-09-29 校准：alpha轴电压对齐法 */

/*==================== 系统参数 ====================*/
#define ADC_VREF           3.3f
#define ADC_SHUNT_GAIN     10.0f         /* DRV8305 内部增益 */
#define ADC_SHUNT_RESISTOR 0.007f        /* 7mΩ 采样电阻 */
#define ADC_CURRENT_SCALE  (ADC_VREF / 4095.0f / ADC_SHUNT_GAIN / ADC_SHUNT_RESISTOR)

#define SQRT3              1.7320508075688772f
#define PI                 3.141592653589793f
#define TWO_PI             6.283185307179586f

#endif /* FOC_CONFIG_H */
