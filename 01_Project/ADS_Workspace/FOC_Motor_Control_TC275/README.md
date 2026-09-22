# TC275 FOC 无刷电机控制项目

基于 Infineon TC275 + DRV8305 的无刷电机磁场定向控制 (FOC) 项目。

## 硬件平台

- **MCU**: Infineon AURIX TC275 (KIT_TC275_LK)
- **栅极驱动**: Texas Instruments BOOSTXL-DRV8305EVM
- **电机**: 外转子云台无刷电机 (极对数 7)
- **编码器**: AS5047P 14位磁编码器
- **供电**: 24V PVDD + USB 5V

## 已完成阶段

- [x] 开发板 Blinky LED 验证（Memtool 烧录成功）
- [x] 硬件引脚接线确认（见 [WIRING.md](WIRING.md)）
- [x] CCU60 三相互补 PWM 初始化（20kHz 中心对齐）
- [x] DRV8305 GPIO 模拟 SPI 通信调通（TestSPI 读写一致）
- [x] DRV8305 配置为 3xPWM 模式（reg 0x07=0x2E6，死区 3520ns）
- [x] nFAULT 实时监测 + LED 故障码解码
- [x] **开环正弦拖动电机旋转成功**（64 点 sin LUT，DUTY_AMPLITUDE=0.20）

## 当前阶段

开环 V/f 拖动已转起来。下一步：电流采样 → 编码器 → 闭环 FOC。

## 下一步计划

### 阶段 B：电流采样
- [ ] FOC_ADC_Init() 接入主程序
- [ ] 校准零电流偏移（替换硬编码 2048）
- [ ] ADC 改为 PWM 下半周期触发同步采样

### 阶段 C：编码器
- [ ] FOC_SPI_Init() 接入主程序
- [ ] 读 AS5047P 角度并验证线性

### 阶段 D：闭环 FOC
- [ ] 10kHz PWM 中断：ADC→Clarke→Park→电流环 PI→SVPWM
- [ ] 接入 AS5047P 角度反馈
- [ ] 加速度环外环

## 文件结构

```
FOC_Motor_Control_TC275/
├── Cpu0_Main.c          # 主程序入口
├── FOC_Config.h         # 引脚定义 + 电机参数配置
├── FOC_PWM.c / .h       # GTM ATOM PWM 初始化 + 占空比设置
├── FOC_DRV8305.c / .h   # DRV8305 GPIO模拟SPI驱动
├── FOC_SPI.c / .h       # QSPI1 硬件SPI (AS5047P编码器)
├── FOC_ADC.c / .h       # VADC 电流采样
├── FOC_Algorithm.c / .h  # FOC算法 (Clarke/Park/SVPWM)
├── WIRING.md            # 详细接线表
└── Libraries/           # iLLD 底层驱动库
```

## 开发环境

- AURIX Development Studio (ADS) + TASKING 编译器
- 烧录工具: Memtool 2021
- 调试: J-Link / UART
