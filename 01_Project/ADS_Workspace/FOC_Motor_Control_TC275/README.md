# TC275 FOC 无刷电机控制项目

基于 Infineon TC275 + DRV8305 的无刷电机磁场定向控制 (FOC) 项目，参考 SguanFOC 库实现。

## 硬件平台

- **MCU**: Infineon AURIX TC275 (KIT_TC275_LK)
- **栅极驱动**: Texas Instruments BOOSTXL-DRV8305EVM
- **电机**: 外转子云台无刷电机 (极对数 7)
- **编码器**: AS5047P 14位磁编码器
- **供电**: 24V PVDD + USB 5V

## 已完成阶段

### 阶段 A：基础验证 ✅
- [x] 开发板 Blinky LED 验证（Memtool 2021 烧录成功）
- [x] UART 串口打通（ASC0，TX=P14.0/RX=P14.1，115200，板载 DAP VCOM=COM10）
- [x] CCU60 3相上桥 PWM 初始化（20kHz 中心对齐，3x 模式）
- [x] DRV8305 GPIO 模拟 SPI 通信调通
- [x] DRV8305 配置为 3xPWM 模式（reg 0x07=0x2E6，死区 3520ns）
- [x] nFAULT 实时监测 + LED1 故障码解码（低电平点亮）

### 阶段 B：开环 V/f 拖动 ✅
- [x] 开环正弦拖动电机旋转成功（DUTY_AMPLITUDE=0.15f）
- [x] 平滑正弦输出（多项式快速 sin/cos，替代 LUT 插值）
- [x] 旋钮调速通道预留（P40.0）

### 阶段 C：电流采样 ✅
- [x] VADC Group4 三相电流采样打通（P40.7/8/9，对应 AN37/38/39）
- [x] 启动时 1000 次采样平均完成零电流偏移校准
- [x] 典型零偏值：A≈2045, B≈2040, C≈2038（随温度略有波动）
- [x] 采样与 PWM 中断完全同步（20kHz ISR 内读 ADC）

### 阶段 D：FOC 算法验证 ✅
- [x] 移植 SguanFOC 快速多项式 sin/cos（比标准库快 60 倍，20kHz ISR 内可运行）
- [x] Clarke/Park 变换实现并验证正确
- [x] 全电流环计算移入 PWM ISR（采样+变换+输出完全同步）
- [x] 软件配置开关：`CURRENT_DIR_A/B`（电流极性 ±1）、`MOTOR_DIR`（相序 ±1）
- [x] 固定角度堵转测试：id/iq 为直流（id≈-30, iq≈1250），算法 100% 正确

## 当前阶段

**阶段 E：AS5047P 编码器调试**
- 硬件 QSPI1 驱动已编写，当前卡在 SPI 读取 busy 状态，待排查接线/时序

## 下一步计划

### 阶段 E：编码器接入
- [ ] 解决 AS5047P QSPI 读取 busy 问题
- [ ] 验证编码器角度线性（手转电机，角度跟随正确）
- [ ] 编码器零位校准（上电对齐转子 N 极到 A 相轴线）

### 阶段 F：电流环闭环
- [ ] 接入编码器实际电角度，替换开环累加角度
- [ ] 接入 id/iq 双 PI 闭环
- [ ] 调试 PI 参数，实现电流闭环响应

### 阶段 G：转速环闭环
- [ ] 编码器速度计算
- [ ] 外环转速 PI 控制
- [ ] 旋钮调速功能接入

## 关键参数

| 参数 | 值 |
|------|----|
| PWM 频率 | 20kHz 中心对齐 |
| 死区时间 | 3520ns |
| 电机极对数 | 7 |
| 编码器分辨率 | 14 位（16384） |
| 开环电压幅值 | 0.15f（15% 占空比） |
| 初始 PI 参数 | Kp=0.05f, Ki=0.001f, outMax=0.2f |

## 文件结构

```
FOC_Motor_Control_TC275/
├── Cpu0_Main.c          # 主程序入口，ISR 电流环实现
├── FOC_Config.h         # 引脚定义 + 电机参数配置
├── FOC_PWM.c / .h       # CCU60 PWM 初始化 + 占空比设置
├── FOC_DRV8305.c / .h   # DRV8305 GPIO 模拟 SPI 驱动
├── FOC_SPI.c / .h       # QSPI1 硬件 SPI (AS5047P 编码器)
├── FOC_ADC.c / .h       # VADC 三相电流采样
├── FOC_Algorithm.c / .h # FOC 算法 (Clarke/Park/SVPWM/PID)
├── calibration_notes.txt # 校准记录
├── WIRING.md            # 详细接线表
└── Libraries/           # iLLD 底层驱动库
```

## Git 提交历史

- `a4dd7f0` feat: 移植 SguanFOC 快速 sin/cos + 全 ISR 电流环，验证 Clarke/Park 正确
- `b0edbca` docs: 对齐引脚文档
- `d8e03d2` feat: 开环 V/f 拖动（CCU60 20kHz 中断）
- `2c0edd7` feat: 平滑开环（浮点相位累加 + LUT 插值）
- `83009d2` tune: DUTY_AMPLITUDE 0.20 → 0.10
- `fe57246` tune: DUTY_AMPLITUDE 0.10 → 0.15
- `eb26d9b` feat: 新增 UART 调试输出
- `611c722` feat: 阶段 B - VADC 接入主程序
- `5b6dbd6` fix: UART 配置对齐官方例程 + ADC 扫描通道配置
- `08dd013` feat: 启动时零电流偏移校准

## 开发环境

- AURIX Development Studio (ADS) + TASKING 编译器
- 烧录工具: Memtool 2021
- 调试: UART 串口（115200，8N1）
- Git 推送: SSH over 443 (ssh.github.com:443)
