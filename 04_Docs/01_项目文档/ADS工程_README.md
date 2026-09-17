# TC275 FOC 无刷电机控制项目

基于 Infineon TC275 + DRV8305 的无刷电机磁场定向控制 (FOC) 项目。

## 硬件平台

- **MCU**: Infineon AURIX TC275 (KIT_TC275_LK)
- **栅极驱动**: Texas Instruments BOOSTXL-DRV8305EVM
- **电机**: 外转子云台无刷电机 (极对数 7)
- **编码器**: AS5047P 14位磁编码器
- **供电**: 24V PVDD + USB 5V

## 已完成阶段

- [x] 开发板 Blinky LED 验证 (Memtool 烧录成功)
- [x] 硬件引脚接线确认 (见 [WIRING.md](WIRING.md))
- [x] GTM ATOM PWM 初始化 (6路互补PWM + 软件死区)
- [x] DRV8305 GPIO 模拟 SPI 通信框架
- [x] 3路INH开环旋转验证 (有嗡嗡声 + 定位点，证明磁场方向正确)

## 当前进展与卡点

### 已验证正常
- 3路INH正弦开环：电机有嗡嗡声，手拧有定位点 → 磁场方向正确
- PWM输出正常：示波器确认SCLK/SDO有波形
- LED指示正常：闪5次 = SPI失败

### 当前卡点
1. **SPI读DRV8305寄存器失败**：示波器确认SCLK有脉冲、SDO有数据跳变，但读回值不匹配（LED闪5次）
2. **6路PWM软件死区过流**：两种INL极性配置均触发nFAULT过流保护
3. **3路INH力矩不足**：只有上桥工作，半波电流，力矩不够电机转不起来

## 项目计划 (下一步)

### 阶段1：解决SPI通信 (优先级最高)
- [ ] 确认SPI帧格式 (16位 vs 其他长度)
- [ ] 确认采样点 (上升沿 vs 下降沿)
- [ ] 确认SCS时序 (拉低/拉高时机)
- [ ] 成功读取DRV8305 ID寄存器
- [ ] 配置DRV8305到3路PWM模式 (寄存器0x06 bit8:7=01)
- [ ] 读取nFAULT状态寄存器，清除故障

### 阶段2：电机开环旋转
- [ ] 3路PWM模式下正弦开环旋转
- [ ] 验证转向、转速可调
- [ ] 确认力矩足够带动负载

### 阶段3：电流采样
- [ ] VADC初始化 3路电流采样
- [ ] ADC校准 (零电流偏移)
- [ ] 电流采样值验证 (手转电机轴看电流变化)

### 阶段4：闭环FOC
- [ ] Clarke变换 + Park变换
- [ ] 电流环PI调节 (Id/Iq)
- [ ] SVPWM生成
- [ ] AS5047P编码器角度读取
- [ ] 速度环PI调节

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
