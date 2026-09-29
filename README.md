# 基于英飞凌 TC275 的 PMSM 电机 FOC 矢量控制系统

基于英飞凌 AURIX TC275TP 三核单片机，独立完成 PMSM 永磁同步电机 FOC 矢量控制系统从硬件接线到算法调试的全流程开发。

## 硬件平台

| 器件 | 型号 | 用途 |
|------|------|------|
| 主控 | 英飞凌 TC275 Lite Kit（AURIX TC275TP 三核） | 电机控制、通信、诊断 |
| 栅极驱动 | TI DRV8305 (BOOSTXL-DRV8305EVM) | 三相栅极驱动，内置三路电流采样放大器 |
| 电机 | 外转子云台无刷电机，极对数 7 | 被控对象 |
| 编码器 | AS5047P 14 位磁编码器 | 转子角度/转速测量 |
| 电源 | 24V PVDD + USB 5V | 供电 |
| 调试器 | 板载 Miniwiggler（DAP） | 烧录与在线调试 |

## 当前开发进展

### 已完成 ✅

#### 阶段 A：基础验证
- [x] 开发板 Blinky LED 验证（Memtool 烧录成功）
- [x] 硬件引脚接线确认（见 [01_Project/ADS_Workspace/FOC_Motor_Control_TC275/WIRING.md](01_Project/ADS_Workspace/FOC_Motor_Control_TC275/WIRING.md)）
- [x] UART 串口打通（ASC0，TX=P14.0/RX=P14.1，115200，板载 DAP VCOM=COM10）
- [x] CCU60 3相上桥 PWM 初始化（20kHz 中心对齐，3x 模式）
- [x] DRV8305 GPIO 模拟 SPI 通信调通（TestSPI 读写一致，LED 常亮）
- [x] DRV8305 配置为 3xPWM 模式（reg 0x07=0x2E6，死区 3520ns，实测避免 VDS 穿通）
- [x] DRV8305 锁存故障清除（reg 0x09=0x022）
- [x] nFAULT 实时监测 + LED 故障码解码（VDS/IC/VGS 三类）

#### 阶段 B：开环 V/f 拖动
- [x] 开环正弦拖动电机旋转成功（DUTY_AMPLITUDE=0.15f）
- [x] 平滑正弦输出（多项式快速 sin/cos，替代 LUT 插值）
- [x] 旋钮调速通道预留（P40.0）

#### 阶段 C：电流采样
- [x] VADC Group4 三相电流采样打通（P40.7/8/9，对应 AN37/38/39）
- [x] 启动时 1000 次采样平均完成零电流偏移校准
- [x] 典型零偏值：A≈2045, B≈2040, C≈2038（随温度略有波动）
- [x] 采样与 PWM 中断完全同步（20kHz ISR 内读 ADC）

#### 阶段 D：FOC 算法验证
- [x] 移植 SguanFOC 快速多项式 sin/cos（比标准库快 60 倍，20kHz ISR 内可运行）
- [x] Clarke/Park 变换实现并验证正确
- [x] 全电流环计算移入 PWM ISR（采样+变换+输出完全同步）
- [x] 软件配置开关：`CURRENT_DIR_A/B`（电流极性 ±1）、`MOTOR_DIR`（相序 ±1）
- [x] 固定角度堵转测试：id/iq 为直流（id≈-30, iq≈1250），算法 100% 正确

#### 阶段 E：AS5047P 编码器接入 ✅
- [x] QSPI1 硬件 SPI 驱动（SCLK=P10.2, MOSI=P10.3, MISO=P10.1, CSN=P10.0）
- [x] AS5047P 命令帧偶校验（bit15 PARD）自动计算
- [x] SPI Mode 1（CPOL=0, CPHA=1）显式配置
- [x] QSPI1 TX/RX/Error 三中断（pri 4/5/6）清除 onTransfer 标志
- [x] 手转电机验证：angle 0~16383 单调变化、回绕正确、EF 恒为 0

### 当前阶段 🚧

**阶段 F：电流环闭环**

## 下一步计划

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
| 死区时间 | 3520ns（DRV8305 内部） |
| 电机极对数 | 7 |
| 编码器分辨率 | 14 位（16384） |
| 开环电压幅值 | 0.15f（15% 占空比） |
| 初始 PI 参数 | Kp=0.05f, Ki=0.001f, outMax=0.2f |

## 控制算法

- **坐标变换**：Clarke 变换（3→2）、Park 变换（静止→旋转）、反 Park 变换
- **三角函数**：3次多项式快速拟合（比标准库快60倍）
- **双闭环 PI**：电流环（d/q 轴解耦），待调参
- **PWM 输出**：CCU60 3路上桥 PWM，DRV8305 内部生成互补下桥
- **电流采样**：VADC 同步采样三相电流，与PWM中断完全同步
- **角度获取**：QSPI 读取 AS5047P 编码器绝对角度（调试中）

## 工具链

- **编译**：Tasking TriCore + AURIX Development Studio (ADS)
- **调试**：UART 串口（115200，8N1）
- **烧录**：Infineon Memtool 2021
- **版本管理**：Git + GitHub，SSH over 443 推送

## 目录结构

```
FOC_Motor_Control_TC275/          # 核心工程（01_Project/ADS_Workspace/）
├── Cpu0_Main.c          # 主程序入口，ISR 电流环实现
├── FOC_Config.h         # 引脚定义 + 电机参数配置
├── FOC_PWM.c / .h       # CCU60 PWM 初始化 + 占空比设置
├── FOC_DRV8305.c / .h   # DRV8305 GPIO 模拟 SPI 驱动
├── FOC_SPI.c / .h       # QSPI1 硬件 SPI (AS5047P 编码器)
├── FOC_ADC.c / .h       # VADC 三相电流采样
├── FOC_Algorithm.c / .h # FOC 算法 (Clarke/Park/SVPWM/PID)
├── calibration_notes.txt # 校准记录
└── WIRING.md            # 详细接线表
```

## Git 提交历史

- `26c9ef8` docs: 更新根 README
- `<本次>` feat: AS5047P 编码器 QSPI 接入完成（偶校验+Mode1+三中断）
- `f8bd338` docs: 更新文档至当前状态
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

## 技术栈

`C` `TC275` `FOC` `SVPWM` `PMSM` `VADC` `QSPI` `DRV8305` `PI Control`
