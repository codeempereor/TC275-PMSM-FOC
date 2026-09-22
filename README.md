# 基于英飞凌 TC275 的 PMSM 电机 FOC 矢量控制系统

基于英飞凌 AURIX TC275TP 三核单片机，采用 AUTOSAR 分层架构，独立完成 PMSM 永磁同步电机 FOC 矢量控制系统从硬件接线到算法调试的全流程开发。

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

- [x] 开发板 Blinky LED 验证（Memtool 烧录成功）
- [x] 硬件引脚接线确认（见 [04_Docs/WIRING.md](04_Docs/WIRING.md)）
- [x] CCU60 三相互补 PWM 初始化（20kHz 中心对齐，硬件死区 5µs）
- [x] DRV8305 GPIO 模拟 SPI 通信调通（TestSPI 读写一致，LED 常亮）
- [x] DRV8305 配置为 3xPWM 模式（reg 0x07=0x2E6，死区 3520ns，实测避免 VDS 穿通）
- [x] DRV8305 锁存故障清除（reg 0x09=0x022）
- [x] nFAULT 实时监测 + LED 故障码解码（VDS/IC/VGS 三类）
- [x] **开环正弦拖动电机旋转成功**（64 点 sin LUT，三相互差 120°，DUTY_AMPLITUDE=0.20）
- [x] FOC 算法框架（Clarke/Park/SVPWM + PID，尚未接入主循环）
- [x] VADC 电流采样代码框架（尚未接入主程序，未校准零电流）
- [x] AS5047P 编码器 QSPI 驱动框架（尚未接入主程序）

### 当前阶段 🚧

开环 V/f 拖动已转起来，下一步是把电流采样和编码器接进来，进入闭环 FOC。

## 开发计划

### 阶段 A：地基清理（进行中）
- [x] 修正引脚文档与代码不一致（P02.0/7/4，EN_GATE 硬件直连）
- [x] 删除死代码 PIN_EN_GATE 宏
- [ ] 开环从空循环延时改到 CCU60 周期中断（10kHz），转速精确可控

### 阶段 B：电流采样打通
- [ ] FOC_ADC_Init() 接入主程序
- [ ] 校准零电流偏移（替换硬编码 2048）
- [ ] ADC 改为 PWM 下半周期触发同步采样
- [ ] 手转电机轴验证三相电流加法定理

### 阶段 C：编码器打通
- [ ] FOC_SPI_Init() 接入主程序
- [ ] 读 AS5047P ID/ERRFL 确认链路
- [ ] 手转轴验证角度线性变化（0~16383）
- [ ] 计算机械转速与电角度

### 阶段 D：闭环 FOC
- [ ] 10kHz PWM 中断内跑 FOC：ADC→Clarke→Park→电流环 PI→反 Park→SVPWM
- [ ] Id=0、Iq 开环台阶验证电流环收敛
- [ ] 接入 AS5047P 角度替换开环斜坡
- [ ] 加速度环外环（1kHz）

### 阶段 E：多核分工
- [ ] Core1 跑 10kHz 电流环 ISR
- [ ] Core2 跑 1kHz 速度环 + 故障监控
- [ ] Core0 跑调试通信

## 控制算法

- **坐标变换**：Clarke 变换（3→2）、Park 变换（静止→旋转）、反 Park 变换
- **SVPWM**：七段式空间矢量发波，电压矢量合成与扇区判断
- **双闭环 PI**：电流环（d/q 轴解耦）+ 速度环
- **PWM 输出**：CCU60 ATOM 三相互补 PWM，硬件死区 5µs + DRV8305 内部死区 3520ns
- **电流采样**：VADC 同步采样三相电流
- **角度获取**：QSPI 读取 AS5047P 编码器绝对角度

## 多核调度

| 核心 | 任务 | 频率 |
|------|------|------|
| Core0 | 主程序 + 开环测试 | 后台 |
| Core1 | 电流环实时控制（规划） | 10kHz |
| Core2 | 速度环与状态监控（规划） | 1kHz |

## 工具链

- **编译**：Tasking TriCore + AURIX Development Studio (ADS)
- **调试**：UDE Starterkit（在线调试、变量观测）
- **烧录**：Infineon Memtool 2021
- **版本管理**：Git + GitHub

## 目录结构

```
01_Project/TC275_demo/          # 核心工程
├── project/TC275/
│   ├── _02_CDD/                # 复杂设备驱动（自研代码）
│   │   ├── FOC_Config.h        # 引脚定义 + 电机参数
│   │   ├── FOC_PWM.c/h         # GTM ATOM PWM 驱动
│   │   ├── FOC_DRV8305.c/h     # DRV8305 GPIO SPI 驱动
│   │   ├── FOC_SPI.c/h         # QSPI1 编码器驱动
│   │   ├── FOC_ADC.c/h         # VADC 电流采样
│   │   └── FOC_Algorithm.c/h   # FOC 算法 (Clarke/Park/SVPWM)
│   ├── _03_MCAL/               # MCAL 驱动（EB tresos 生成）
│   ├── _01_BSW/                # BSW 服务层
│   └── _04_StartUp/            # 启动与主程序
│       └── Cpu0_Main.c         # 主程序入口
04_Docs/                        # 设计文档与学习资料
├── WIRING.md                   # 完整接线表
└── *.docx                      # 学习计划书与知识手册
```

## 技术栈

`C` `AUTOSAR` `TC275` `FOC` `SVPWM` `PMSM` `GTM` `VADC` `QSPI` `DRV8305` `PI Control`
