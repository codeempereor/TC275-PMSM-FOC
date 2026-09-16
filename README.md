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
- [x] GTM ATOM PWM 初始化（6路互补PWM + 软件死区）
- [x] DRV8305 GPIO 模拟 SPI 通信框架
- [x] 3路INH开环旋转验证（有嗡嗡声 + 定位点，证明磁场方向正确）
- [x] FOC 算法框架（Clarke/Park/SVPWM + PID）
- [x] VADC 电流采样配置
- [x] AS5047P 编码器 QSPI 驱动框架

### 当前卡点 🚧

1. **SPI读DRV8305寄存器失败**：示波器确认SCLK有脉冲、SDO有数据跳变，但读回值不匹配（LED闪5次）
2. **6路PWM软件死区过流**：两种INL极性配置均触发nFAULT过流保护
3. **3路INH力矩不足**：只有上桥工作，半波电流，力矩不够电机转不起来

## 开发计划

### 阶段1：解决SPI通信（优先级最高）
- [ ] 确认SPI帧格式（16位 vs 其他长度）
- [ ] 确认采样点（上升沿 vs 下降沿）
- [ ] 确认SCS时序（拉低/拉高时机）
- [ ] 成功读取DRV8305 ID寄存器
- [ ] 配置DRV8305到3路PWM模式（寄存器0x06 bit8:7=01）
- [ ] 读取nFAULT状态寄存器，清除故障

### 阶段2：电机开环旋转
- [ ] 3路PWM模式下正弦开环旋转
- [ ] 验证转向、转速可调
- [ ] 确认力矩足够带动负载

### 阶段3：电流采样
- [ ] VADC初始化 3路电流采样
- [ ] ADC校准（零电流偏移）
- [ ] 电流采样值验证（手转电机轴看电流变化）

### 阶段4：闭环FOC
- [ ] Clarke变换 + Park变换
- [ ] 电流环PI调节（Id/Iq）
- [ ] SVPWM生成
- [ ] AS5047P编码器角度读取
- [ ] 速度环PI调节

## 控制算法

- **坐标变换**：Clarke 变换（3→2）、Park 变换（静止→旋转）、反 Park 变换
- **SVPWM**：七段式空间矢量发波，电压矢量合成与扇区判断
- **双闭环 PI**：电流环（d/q 轴解耦）+ 速度环
- **PWM 输出**：GTM ATOM 输出三相互补 PWM，带软件死区
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
