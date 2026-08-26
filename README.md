# 基于英飞凌 TC275 的 PMSM 电机 FOC 矢量控制系统

基于英飞凌 AURIX TC275TP 三核单片机，采用 AUTOSAR 分层架构，独立完成 PMSM 永磁同步电机 FOC 矢量控制系统从硬件接线到算法调试的全流程开发。

## 硬件平台

| 器件 | 型号 | 用途 |
|------|------|------|
| 主控 | 英飞凌 TC275 Lite Kit（AURIX TC275TP 三核） | 电机控制、通信、诊断 |
| 栅极驱动 | TI DRV8305 | 三相栅极驱动，内置三路电流采样放大器 |
| 电机 | 3505 PMSM 永磁同步电机 | 被控对象 |
| 编码器 | AS5047P 14 位磁编码器 | 转子角度/转速测量 |
| 调试器 | 板载 Miniwiggler（DAP） | 烧录与在线调试 |

## 控制算法

- **坐标变换**：Clarke 变换（3→2）、Park 变换（静止→旋转）、反 Park 变换
- **SVPWM**：七段式空间矢量发波，电压矢量合成与扇区判断
- **双闭环 PI**：电流环（d/q 轴解耦）+ 速度环，10kHz 电流环带宽
- **PWM 输出**：GTM TOM 输出 10kHz 三相互补 PWM，带死区
- **电流采样**：PWM 中点触发 VADC 同步采样两相电流
- **角度获取**：QSPI 读取 AS5047P 编码器绝对角度

## 多核调度

| 核心 | 任务 | 频率 |
|------|------|------|
| Core0 | CAN 通信与诊断 | 后台 |
| Core1 | 电流环实时控制 | 10kHz |
| Core2 | 速度环与状态监控 | 1kHz |

## 工具链

- **编译**：Tasking TriCore v6.3
- **调试**：UDE Starterkit 5.0（在线调试、变量观测）
- **烧录**：Infineon Memtool 2021
- **MCAL 配置**：EB tresos Studio（Port/Dio/Adc/Pwm/Spi/Can/Lin）
- **BSW 配置**：Vector DaVinci Configurator / Developer
- **版本管理**：Git + GitHub

## 性能指标

- 转速稳定控制在目标值 ±2%
- 电流环带宽 1kHz
- 支持正反转平滑切换
- 过流、过温故障保护

## 目录结构

```
01_Project/TC275_demo/          # 核心工程
├── project/TC275/
│   ├── _02_CDD/                # 复杂设备驱动（自研代码）
│   │   ├── FOC/                # FOC 算法框架
│   │   └── LedTest/            # GPIO/UART 测试驱动
│   ├── _03_MCAL/               # MCAL 驱动（EB tresos 生成）
│   ├── _01_BSW/                # BSW 服务层
│   └── _04_StartUp/            # 启动与主程序
04_Docs/                        # 设计文档与学习资料
```

## 技术栈

`C` `AUTOSAR` `TC275` `FOC` `SVPWM` `PMSM` `GTM` `VADC` `QSPI` `CAN` `Multi-core` `PI Control`
