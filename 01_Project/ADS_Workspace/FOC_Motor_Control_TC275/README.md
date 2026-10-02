# TC275 FOC 无刷电机控制项目

基于 Infineon TC275 + DRV8305 的云台无刷电机磁场定向控制 (FOC) 项目，参考 SguanFOC 库实现。

## 硬件平台

- **MCU**: Infineon AURIX TC275（立英电擎 Lite-Kit，SAK-TC275TP-64F200W）
- **栅极驱动**: Texas Instruments BOOSTXL-DRV8305EVM（3xPWM 模式）
- **电机**: ProDrone 外转子云台无刷电机（**极对数 10**，24V PVDD，相阻约 10Ω）
- **编码器**: AS5047P 14bit 磁编码器（16384 CPR，QSPI1 5MHz）
- **供电**: 24V PVDD + USB 5V

## 已完成阶段

### 阶段 A：基础验证 ✅
- [x] 开发板 Blinky LED 验证（Memtool 2021 烧录成功）
- [x] UART 串口打通（ASC0，TX=P14.0/RX=P14.1，115200，板载 VCOM）
- [x] CCU60 3相上桥 PWM 初始化（20kHz 中心对齐，3xPWM 模式）
- [x] DRV8305 GPIO 模拟 SPI 通信调通 + 配置（reg 0x07=0x2E6，死区 3520ns）
- [x] nFAULT 实时监测 + LED1 故障码解码（低电平点亮）

### 阶段 B：开环 V/f 拖动 ✅
- [x] 开环正弦拖动电机旋转成功（平滑正弦输出）
- [x] 相位累加 + 多项式快速 sin/cos

### 阶段 C：电流采样 ✅
- [x] VADC Group4 三相电流采样打通（P40.7/8/9 = AN37/38/39 = G4Ch5/6/7，固定采样 A/B，C 相 ic=-(ia+ib)）
- [x] 启动时 1000 次采样平均零电流偏移校准（典型 A≈2045、B≈2040、C≈2038）
- [x] 采样与 PWM 中断完全同步（20kHz ISR 内软件触发扫描）
- [x] **VADC Group0 电位器旋钮采样**（AN0 = G0Ch0，后台自动扫描）——本轮新增

### 阶段 D：FOC 算法验证 ✅
- [x] Clarke/Park 变换 + 电流环双 PI（Kp=0.0005/Ki=0.0002，duty 限幅 [0.05,0.45]）
- [x] 软件配置开关：`CURRENT_DIR_A/B`（±1）、`MOTOR_DIR`（±1）
- [x] 固定角度堵转测试：id/iq 为直流，算法正确
- [x] 关键组合已验证：MOTOR_DIR=+1、CURRENT_DIR=+1、Park 角度取反、极对数 10

### 阶段 E：编码器接入 ✅
- [x] QSPI1 硬件 SPI 读取 AS5047P（P10.0/1/2/3，Mode1 5MHz）
- [x] 主循环单帧读取 + 坏帧过滤（delta > ±4096 丢弃）
- [x] 上电预定位强拉转子 → 编码器零偏校准（每次上电不同属正常，无漂移）

### 阶段 F：I/f 自启动 + 电流环闭环 ✅（commit 0d23e76，已 push）
- [x] 三段状态机：0=预定位（id 斜坡 0→200）→ 1=开环加速（ω 2→60 rad/s 电频率，id=200 恒定）→ 2=闭环（θ_i 与电角度差 <0.3 切换）
- [x] 闭环后 iq_ref=-50 恒定，电流环持续转动（逆时针，0.58A）
- [x] 电流斜坡拖动平滑无冲击（标准 I/f）

### 阶段 G：速度环双闭环 ✅（B 增量，工作区未提交）
- [x] 速度测量：ISR 计数 100 ISR=5ms@20kHz，编码器 delta 累计 → 机械 rad/s，滤波 0.8/0.2
- [x] 速度环：Kp=0.5/Ki=0.05、dQ 斜坡 ±6、iq_ref 限幅 ±150、windup 149、积分限幅 ±3000
- [x] 切换瞬间 SW 诊断快照（th/el 打印一次）
- [x] 磁极 π 自动纠正：方向确认窗（dir_accum ±400 判向）+ 运行中方向自检（spd×sref 反向 → 翻 π）
- [x] **实测收敛**：spd 从 -2806 收敛至 -1000±15%，iqr 回落稳在 -13~-16，方向逆时针正确
- [x] vd 限幅 0.2（防电压矢量拉偏卡死）
- [ ] 已知问题：启动切换后确认窗（iq=-50）力矩不足时电机卡在齿槽，需手拨才转（**待调优，本轮明确不调**）

### 阶段 H：旋钮调速 ✅（本轮新增，工作区未提交）
- [x] 板载电位器 = AN0 = **VADC Group0 Ch0**（引脚 67，纯模拟输入，无需 GPIO 配置）
- [x] 后台自动扫描（autoscan），主循环每 100 次读一次
- [x] 映射：pot 0~4095 → `g_speed_ref` 0 ~ -12 rad/s（逆时针），raw<40 死区停，一阶低通防跳变
- [x] 串口新增 `pot=` 字段打印

## 当前阶段

**速度环闭环运行 + 旋钮调速**：旋钮从零位拧起 → 电机逆时针加速至 -12 rad/s（机械），串口 `sref` 跟随 `pot` 变化。
启动瞬间仍可能卡齿槽（确认窗启动力矩不足），待后续调优。

## 关键参数

| 参数 | 值 |
|------|----|
| PWM 频率 | 20kHz 中心对齐 |
| 死区时间 | 3520ns |
| 电机极对数 | **10** |
| 编码器分辨率 | 14bit（16384） |
| duty 限幅 | [0.05, 0.45] |
| 电流环 PID | Kp=0.0005 / Ki=0.0002 / vd 限幅 0.2 / vq 限幅 0.5 |
| I/f 启动 | I_START=200，ω 2→60 rad/s，斜坡 100，预定位 6000×ISR |
| 速度环 PID | Kp=0.5 / Ki=0.05，iq_ref ±150，dQ ±6/次，积分 ±3000 |
| 旋钮调速 | pot 0~4095 → sref 0~-12 rad/s（死区 40，一阶滤波 0.9/0.1） |
| 速度测量 | ISR 计数窗 5ms，编码器 delta → 机械 rad/s |

## 文件结构

```
FOC_Motor_Control_TC275/
├── Cpu0_Main.c          # 主程序入口：I/f 状态机 + 电流环 ISR + 速度环 + 旋钮调速
├── FOC_Config.h         # 引脚定义 + 电机参数配置
├── FOC_PWM.c / .h       # CCU60 PWM 初始化 + 占空比设置
├── FOC_DRV8305.c / .h   # DRV8305 GPIO 模拟 SPI 驱动
├── FOC_SPI.c / .h       # QSPI1 硬件 SPI (AS5047P 编码器)
├── FOC_ADC.c / .h       # VADC Group4 电流采样 + Group0 电位器
├── FOC_Algorithm.c / .h # FOC 算法 (Clarke/Park/SVPWM/PID)
├── calibration_notes.txt # 校准记录
├── WIRING.md            # 详细接线表（含旋钮）
└── Libraries/           # iLLD 底层驱动库
```

## Git 提交历史

- `0d23e76` feat: I/f startup working（预定位→开环加速→闭环，电流环持续转，已 push main）
- `a4dd7f0` feat: 移植 SguanFOC 快速 sin/cos + 全 ISR 电流环
- `b0edbca` docs: 对齐引脚文档
- `d8e03d2` feat: 开环 V/f 拖动（CCU60 20kHz 中断）
- `2c0edd7` feat: 平滑开环（浮点相位累加）
- `eb26d9b` feat: 新增 UART 调试输出
- `611c722` feat: 阶段 B - VADC 接入主程序
- `5b6dbd6` fix: UART 配置对齐官方例程 + ADC 扫描通道配置
- `08dd013` feat: 启动时零电流偏移校准

> 工作区未提交：速度环 B 增量 + 旋钮调速（当前为速度环收敛 + 旋钮调速组合状态）

## 开发环境

- AURIX Development Studio (ADS) + TASKING 编译器（sint32 等 TASKING 类型）
- 烧录工具: Memtool 2021
- 调试: UART 串口（115200，8N1）
- Git 推送: SSH over 443 (ssh.github.com:443)
