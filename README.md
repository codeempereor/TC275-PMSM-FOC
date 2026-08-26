# TC275 AUTOSAR FOC 学习项目

基于英飞凌 AURIX TC275 Lite Kit 的 AUTOSAR 开发与 PMSM 电机 FOC 矢量控制学习记录。

## 硬件平台
- TC275 Lite Kit（三核，AURIX TC275TP）
- DRV8305 三相栅极驱动
- 3505 PMSM 电机 + AS5047P 磁编码器

## 软件工具链
- Tasking TriCore v6.3 编译器
- UDE 在线调试
- EB tresos MCAL 配置
- Vector DaVinci BSW 配置

## 学习进度
- [x] Day1-2: 环境搭建 + GPIO 输出（LED 闪烁）
- [x] Day3: GPIO 输入（按键控制 LED）
- [x] Day4: UART 串口打印（ASCLIN0 纯寄存器）
- [ ] Day5: GTM_TOM_PWM 输出
- [ ] Day6: GTM_TIM 输入捕获 + 中断
- [ ] Day7: VADC 模数转换
- [ ] Day8: QSPI 通信
- [ ] Day9: DMA + 中断系统
- [ ] Day10: 多核调度
- [ ] Day11-14: FOC 算法（Clarke/Park/SVPWM/PI 双闭环）

## 目录结构
```
01_Project/TC275_demo/    # 核心工程（只跟踪 _02_CDD 自研代码和 BrsMain.c）
04_Docs/                   # 学习文档（计划书、知识手册）
```

## 自研代码位置
`01_Project/TC275_demo/project/TC275/_02_CDD/`
- LedTest/ - LED + 按键测试
- FOC/ - FOC 算法框架
