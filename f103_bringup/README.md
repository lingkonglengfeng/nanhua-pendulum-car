# F103 四轮车载倒立摆固件

这是本仓库当前使用的固件工程，目标芯片为 STM32F103C8T6。2026-10-20 整理时，`Core/Inc/app_vehicle.h` 中的版本号为 `0x103C8048`。

## 编译入口

- Keil 工程：[`MDK-ARM/f103_bringup.uvprojx`](MDK-ARM/f103_bringup.uvprojx)。
- 编译器：工程记录为 ARM Compiler 5.06 update 5（build 528），未切换到 AC6。
- 设备包：`Keil.STM32F1xx_DFP.2.2.0`。
- CubeMX 工程：[`f103_bringup.ioc`](f103_bringup.ioc)，其中记录的固件包为 STM32Cube FW_F1 V1.8.7。
- 当前存储区配置：Flash 64 KiB，RAM 20 KiB。资源占用应以本次实际构建报告为准。

在 Keil 中编译后，使用 ST-Link 下载，通过 Watch 的 `g_fw_build` 核对版本。重新生成 CubeMX 代码前阅读 [骨架与回归说明](docs/CubeMX骨架与回归说明.md)。

`MDK-ARM/speed_pid_build/` 保存了构建脚本、`verify_*.ps1` 和 `test_balance*.c`。这些检查脚本部分使用 Windows / MSVC，本机构建脚本默认 ARMCC 位于 `C:\Keil_v5\ARM\ARMCC\bin`，换电脑需要调整工具路径。忽略规则保留脚本与测试源文件，排除其生成物和 `before_*` 备份。

## 代码入口

| 文件 | 作用 |
| --- | --- |
| [`Core/Src/app_vehicle.c`](Core/Src/app_vehicle.c) | 应用初始化、200 Hz 控制任务、电机与编码器、轮速环 |
| [`Core/Src/app_motor_dma.inc`](Core/Src/app_motor_dma.inc) | 双向慢衰减 GPIO DMA 波形，被应用源文件包含 |
| [`Core/Src/app_encoder_filter.inc`](Core/Src/app_encoder_filter.inc) | 编码器输入与测速处理，被应用源文件包含 |
| [`Core/Src/as5600.c`](Core/Src/as5600.c) | 摆角传感器通信 |
| [`Core/Src/pendulum_angle.c`](Core/Src/pendulum_angle.c) | 角度处理、最低点标定与角速度 |
| [`Core/Src/balance_control.c`](Core/Src/balance_control.c) | 直立、速度外环、位置保持等控制逻辑 |
| [`Core/Src/balance_swing.inc`](Core/Src/balance_swing.inc) | 自动起摆与接管逻辑 |
| [`Core/Src/balance_motion.inc`](Core/Src/balance_motion.inc) | 直行、位置序列等运动逻辑 |
| [`Core/Src/app_commands.c`](Core/Src/app_commands.c) | 串口命令解析及提交 |
| [`Core/Src/app_telemetry.c`](Core/Src/app_telemetry.c) | 串口 DMA、状态与日志输出 |

## 当前接线与轮位

前后左右以本车定义的车头为准。**电机通道和原始编码器编号是两套编号**，通过 `g_enc_map` 对应。

| 电机通道 | 实际轮位 | 驱动 IN1 | 驱动 IN2 | 原始编码器编号 | 编码器 A / B |
| --- | --- | --- | --- | --- | --- |
| 0 | 右后 | PA8 | PA2 | 0 | PA0 / PA1 |
| 1 | 右前 | PA9 | PA3 | 2 | PA6 / PA7 |
| 2 | 左后 | PA10 | PB14 | 1 | PA4 / PA5 |
| 3 | 左前 | PA11 | PB15 | 3 | PB12 / PB13 |

F103 当前源码默认值：

```c
g_enc_map  = { 0,  2,  1,  3};  // 电机通道 -> 原始编码器
g_mot_sign = {-1, -1,  1,  1};  // 按电机通道排列
g_enc_sign = {-1,  1, -1, -1};  // 按原始编码器编号排列
```

这组方向值属于本车的接线结果，复现时仍需逐轮验证。完整轮位沿革见 [轮位与通道对应](docs/轮位与通道对应.md)。部分早期文档把通道 3 写成右前等旧称，不应据此更改当前轮位表。

| 其他接口 | 引脚 / 配置 |
| --- | --- |
| AS5600 | PB8 SCL、PB9 SDA，软件 I²C |
| 串口日志 / 命令 | USART3，PB10 TX、PB11 RX，115200 / 8N1，无流控 |
| SWD | PA13 SWDIO、PA14 SWCLK |
| 板载 LED | PC13，当前应用用作速度到达指示 |

驱动输出已演进为 TIM1 / TIM4 触发 GPIO DMA，串口使用 DMA，HAL 时基使用 TIM2；不能沿用早期“PWM+静态 DIR”或“TIM2 / TIM4 空闲”的说明。查看 [双向慢衰减实现记录](docs/不改线双向慢衰减PWM.md) 时，其旧轮位名称也应按上表解读。

## 调试顺序

建议先逐轮开环、反馈方向和轮速闭环，再检查摆角，最后进行直立和起摆。源码默认参数来自本车多轮调试，不能直接当作其他机械结构的已验证参数。

1. 连接串口后发送 `STOP`，使用 `AS5600` 查询传感器状态；命令以换行结束。
2. 摆杆自然下垂并保持静止，发送 `ZERO`。标定采集 200 点，约 1 秒；确认回执和标定结果后再继续。
3. 选择手扶顶端捕捉流程或 `SWING` 自动起摆流程。`ZERO` 后存在等待顶端捕捉状态，操作前先读对应说明。
4. 进入直立 ACTIVE 后，再试速度、转向、位置保持或 `BALSTEP`。`STOP` 用于退出驱动。

具体指令和运行条件以 [串口控制指令](docs/串口控制指令.md)、[最低点标定与直立](docs/倒立摆串口控制与最低点标定.md)、[自动起摆](docs/自动起摆控制与调试.md) 和 [顶端接管](docs/自动起摆顶端接管修正.md) 为准。不要将多个启动命令一次发送而跳过中间状态确认。

串口修改的标定与参数主要保存在 RAM，复位后的默认值查看对应源码。v48 的轮速退出配置变更见 [关闭轮速超限退出](docs/关闭轮速超限退出.md)。

## 文档阅读提示

`docs/` 保留了开发期间的版本记录。早期《F103整车调试手册》《F103引脚分配与应用》《FreeRTOS架构说明》中的部分轮位、版本号、资源用途和参数已经变更，适合了解过程，当前使用先看本说明及源码。

完整调试阅读顺序见 [调试过程整理](../docs/调试过程.md)。日志分析文档中有些路径指向本地原始采集目录；批量原始日志默认不提交，文档保留当时分析结论与适用版本。
