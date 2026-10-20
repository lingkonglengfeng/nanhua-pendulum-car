# FreeRTOS 架构说明（v5 固件 · CubeMX 骨架回归版）

> 2026-10-12 起，f103_bringup 与 f407_bringup 均为「CubeMX 生成骨架 + 应用代码」结构：
> 应用逻辑全部位于 USER CODE 区或独立添加的源文件，重新生成代码基本安全——
> 但生成后请过一遍 `docs/CubeMX骨架与回归说明.md` 里的检查清单（3 条）。

## 任务结构

| 任务 | 周期 | 优先级 | 职责 |
|---|---|---|---|
| ControlTask | 5ms（osDelayUntil 绝对周期，200Hz）| High（512 字栈）| AS5600 读角 + 摆角处理 + 零点/符号标定命令 + 编码器增量 + 台架测试/直立环输出 |
| defaultTask | 500ms | Normal（128 字栈）| g_rtos_heartbeat 自增；F103 的 PC13 已改为速度到达指示，由 ControlTask 更新 |
| UartLogTask（F103） | 5ms 服务 RX，默认 50ms 打印 | Low（2048 字节静态栈）| DMA 环形接收、指令解析/回执、CSV 格式化及 TX DMA，合法命令由 ControlTask 周期开始提交 |

## 关键配置

- **HAL 时基**：F407=TIM6 / F103=TIM2（`stm32f4xx_hal_timebase_tim.c` / `stm32f1xx_hal_timebase_tim.c`
  强覆盖 `HAL_InitTick`）
- **中断归属**：SVC/PendSV → FreeRTOS port（FreeRTOSConfig.h 宏映射）；
  SysTick → cmsis_os2.c（`USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=0`）；
  `stm32f4xx_it.c` / `stm32f1xx_it.c` 中对应处理函数已删除（留有注释）
- **编码器 EXTI**（优先级 5 = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY）：
  中断内不调用任何 RTOS API，只做计数 ✓
- **USART3 TX DMA（F103）**：DMA1 通道 2，中断优先级 7，仅完成/错误计数及释放发送缓冲区；独立静态日志任务不占用 FreeRTOS 动态堆。详见 `串口DMA日志.md`。
- **USART3 RX DMA（F103）**：DMA1 通道 3，512 字节循环缓冲区；通道 3 / USART3 错误中断优先级 7，ISR 不解析指令。命令队列 8 条、回执队列 16 条，详见 `串口控制指令.md`。
- **堆**：heap_4；F407 = 24576 B；F103 = 6144 B
  （CubeMX 默认 3072 不够，已在 FreeRTOSConfig.h 的 USER CODE Defines 区覆盖，重生成不丢）
- **时钟**：HSE 8MHz 晶振；F407 = 168MHz（PLLM8 / N336 / P2）、F103 = 72MHz（PLL×9）。
  修复代码在 `App_Init()`（用户区，重生成不丢）；ioc 时钟树已同步。
- **FreeRTOSConfig.h**：CubeMX 标准模板（V10.3.1），设备头经 CMSIS_device_header 宏指向
  stm32f4xx.h / stm32f1xx.h

## 源代码布局（新增文件均为独立 .c 加入 uvprojx，重生成时一般被保留）

| 文件 | 职责 |
|---|---|
| `Core/Src/app_vehicle.c` / `Core/Inc/app_vehicle.h` | 应用层：时钟修复、电机 PWM/DIR、编码器 EXTI、直立环、ControlTask、全部 Watch 变量 |
| `Core/Src/app_telemetry.c` / `Core/Inc/app_telemetry.h` | F103 USART3/DMA、四轮快照、日志任务、CSV 格式化及日志 Watch 变量 |
| `Core/Src/app_commands.c` / `Core/Inc/app_commands.h` | ASCII 行指令解析、参数验证、固定队列、控制周期提交与回执 |
| `Core/Src/as5600.c` / `Core/Inc/as5600.h` | AS5600 位带 I2C（PB8=SCL / PB9=SDA，开漏+上拉，含总线解锁）|
| `Core/Src/pendulum_angle.c` + `Core/Inc/pendulum_angle.h` / `pendulum_angle_config.h` | 摆角处理：零点采集、跨零、角速率低通；标定参数存 config 头 |
| `Core/Src/freertos.c` | 任务创建（USER CODE 区调用 `App_Init()` / `App_ControlTask`）|

## 兼容性说明

- 全部 Watch 变量（g_enc_*、g_pendulum_*、g_bal_*、g_motor_test_*、g_as5600_*）命名与行为
  与 v4 一致，调试流程不变；新增 g_hse_fail / g_clock_hz / g_rtos_heartbeat / g_app_init_done
- 固件版本号：**F407 = 0xF4070005；F103 = 0x103C8010**（Watch 变量 `g_fw_build` 核对）
- 重新生成代码前请阅读 `docs/CubeMX骨架与回归说明.md`（ioc 待办 1 项 + 生成后检查 3 条）

## 源码来源

- F407：FW_F4 V1.28.0 官方包 FreeRTOS V10.3.1（RVDS/ARM_CM4F 移植层）
- F103：CubeF1 V1.8.6 官方包 FreeRTOS V10.3.1（RVDS/ARM_CM3 移植层）+ 补
  freertos_os2.h / freertos_mpool.h（CubeMX 首次生成缺失）
- 应用层逻辑移植自车工程 G474 版（摆角/AS5600 为原样移植，引脚宏改 PB8/PB9）

通道1/2已根据联动日志修正默认反馈映射与编码器2符号，详见 [映射修正](通道1-2反馈映射修正.md)。

从0x103C8010起，已在速度模式时重复RUN 1保持积分历史；从停止/其它驱动模式进入仍重置。详见 [通道1实测](通道1速度环实测.md)。

0x103C8010新增PIDW单轮PID覆盖与W配置行，详见 [单轮PID调试](单轮PID调试.md)。后轮参数不变，前轮试验组尚未固化。
