# CubeMX 一步一步配置手册（f103_bringup 版）

> 用途：① 你想在 CubeMX 里把这份工程从头重配一遍时照做；② 排查配置问题时对照。
> 注意：**现有 `f103_bringup.ioc` 已经配置完毕**，日常直接用即可；本手册是"如果重来一遍"的完整步骤。
> 所有参数与 `Core/Src/app_vehicle.c`、`as5600.c`、`freertos.c` 里的代码严格对应。

---

## 第 0 步：新建工程

1. CubeMX → **File → New Project**
2. 搜索 `STM32F103C8` → 选 **STM32F103C8Tx（LQFP48）** → Start Project

## 第 1 步：时钟源（RCC）

**System Core → RCC**
- High Speed Clock (HSE)：选 **Crystal/Ceramic Resonator**（外部 8MHz 晶振）
- 效果：PD0/PD1 自动变成 OSC_IN/OSC_OUT，不用手动动

## 第 2 步：SYS（关键！）

**System Core → SYS**
- Debug：**Serial Wire**（保留 SWD 下载）
- **Timebase Source：选 TIM2**（⚠ 不选会生成和 FreeRTOS 冲突的 SysTick 代码，这是整份配置里最重要的一条）

## 第 3 步：逐个引脚配置（点芯片图上的引脚选择）

| 引脚 | 配置项 | 参数 | 用途 |
|---|---|---|---|
| PA8 / PA9 / PA10 / PA11 | 在第 4 步配 TIM1 时选通道，自动分配 | — | 四路 PWM |
| PA2 / PA3 / PB14 / PB15 | GPIO_Output | Push Pull、No pull-up-down、Low speed、初始 **Low** | 四路 DIR（低=前进）|
| PA0 / PA1 / PA4 / PA5 / PA6 / PA7 / PB12 / PB13 | GPIO_Input | **Pull-up** | 四路编码器 A/B（EXTI 中断由代码里做）|
| PB8 / PB9 | GPIO_Output | **Open Drain**、No pull-up-down、Low speed、初始 **High** | AS5600 位带 I2C（SCL/SDA）|
| PC13 | GPIO_Output | Push Pull、No pull、Low speed、初始 **High** | 心跳 LED（低电平点亮）|
| PA13 / PA14 | 第 2 步选 Serial Wire 后自动配好 | — | SWD 调试 |

> 提示：输出开漏（PB8/PB9）的设法是——选择 GPIO_Output 后，在下方的 GPIO 面板里把
> **GPIO output level = High、GPIO mode = Output Open Drain、Maximum output speed = Low**。

## 第 4 步：TIM1 四路 PWM

**Timers → TIM1**
- Clock Source：**Internal Clock**
- Channel1 → **PWM Generation CH1**；Channel2/3/4 → 同样选 PWM Generation CH2/CH3/CH4
- 下方 Parameter Settings（每个通道标签页相同）：
  - **Prescaler = 0**
  - **Counter Period = 3599**（→ 72MHz/(3599+1) = 20kHz）
  - PWM Mode 1、Pulse = 0（默认）
- 不用动 Break/死区（代码运行时会开 MOE）

## 第 5 步：时钟树

**Clock Configuration 页**
- Input frequency = **8 MHz**
- PLL Source Mux = **HSE**
- PLLMul = **×9**
- System Clock Mux = **PLLCLK**（HCLK 应显示 **72 MHz**）
- AHB Prescaler = /1
- **APB1 Prescaler = /2**（36 MHz）；**APB2 Prescaler = /1**（72 MHz）
- （可选）ADC Prescaler 改成 /6：消除 RCC 上的黄色警告（ADC 本工程不用，纯提示）

## 第 6 步：FreeRTOS

**Middleware and Software Packs → FREERTOS**
- Interface：**CMSIS_V2**
- **Config parameters 页 → MEMORY MANAGEMENT settings → TOTAL_HEAP_SIZE = 6144**（Bytes）
  （默认 3072 不够用；其余参数保持默认）
- **Tasks and Queues 页**：保留自带的 defaultTask 不动（Priority = osPriorityNormal、
  Stack Size = 128 Words、Entry = StartDefaultTask、Allocation = Dynamic）
  ——**不用在这里加 ControlTask**，控制任务是在 `freertos.c` 的 USER CODE 区里创建的
- Advanced settings：默认

## 第 7 步：NVIC（一般不用动）

**System Core → NVIC**：Priority Group = 4 bits；其余默认（PendSV/SysTick = 15，
硬件错误全使能）。**编码器 EXTI 的优先级是代码里设的**（优先级 5），不用在这里配。

## 第 8 步：Project Manager

**Project Manager 页**
- Project Name = `f103_bringup`；Location 选你的目录
- Toolchain/IDE = **MDK-ARM**（V5 系列，对应你的 Keil）
- Firmware Package = STM32Cube FW_F1 V1.8.6

**Code Generator 子页**
- ☑ Copy all used libraries into the project folder
- ☑ **Keep User Code when re-generating**（必须，默认就勾着）
- ☑ Delete previously generated files when re-generating
- ☐ Generate peripheral initialization as a pair of .c/.h files per peripheral（不勾）

## 第 9 步：生成

点 **GENERATE CODE** → 完成后点 Open Project（用 Keil 打开）。

---

## 生成后必查清单（4 条，1 分钟）

1. `Core/Src/stm32f1xx_it.c` 里**没有** `SysTick_Handler`（因为时基=TIM2，有的话说明第 2 步没配对）
2. `Core/Src/stm32f1xx_hal_timebase_tim.c` 文件存在（TIM2 时基文件）
3. `Core/Inc/FreeRTOSConfig.h` 里 `configTOTAL_HEAP_SIZE` = **6144**
4. Keil 工程（uvprojx）里这 4 个文件还在：`freertos.c`、`app_vehicle.c`、`as5600.c`、
   `pendulum_angle.c`（丢了就右键 Add Existing Files 重新加回，都在 Core/Src 下）

编译应 **0 错 0 警**；烧录后 Watch 核对：`g_hse_fail=0`、`g_clock_hz=72000000`、
`g_app_init_done=1`。

## "故意不在 CubeMX 里配"的东西（免得你找不到而困惑）

| 项目 | 为什么不在 CubeMX 里配 |
|---|---|
| 编码器 EXTI（PA0/PA4/PA6/PB12 四条中断线）| 代码用寄存器直接配（`app_vehicle.c` 的 `s_enc_gpio_exti_init`）|
| PWM 的使能、CCR 占空比 | 代码直接写寄存器；CubeMX 只负责 PSC/ARR/模式框架 |
| AS5600 的 I2C | 软件位带 I2C（PB8/PB9 普通开漏 IO），不占硬件 I2C |
| DIR 方向翻转逻辑、心跳灯闪烁 | 代码控制（`app_vehicle.c` / `freertos.c`）|

## 附录：F407（f407_bringup）的差异对照

| 项目 | F103 | F407 |
|---|---|---|
| SYS Timebase | TIM2 | **TIM6** |
| 时钟树 | HSE 8M ×9 = 72M；APB1 /2、APB2 /1 | HSE 8M；PLLM=8、PLLN=336、PLLP=/2 = **168M**；APB1 /4=42M、APB2 /2=84M |
| PWM | TIM1 CH1~4 = PA8~PA11，Period=3599 | TIM3 CH3/CH4 = PB0/PB1，Period=4199；TIM9 CH1/CH2 = PE5/PE6，Period=8399 |
| DIR | PA2/PA3/PB14/PB15 | PB6/PB7/PB10/PB11 |
| 编码器 | PA0/1、PA4/5、PA6/7、PB12/13 | PA4/5、PE2/3、PD6/7、PC7/12 |
| I2C（AS5600）| PB8/PB9（同）| PB8/PB9（同）|
| LED | PC13 | 无（defaultTask 只做心跳计数）|
| FreeRTOS 堆 | 6144 | 24576 |

> 本手册配套文档：`FreeRTOS架构说明.md`、`CubeMX骨架与回归说明.md`
