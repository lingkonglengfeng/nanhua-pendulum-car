# CubeMX 骨架与回归说明（2026-10-12）

## 一、发生了什么

用 CubeMX 对 f103_bringup / f407_bringup 重新生成代码后，v4 的手写应用代码
（原来直接写在 main.c 里）被覆盖，只剩 USER CODE 区；另有一批结构损坏。本轮已全部修复
并重新编译通过（两个工程均 0 错误 0 警告）。损坏与修复对照：

| 现象 | 后果 | 本轮修复 |
|---|---|---|
| f407 `Middlewares/` 整个目录丢失 | uvprojx 里的 FreeRTOS 文件全部找不到 | 从 `STM32Cube_FW_F4_V1.28.0` 官方包整体恢复（V10.3.1）|
| f407 无 freertos.c、main.c 无 RTOS 启动 | 项目没有任务框架 | 新建 freertos.c（与 F103 生成版同构，任务创建在 USER CODE 区）|
| f407 `it.c` 生成了空的 SVC/PendSV/SysTick 处理器 | 与 port.c / cmsis_os2.c 重定义冲突（链接错误）| 删除三个函数（留注释）|
| 两工程 `hal_conf` 的 HAL_TIM_MODULE_ENABLED 被注释 | 时间基文件编译失败 | 取消注释（ioc 已加 TIM，下次重生成会自动开启）|
| f103 CMSIS_RTOS_V2 缺 freertos_os2.h / freertos_mpool.h | cmsis_os2.c 编译失败 | 从 CubeF1 官方包补齐 |
| f103 配置缺 INCLUDE_xTaskGetCurrentTaskHandle | freertos_os2.h 编译期 #error | 在 FreeRTOSConfig.h USER CODE Defines 补 |
| 时钟树仍为 HSI（F103=36MHz / F407=25MHz）| PWM 频率全错、控制周期失真 | 在 `App_Init()` 用 HSE 重设（72M / 168M，失败回退并可 Watch 观察）|

## 二、现在的架构

- 应用代码：`app_vehicle.c/h` + `as5600.c/h` + `pendulum_angle.c/h`（两平台同一份，
  引脚用 `#if defined(STM32F407xx)` 区分）＋ `freertos.c` 的 USER CODE 区接线
- 任务/中断/堆：见 `FreeRTOS架构说明.md`
- 固件版本号：**F407 = 0xF4070005；F103 = 0x103C8005**

## 三、ioc 已同步的修订（本轮已直接改入 .ioc 文件）

**f103_bringup.ioc**
- `FREERTOS.configTOTAL_HEAP_SIZE=6144`
- `RCC.PLLSourceVirtual=RCC_PLLSOURCE_HSE`（72MHz 时钟树，频率缓存值同步）
- TIM1：CH1~CH4 = PWM Generation，Prescaler=0，Period=3599（20kHz）；MX_TIM1_Init 入生成列表

**f407_bringup.ioc**
- 启用 FREERTOS（CMSIS_V2）+ `configTOTAL_HEAP_SIZE=24576`
- 时钟树：PLL 源 HSE、PLLM=8、PLLN=336、PLLP=÷2 → 168MHz（APB1=42M、APB2=84M）
- TIM3：CH3/CH4 = PWM，Period=4199；TIM9：CH1/CH2 = PWM，Period=8399；
  `MX_TIM3_Init` / `MX_TIM9_Init` 入生成列表
- NVIC：PendSV/SVCall/SysTick 改为 RTOS 版格式，补 Saved* 键

> 用 CubeMX 打开这两个 .ioc 时应能正常加载；如遇加载报错，把上述几项在 GUI 里手工重设一遍即可。

## 四、SYS 时基已对齐（2026-10-12 全部完成，无需手工步骤）

两块 ioc 的 SYS 时基已设为 **TIM2（F103）/ TIM6（F407）**
（`VP_SYS_VS_tim2 / VP_SYS_VS_tim6`，键名格式已对照 CubeMX 自带官方样板 ioc 核实）：

- 重新生成时，CubeMX 会**自动**生成不含 SysTick_Handler 的 `it.c`，并重写
  `stm32f1xx/4xx_hal_timebase_tim.c` —— 与工程架构（SysTick 归 cmsis_os2.c）完全一致；
- 时基识别键（`NVIC.TimeBase=... / NVIC.TimeBaseIP=...`）已按 ST 官方样板格式补全——
  这是 GUI 里 SYS 下拉框正确显示 TIM2/TIM6 的关键；
- 过去"重新生成后 SysTick_Handler 重复定义"的风险已消除。
  若极端情况下仍遇到，说明 ioc 被旧状态覆盖：在 CubeMX 的 SYS 里把
  Timebase Source 再选一次 TIM2/TIM6 后重新生成即可。

> 全部键名已逐一对照 CubeMX 自带数据库核实：F1 族用 `RCC.PLLSource`、F4 族用
> `RCC.PLLSourceVirtual`；周期参数两族都用 `Period`（仅 G4/H5 等新族用 `PeriodNoDither`）；
> FreeRTOS 堆用 `FREERTOS.configTOTAL_HEAP_SIZE`。

## 五、重新生成代码后的检查清单（3 条）

1. （已消除）SYS 时基已对齐，正常情况不再出现 SysTick_Handler 重复定义；
2. `stm32f1xx/4xx_hal_conf.h` 若 `HAL_TIM_MODULE_ENABLED` 又被注释 → 取消注释；
3. `main.c` 生成后，主体会自动带上 `osKernelInitialize(); MX_FREERTOS_Init(); osKernelStart();`；
   而在 F407 的 USER CODE 2 里也留着一份旧调用。**两份共存是安全的**：用户区那份先执行，
   调度器启动后不会返回，生成的调用根本不会被抵达。想干净可以删掉用户区那三行，删不删都不影响功能。

应用代码都在 USER CODE 区或独立文件里，正常不会被覆盖；uvprojx 里自加的文件条目
（freertos.c、app_vehicle.c、as5600.c、pendulum_angle.c）如被清掉，按原路径重新添加即可。

4. **生成后检查 `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/`：应包含 `freertos_mpool.h` 和 `freertos_os2.h`**——2026-10-15 出现过一次重新生成后这两个文件丢失的情况（原因未确定，V1.8.6/V1.8.7 官方包都是齐的）。缺了就补：源文件在固件仓库 `D:\STM32Cube\Repository\STM32Cube_FW_F1_V1.8.6\Middlewares\Third_Party\FreeRTOS\Source\CMSIS_RTOS_V2\` （或 V1.8.7 同路径），也可从旧工程同路径直接拷。
   **永久防护（2026-10-16 已做）**：这两个头文件已放入 `Core/Inc/`（CubeMX 不会删除用户文件）——即使以后重新生成又把 Middlewares 里的删了，编译仍能从 Core/Inc 找到，不再受影响。
   注意：固件仓库已移动到 **D:\STM32Cube\Repository**——请确认 CubeMX 的固件包路径设置与之一致。

## 六、烧录与上电验证（Keil Watch 清单）

1. 编译两个工程（当前状态 0 错误 0 警告）→ 烧录
2. Watch 核对顺序：
   - `g_fw_build` = 0xF4070005 / 0x103C8005（确认烧的是新固件）
   - `g_hse_fail` = 0；`g_clock_hz` = 168000000 / 72000000（晶振正常）
   - `g_app_init_done` = 1；`g_rtos_heartbeat` 持续上涨（调度器活着；F103 绿灯闪）
   - `g_as5600_read_ok` 上涨、`read_errors` 不涨；`g_as5600_valid`=1（磁铁对准）
3. 零点标定（流程同 v4）：扶杆直立 → 写 `g_pendulum_zero_request=1` →
   等 `g_pendulum_zero_result=2` → 记下 `g_pendulum_upright_raw`；
   写 `g_pendulum_sign_request=±1` 校正符号（杆顶向车头为正）→
   **把这两个值抄进 `pendulum_angle_config.h` 重新编译**（RAM 里的标定重启即丢）
4. 台架单电机测试：`g_motor_test_wheel`（0=右前 1=左前 2=右后 3=左后）、
   `g_motor_test_pwm_pct`、`g_motor_test_dir`、`g_motor_test_enable=1`。
   ⚠ `dir=1 且 pwm=0` 是全速反转（AT8236 真值表），车必须架空
5. 编码器：手转轮子看 `g_enc_count[i]` 变化；方向不对改 `g_enc_sign[i]`（±1）
6. 直立环：车架好、摆杆装好，先给 `g_bal_kp` / `g_bal_kd` 小值，
   `g_bal_enable=1` 前确认车被约束；`g_bal_fault` 在涨说明摆角无效（检查磁铁/通信）

## 七、本轮的取舍说明

- 时钟修复放在 `App_Init()`（USER CODE）而不是改 SystemClock_Config 函数体——
  这样即使以后 ioc 时钟树再被别的生成流程覆盖，代码路径仍然正确；ioc 修好后本修复
  与生成配置一致（幂等，可保留）。
- HAL 时基沿用 v4 方案（TIM2/TIM6 文件法）而不是 CubeMX 新版默认的「SysTick 合并法」——
  与车工程一致，且避免修改更多生成文件。
- 编码器 EXTI、PWM、DIR 全部用寄存器级实现（不依赖 HAL_TIM 模块），
  这样 hal_conf 里模块开关的变动不会影响应用编译。
