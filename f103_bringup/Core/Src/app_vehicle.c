/**
  ******************************************************************************
  * @file    app_vehicle.c
  * @brief   车载倒立摆应用层实现（F407 探索者 / F103C8T6 两平台共用）
  *
  *   - App_Init()        ：时钟修复(HSE→72/168MHz) + GPIO + PWM/DIR + 编码器 EXTI
  *                          + AS5600 位带 I2C + 摆角模块（调度器启动前调用）
  *   - App_ControlTask() ：200Hz（5ms）——AS5600 读角 → 摆角处理 → 零点/符号命令
  *                          → 编码器增量 → 单电机台架测试 / 直立环 PD / 滑行
  *
  *   中断：仅编码器 A 相 EXTI（双边沿），中断内只做计数，不调用任何 RTOS API；
  *         EXTI 优先级 = 5（configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY）。
  *   HAL 时基：F407=TIM6 / F103=TIM2（stm32f4xx/1xx_hal_timebase_tim.c）。
  *   本车F103最新实测轮位：电机0=右后、1=右前、2=左后、3=左前。
  ******************************************************************************
  */
#include "app_vehicle.h"
#include "as5600.h"
#include "pendulum_angle.h"
#include "cmsis_os.h"
#if defined(STM32F103xB)
#include "FreeRTOS.h"
#include "task.h"
#include "app_telemetry.h"
#include "app_commands.h"
#endif

/* CubeMX 生成的 main.h 不声明时钟函数原型，这里补充（HSE 起振失败时回退用） */
void SystemClock_Config(void);

/* ======================== Watch 变量定义 ======================== */

volatile uint32_t g_fw_build = APP_FW_BUILD;
volatile uint32_t g_hse_fail = 0U;
volatile uint32_t g_clock_hz = 0U;
volatile uint32_t g_rtos_heartbeat = 0U;
volatile uint8_t  g_app_init_done = 0U;

volatile uint16_t g_as5600_raw_angle = 0U;
volatile uint16_t g_as5600_raw_filt = 0U;
volatile uint32_t g_as5600_angle_mdeg = 0U;
volatile uint8_t  g_as5600_status = 0U;
volatile uint8_t  g_as5600_valid = 0U;
volatile uint32_t g_as5600_read_ok = 0U;
volatile uint32_t g_as5600_read_errors = 0U;
volatile uint32_t g_as5600_timeout_count = 0U;
volatile uint32_t g_as5600_last_sample_ms = 0U;
volatile uint32_t g_as5600_last_valid_ms = 0U;
volatile uint32_t g_as5600_read_time_ms = 0U;

volatile float    g_pendulum_angle_rad = 0.0f;
volatile float    g_pendulum_rate_rad_s = 0.0f;
volatile uint8_t  g_pendulum_angle_valid = 0U;
volatile uint8_t  g_pendulum_calibrated = 0U;
volatile uint16_t g_pendulum_upright_raw = PENDULUM_UPRIGHT_RAW;
volatile uint16_t g_pendulum_bottom_raw = 0U;
volatile float g_pendulum_bottom_deg = 0.0f;
volatile int8_t   g_pendulum_angle_sign = PENDULUM_ANGLE_SIGN;
volatile uint32_t g_pendulum_zero_result = PENDULUM_ZERO_IDLE;
volatile uint16_t g_pendulum_zero_samples = 0U;
volatile uint8_t  g_pendulum_zero_request = 0U;
volatile int8_t   g_pendulum_sign_request = 0;
volatile uint32_t g_pendulum_command_errors = 0U;

volatile uint32_t g_pendulum_sample_count = 0U;
volatile uint32_t g_pendulum_cycle_time_ms = 0U;
volatile uint32_t g_pendulum_max_cycle_time_ms = 0U;
volatile uint32_t g_pendulum_late_wake_count = 0U;
volatile uint32_t g_pendulum_missed_periods = 0U;
volatile uint32_t g_pendulum_delay_errors = 0U;
volatile uint32_t g_pendulum_stale_events = 0U;

volatile int32_t  g_enc_count[APP_WHEEL_COUNT] = {0, 0, 0, 0};
volatile int32_t  g_enc_delta[APP_WHEEL_COUNT] = {0, 0, 0, 0};
/* 编码器保持原始引脚顺序；电机1对应编码器2、电机2对应编码器1，符号随电机同步修正。 */
#if defined(STM32F103xB)
volatile int8_t   g_enc_sign[APP_WHEEL_COUNT] = {-1, 1, -1, -1};
#else
volatile int8_t   g_enc_sign[APP_WHEEL_COUNT] = {-1, 1, -1, -1};
#endif

volatile float    g_bal_u[APP_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile uint16_t g_bal_ccr[APP_WHEEL_COUNT] = {0U, 0U, 0U, 0U};
volatile uint8_t  g_bal_dir[APP_WHEEL_COUNT] = {0U, 0U, 0U, 0U};
/* F103 v19: 用户确认轮位0右后/1右前/2左后/3左前，相对v18反向0和3。 */
#if defined(STM32F103xB)
volatile int8_t   g_mot_sign[APP_WHEEL_COUNT] = {-1, -1, 1, 1};
#else
volatile int8_t   g_mot_sign[APP_WHEEL_COUNT] = {-1, 1, 1, 1};
#endif

volatile uint8_t  g_motor_test_enable = 0U;
volatile uint8_t  g_motor_test_wheel = 0U;
volatile uint8_t  g_motor_test_pwm_pct = 0U;
volatile uint8_t  g_motor_test_dir = 0U;
volatile uint16_t g_motor_test_ccr = 0U;

volatile int16_t  g_drive_test = 0;   /* 四轮开环直行测试：-100..100 */
volatile float    g_enc_speed[APP_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
/* Mapped motor order, 20ms LP of low-speed period/high-speed count estimate. */
volatile float    g_speed_estimated_rpm[APP_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float    g_speed_target[APP_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float    g_enc_counts_per_rev = APP_ENCODER_COUNTS_PER_REV;
volatile uint8_t  g_speed_enable = 0U;
/* Kp 按 RPM 量纲换算；Ki 为通道0架空 30/60/100RPM 实测基础值。
 * 作为共用基础值；前轮可用 PIDW 单独覆盖，带负载表现仍需分别验证。 */
volatile float    g_spd_kp = 0.0002f * APP_ENCODER_COUNTS_PER_REV / 60.0f;
volatile float    g_spd_ki = 0.01f;
volatile float    g_spd_kd = 0.0f;
volatile uint8_t  g_spd_individual[APP_WHEEL_COUNT] = {0U, 0U, 0U, 0U};
volatile float    g_spd_kp_w[APP_WHEEL_COUNT] = {0.001562f, 0.001562f, 0.001562f, 0.001562f};
volatile float    g_spd_ki_w[APP_WHEEL_COUNT] = {0.01f, 0.01f, 0.01f, 0.01f};
volatile float    g_spd_kd_w[APP_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float    g_spd_kp_used[APP_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float    g_spd_ki_used[APP_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float    g_spd_kd_used[APP_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float    g_speed_current[APP_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float    g_speed_fast[APP_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
#if defined(STM32F103xB)
/* Front-wheel feedback trial; rear loops retain their original measurement. */
volatile uint32_t g_speed_filter_ms[APP_WHEEL_COUNT] = {0U, 60U, 0U, 60U};
#else
volatile uint32_t g_speed_filter_ms[APP_WHEEL_COUNT] = {0U, 0U, 0U, 0U};
#endif
volatile uint32_t g_speed_filter_used_ms[APP_WHEEL_COUNT] = {0U, 0U, 0U, 0U};
volatile float    g_speed_error[APP_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float    g_spd_integral[APP_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile uint8_t  g_speed_reset = 0U;
volatile uint8_t  g_speed_active = 0U;
volatile uint32_t g_speed_dt_ms = PENDULUM_SAMPLE_PERIOD_MS;
volatile uint8_t  g_speed_led_wheel = 0U;
volatile float    g_speed_tolerance_pct = 5.0f;
volatile uint8_t  g_speed_reached = 0U;
volatile float    g_spd_u[APP_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
#if defined(STM32F103xB)
/* 本车日志确认：PWM通道1对应编码器2，PWM通道2对应编码器1。 */
volatile int8_t   g_enc_map[APP_WHEEL_COUNT] = {0, 2, 1, 3};
#else
volatile int8_t   g_enc_map[APP_WHEEL_COUNT] = {0, 1, 2, 3};
#endif

/* ======================== 内部状态 ======================== */

static PendulumAngle s_pendulum;
static int32_t s_enc_last[APP_WHEEL_COUNT];
static float   s_spd_prev_measured[APP_WHEEL_COUNT];
static uint32_t s_speed_last_sample_ms;
static uint8_t s_speed_sample_ready;

/* 每轮 PWM 的 ARR（F407: TIM3=4199, TIM9=8399；F103: TIM1=3599） */
#if defined(STM32F407xx)
static uint32_t s_motor_arr[APP_WHEEL_COUNT];
#endif

/* ControlTask only. Measurement keeps updating in STOP/OPEN too, so enabling
 * PID does not discard the observed motion or create a filter restart step. */
static void s_speed_feedback(uint8_t wheel, float fast_rpm, float dt)
{
  uint32_t tau_ms = g_speed_filter_ms[wheel];
  if (tau_ms > 200U) { tau_ms = 200U; }
  g_speed_filter_used_ms[wheel] = tau_ms;
  g_speed_fast[wheel] = fast_rpm;
  if (tau_ms == 0U)
  {
    g_speed_current[wheel] = fast_rpm;
  }
  else
  {
    float alpha = dt / ((float)tau_ms * 0.001f + dt);
    g_speed_current[wheel] += alpha * (fast_rpm - g_speed_current[wheel]);
  }
}

void App_ResetSpeedWheel(uint8_t wheel)
{
  if (wheel < APP_WHEEL_COUNT)
  {
    g_spd_integral[wheel] = 0.0f;
    s_spd_prev_measured[wheel] = g_speed_current[wheel];
    g_spd_u[wheel] = 0.0f;
  }
}

static void s_speed_pid_select(void)
{
  uint8_t i;
  float kp = g_spd_kp;
  float ki = g_spd_ki;
  float kd = g_spd_kd;
  for (i = 0U; i < APP_WHEEL_COUNT; i++)
  {
    uint8_t individual = g_spd_individual[i];
    g_spd_kp_used[i] = (individual != 0U) ? g_spd_kp_w[i] : kp;
    g_spd_ki_used[i] = (individual != 0U) ? g_spd_ki_w[i] : ki;
    g_spd_kd_used[i] = (individual != 0U) ? g_spd_kd_w[i] : kd;
  }
}

/* ======================== 滑动中值滤波（窗口 3） ========================
 * 用途：滤掉 AS5600 原始角度的偶发尖刺（干扰毛刺/单次读错）。
 * 环绕安全：先把窗口样本相对首样本展开成 ±2048 的有符号差值再排序取中值，
 *           避免样本跨 4095->0 边界时中值取错（例如 4090、4095、2 这类窗口）。
 * 代价：窗口 3 @200Hz，单调变化时约滞后 1 个采样周期（~5ms）。 */
#define MEDIAN_WIN 3U
static uint16_t s_med_buf[MEDIAN_WIN];
static uint8_t  s_med_idx = 0U;
static uint8_t  s_med_count = 0U;

static uint16_t median3_circular(uint16_t new_sample)
{
  int32_t diff[MEDIAN_WIN];
  int32_t ref;
  int32_t med;
  uint8_t i;

  /* 入窗（环形缓冲） */
  s_med_buf[s_med_idx] = new_sample;
  s_med_idx = (uint8_t)((s_med_idx + 1U) % MEDIAN_WIN);
  if (s_med_count < MEDIAN_WIN)
  {
    s_med_count++;
  }

  /* 以窗口首样本为参考，展开为有符号差值（处理 4095->0 环绕） */
  ref = (int32_t)s_med_buf[0];
  for (i = 0U; i < s_med_count; i++)
  {
    int32_t d = (int32_t)s_med_buf[i] - ref;
    if (d > 2047)  { d -= 4096; }
    if (d < -2048) { d += 4096; }
    diff[i] = d;
  }

  /* 插入排序（最多 3 个元素） */
  for (i = 1U; i < s_med_count; i++)
  {
    int32_t t = diff[i];
    uint8_t j = i;
    while ((j > 0U) && (diff[j - 1U] > t))
    {
      diff[j] = diff[j - 1U];
      j--;
    }
    diff[j] = t;
  }

  /* 取中值并还原到 0..4095 */
  med = ref + diff[s_med_count / 2U];
  if (med < 0)    { med += 4096; }
  if (med > 4095) { med -= 4096; }
  return (uint16_t)med;
}

/* ======================== 时钟修复 ========================
 * CubeMX 生成的 SystemClock_Config 目前是 HSI 源（F103=36MHz / F407=25MHz，
 * 因 ioc 时钟树未改）。这里在调度器启动前整体改用 HSE：
 *   F103: HSE 8MHz × 9                 = 72MHz（APB1 /2 = 36MHz, APB2 /1）
 *   F407: HSE 8MHz /8 ×336 /2          = 168MHz（APB1 /4 = 42MHz, APB2 /2 = 84MHz）
 * 失败（无晶振/起振超时）则 g_hse_fail=1 并回退到 CubeMX 生成配置继续运行。
 * ioc 时钟树改好后，本函数与生成配置结果一致，可保留（幂等）。
 * ============================================================ */
static void s_clock_init(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  uint32_t guard;

  /* CubeMX 生成的配置以上电默认的 PLL 作为系统时钟源；而 F1/F4 的
   * HAL_RCC_OscConfig 在“PLL 正被用作系统时钟”时会拒绝任何不同的 PLL 配置
   * （直接返回 HAL_ERROR，不会自动切到 HSI 绕行）。因此在改 PLL 源之前，
   * 先把 SYSCLK 切回 HSI，改完再由 HAL_RCC_ClockConfig 切回 PLL。
   * 缺了这一步，HSE+PLL 的请求会被误判为“HSE 起振失败”。 */
  RCC->CFGR &= ~RCC_CFGR_SW;                 /* SW=00 → HSI */
  guard = 1000000U;
  while (((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) && (--guard != 0U))
  {
  }

#if defined(STM32F407xx)
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 8U;
  osc.PLL.PLLN = 336U;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 7U;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    g_hse_fail = 1U;
    SystemClock_Config();          /* 回退：CubeMX 生成配置（HSI 25MHz） */
    g_clock_hz = HAL_RCC_GetHCLKFreq();
    return;
  }
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV4;
  clk.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK)
  {
    g_hse_fail = 2U;   /* 2 = PLL 配置成功但切换系统时钟失败 */
    SystemClock_Config();
  }
#else
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    g_hse_fail = 1U;
    SystemClock_Config();          /* 回退：CubeMX 生成配置（HSI 36MHz） */
    g_clock_hz = HAL_RCC_GetHCLKFreq();
    return;
  }
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
  {
    g_hse_fail = 2U;   /* 2 = PLL 配置成功但切换系统时钟失败 */
    SystemClock_Config();
  }
#endif
  g_clock_hz = HAL_RCC_GetHCLKFreq();
}

/* ======================== 电机 PWM/DIR（寄存器级） ======================== */

static void s_motor_gpio_init(void)
{
  GPIO_InitTypeDef io = {0};

#if defined(STM32F407xx)
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* PWM：TIM3_CH3=PB0（右前）、TIM3_CH4=PB1（左前） */
  io.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  io.Mode = GPIO_MODE_AF_PP;
  io.Pull = GPIO_NOPULL;
  io.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  io.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &io);

  /* PWM：TIM9_CH1=PE5（右后）、TIM9_CH2=PE6（左后） */
  io.Pin = GPIO_PIN_5 | GPIO_PIN_6;
  io.Alternate = GPIO_AF3_TIM9;
  HAL_GPIO_Init(GPIOE, &io);

  /* DIR：PB6/PB7/PB10/PB11（右前/左前/右后/左后），初始低=前进 */
  io.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_11;
  io.Mode = GPIO_MODE_OUTPUT_PP;
  io.Pull = GPIO_NOPULL;
  io.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &io);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);
#else
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* IN1 and IN2 are ordinary GPIO, driven by synchronized DMA. */
  io.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
  io.Mode = GPIO_MODE_OUTPUT_PP;
  io.Pull = GPIO_NOPULL;
  io.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &io);

  /* DIR：PA2/PA3（电机0/1）、PB14/PB15（电机2/3），物理方向由g_mot_sign校准 */
  io.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  io.Mode = GPIO_MODE_OUTPUT_PP;
  io.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &io);
  io.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &io);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
#endif
}

#if defined(STM32F103xB)
#include "app_motor_dma.inc"
#endif

static void s_motor_tim_init(void)
{
#if defined(STM32F407xx)
  /* TIM3（APB1，定时器时钟 84MHz）：20kHz = 84M / 4200 */
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->PSC = 0U;
  TIM3->ARR = 4199U;
  TIM3->CCMR2 = (6U << 4) | (1U << 3) | (6U << 12) | (1U << 11);  /* OC3/OC4: PWM1 + 预装载 */
  TIM3->CCER = (1U << 8) | (1U << 12);                            /* CC3E / CC4E */
  TIM3->CR1 = (1U << 7) | (1U << 0);                              /* ARPE + CEN */
  TIM3->EGR = 1U;                                                 /* 装载影子寄存器 */
  s_motor_arr[0] = 4199U;   /* 右前 TIM3_CH3 / PB0 */
  s_motor_arr[1] = 4199U;   /* 左前 TIM3_CH4 / PB1 */

  /* TIM9（APB2，定时器时钟 168MHz）：20kHz = 168M / 8400 */
  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
  TIM9->PSC = 0U;
  TIM9->ARR = 8399U;
  TIM9->CCMR1 = (6U << 4) | (1U << 3) | (6U << 12) | (1U << 11);  /* OC1/OC2 */
  TIM9->CCER = (1U << 0) | (1U << 4);                             /* CC1E / CC2E */
  TIM9->CR1 = (1U << 7) | (1U << 0);
  TIM9->EGR = 1U;
  s_motor_arr[2] = 8399U;   /* 右后 TIM9_CH1 / PE5 */
  s_motor_arr[3] = 8399U;   /* 左后 TIM9_CH2 / PE6 */
#else
  s_motor_dma_init();
#endif
}

#if defined(STM32F407xx)
static void s_motor_pwm_set(uint8_t wheel, uint32_t ccr)
{
#if defined(STM32F407xx)
  switch (wheel)
  {
    case 0U: TIM3->CCR3 = ccr; break;   /* 右前 PB0 */
    case 1U: TIM3->CCR4 = ccr; break;   /* 左前 PB1 */
    case 2U: TIM9->CCR1 = ccr; break;   /* 右后 PE5 */
    default: TIM9->CCR2 = ccr; break;   /* 左后 PE6 */
  }
#else
  switch (wheel)
  {
    case 0U: TIM1->CCR1 = ccr; break;   /* 右后 PA8 */
    case 1U: TIM1->CCR2 = ccr; break;   /* 右前 PA9 */
    case 2U: TIM1->CCR3 = ccr; break;   /* 左后 PA10 */
    default: TIM1->CCR4 = ccr; break;   /* 左前 PA11 */
  }
#endif
}

static void s_motor_dir_set(uint8_t wheel, uint8_t high)
{
#if defined(STM32F407xx)
  switch (wheel)
  {
    case 0U: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, high ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
    case 1U: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, high ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
    case 2U: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, high ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
    default: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, high ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
  }
#else
  switch (wheel)
  {
    case 0U: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, high ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
    case 1U: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, high ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
    case 2U: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, high ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
    default: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, high ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
  }
#endif
}

#endif

/* 归一化控制量 u∈[-1,1] → AT8236 真值表：
 *   u>0 ：IN2(DIR)=0（前进），PWM = u
 *   u<0 ：IN2=1，PWM = 1-|u|（|u|=1 时 CCR=0 → 全速反转）
 *   u=0 ：(0,0) 滑行 */
static void s_motor_apply(uint8_t wheel, float u)
{
#if defined(STM32F103xB)
  u *= (float)g_mot_sign[wheel];
  if (u > 1.0f) { u = 1.0f; }
  if (u < -1.0f) { u = -1.0f; }
  s_motor_requested[wheel] = u;
  g_bal_u[wheel] = u;
#else
  uint32_t arr = s_motor_arr[wheel];
  uint32_t ccr;
  uint8_t dir;

  u *= (float)g_mot_sign[wheel];
  if (u > 1.0f)  u = 1.0f;
  if (u < -1.0f) u = -1.0f;

  if (u > 0.0f)
  {
    dir = 0U;
    ccr = (uint32_t)(u * (float)arr + 0.5f);
  }
  else if (u < 0.0f)
  {
    dir = 1U;
    ccr = (uint32_t)((1.0f + u) * (float)arr + 0.5f);
  }
  else
  {
    dir = 0U;
    ccr = 0U;
  }
  if (ccr > arr) ccr = arr;

  s_motor_pwm_set(wheel, ccr);
  s_motor_dir_set(wheel, dir);
  g_bal_u[wheel] = u;
  g_bal_ccr[wheel] = (uint16_t)ccr;
  g_bal_dir[wheel] = dir;
#endif
}

static void s_motor_all_coast(void)
{
  uint8_t i;
  for (i = 0U; i < APP_WHEEL_COUNT; i++)
  {
    s_motor_apply(i, 0.0f);
  }
}

/* PC13 低电平点亮：仅在速度环实际工作、选中轮接近非零目标时亮。 */
static void s_speed_led_update(void)
{
  uint8_t wheel = g_speed_led_wheel;
  uint8_t reached = 0U;

  if ((g_speed_active != 0U) && (wheel < APP_WHEEL_COUNT))
  {
    float target = (float)g_speed_target[wheel];
    float abs_target = (target < 0.0f) ? -target : target;
    float error = target - g_speed_current[wheel];
    float abs_error = (error < 0.0f) ? -error : error;
    float tolerance = abs_target * g_speed_tolerance_pct * 0.01f;

    if ((target != 0.0f) && (abs_error <= tolerance))
    {
      reached = 1U;
    }
  }
  g_speed_reached = reached;
#if defined(STM32F103xB)
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,
                    (reached != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

/* ======================== 编码器 EXTI（软件正交解码） ========================
 * 仅 A 相进中断（双边沿）：边沿后读 A/B 电平，a!=b 记 +1，否则 -1。
 * EXTI 线号 = 引脚号，四个 A 相落在互不相同的线上。 */

#if defined(STM32F103xB)
#include "app_encoder_filter.inc"
static void s_enc_filter_flush_ready(void)
{
  uint32_t port_a, port_b, pending, now_cycles;
  /* EXTI priority 5 is masked by the existing FreeRTOS critical section. */
  taskENTER_CRITICAL();
  port_a=GPIOA->IDR; port_b=GPIOB->IDR;
  pending=EXTI->PR; now_cycles=DWT->CYCCNT;
  if(!(pending&EXTI_PR_PR0)) s_enc_filter_flush(0U,(port_a&GPIO_PIN_0)?1U:0U,now_cycles);
  if(!(pending&EXTI_PR_PR4)) s_enc_filter_flush(1U,(port_a&GPIO_PIN_4)?1U:0U,now_cycles);
  if(!(pending&EXTI_PR_PR6)) s_enc_filter_flush(2U,(port_a&GPIO_PIN_6)?1U:0U,now_cycles);
  if(!(pending&EXTI_PR_PR12)) s_enc_filter_flush(3U,(port_b&GPIO_PIN_12)?1U:0U,now_cycles);
  taskEXIT_CRITICAL();
}

static float s_enc_speed_read(uint8_t wheel, float count_rate)
{
  uint32_t period, age, clock_hz=SystemCoreClock;
  int8_t direction;
  taskENTER_CRITICAL();
  s_enc_speed_snapshot(wheel,DWT->CYCCNT,clock_hz/4U,&period,&age,&direction);
  taskEXIT_CRITICAL();
  return s_enc_speed_hybrid(count_rate,period,age,direction,clock_hz);
}
#endif

static void s_enc_gpio_exti_init(void)
{
  GPIO_InitTypeDef io = {0};

  io.Mode = GPIO_MODE_INPUT;
  io.Pull = GPIO_PULLUP;
  io.Speed = GPIO_SPEED_FREQ_LOW;

#if defined(STM32F407xx)
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  io.Pin = GPIO_PIN_4 | GPIO_PIN_5;                     /* w0 右前 PA4(A)/PA5(B) */
  HAL_GPIO_Init(GPIOA, &io);
  io.Pin = GPIO_PIN_2 | GPIO_PIN_3;                     /* w1 左前 PE2(A)/PE3(B) */
  HAL_GPIO_Init(GPIOE, &io);
  io.Pin = GPIO_PIN_6 | GPIO_PIN_7;                     /* w2 右后 PD6(A)/PD7(B) */
  HAL_GPIO_Init(GPIOD, &io);
  io.Pin = GPIO_PIN_7 | GPIO_PIN_12;                    /* w3 左后 PC7(A)/PC12(B) */
  HAL_GPIO_Init(GPIOC, &io);

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  /* SYSCFG->EXTICR：线 2→端口E(4)，线 4→端口A(0)，线 6→端口D(3)，线 7→端口C(2) */
  SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~(0xFUL << 8))  | (0x4UL << 8);
  SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~(0xFUL << 0))  | (0x0UL << 0);
  SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~(0xFUL << 8))  | (0x3UL << 8);
  SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~(0xFUL << 12)) | (0x2UL << 12);

  EXTI->IMR  |= (1UL << 2) | (1UL << 4) | (1UL << 6) | (1UL << 7);
  EXTI->RTSR |= (1UL << 2) | (1UL << 4) | (1UL << 6) | (1UL << 7);
  EXTI->FTSR |= (1UL << 2) | (1UL << 4) | (1UL << 6) | (1UL << 7);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#else
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  io.Pin = GPIO_PIN_0 | GPIO_PIN_1;                     /* enc0 右后 PA0(A)/PA1(B) */
  HAL_GPIO_Init(GPIOA, &io);
  io.Pin = GPIO_PIN_4 | GPIO_PIN_5;                     /* enc1 左后 PA4(A)/PA5(B) */
  HAL_GPIO_Init(GPIOA, &io);
  io.Pin = GPIO_PIN_6 | GPIO_PIN_7;                     /* enc2 右前 PA6(A)/PA7(B) */
  HAL_GPIO_Init(GPIOA, &io);
  io.Pin = GPIO_PIN_12 | GPIO_PIN_13;                   /* enc3 左前 PB12(A)/PB13(B) */
  HAL_GPIO_Init(GPIOB, &io);

  {
    uint8_t levels[APP_WHEEL_COUNT];
    uint32_t port_a=GPIOA->IDR, port_b=GPIOB->IDR;
    levels[0]=(port_a&GPIO_PIN_0) ? 1U : 0U;
    levels[1]=(port_a&GPIO_PIN_4) ? 1U : 0U;
    levels[2]=(port_a&GPIO_PIN_6) ? 1U : 0U;
    levels[3]=(port_b&GPIO_PIN_12) ? 1U : 0U;
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    s_enc_filter_init(levels,(SystemCoreClock/1000000U)*APP_ENCODER_QUALIFY_US);
  }

  __HAL_RCC_AFIO_CLK_ENABLE();
  /* AFIO->EXTICR：线 0/4/6→端口A(0)，线 12→端口B(1) */
  AFIO->EXTICR[0] = (AFIO->EXTICR[0] & ~0xFUL) | 0x0UL;
  AFIO->EXTICR[1] = (AFIO->EXTICR[1] & ~0xFUL) | 0x0UL;
  AFIO->EXTICR[1] = (AFIO->EXTICR[1] & ~(0xFUL << 8)) | 0x0UL;
  AFIO->EXTICR[3] = (AFIO->EXTICR[3] & ~0xFUL) | 0x1UL;

  EXTI->IMR  |= (1UL << 0) | (1UL << 4) | (1UL << 6) | (1UL << 12);
  EXTI->RTSR |= (1UL << 0) | (1UL << 4) | (1UL << 6) | (1UL << 12);
  EXTI->FTSR |= (1UL << 0) | (1UL << 4) | (1UL << 6) | (1UL << 12);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
#endif
}

/* 中断内计数：a!=b → +1，否则 -1（方向由 g_enc_sign 修正） */
static inline void s_enc_step(uint8_t wheel, GPIO_TypeDef *pa, uint16_t pina,
                              GPIO_TypeDef *pb, uint16_t pinb)
{
  uint32_t a = ((pa->IDR & pina) != 0U) ? 1U : 0U;
  uint32_t b = ((pb->IDR & pinb) != 0U) ? 1U : 0U;
#if defined(STM32F103xB)
  s_enc_filter_edge(wheel,(uint8_t)a,(uint8_t)b,DWT->CYCCNT);
#else
  int32_t step = (a != b) ? 1 : -1;
  g_enc_count[wheel] += step * (int32_t)g_enc_sign[wheel];
#endif
}

#if defined(STM32F407xx)
void EXTI2_IRQHandler(void)        /* w1 左前：PE2(A) */
{
  if ((EXTI->PR & EXTI_PR_PR2) != 0U)
  {
    EXTI->PR = EXTI_PR_PR2;
    s_enc_step(1U, GPIOE, GPIO_PIN_2, GPIOE, GPIO_PIN_3);
  }
}
void EXTI4_IRQHandler(void)        /* w0 右前：PA4(A) */
{
  if ((EXTI->PR & EXTI_PR_PR4) != 0U)
  {
    EXTI->PR = EXTI_PR_PR4;
    s_enc_step(0U, GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5);
  }
}
void EXTI9_5_IRQHandler(void)      /* w2 右后 PD6(A) + w3 左后 PC7(A) */
{
  uint32_t pr = EXTI->PR & (EXTI_PR_PR6 | EXTI_PR_PR7);
  if ((pr & EXTI_PR_PR6) != 0U)
  {
    EXTI->PR = EXTI_PR_PR6;
    s_enc_step(2U, GPIOD, GPIO_PIN_6, GPIOD, GPIO_PIN_7);
  }
  if ((pr & EXTI_PR_PR7) != 0U)
  {
    EXTI->PR = EXTI_PR_PR7;
    s_enc_step(3U, GPIOC, GPIO_PIN_7, GPIOC, GPIO_PIN_12);
  }
}
#else
void EXTI0_IRQHandler(void)        /* enc0 右后：PA0(A) */
{
  if ((EXTI->PR & EXTI_PR_PR0) != 0U)
  {
    EXTI->PR = EXTI_PR_PR0;
    s_enc_step(0U, GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1);
  }
}
void EXTI4_IRQHandler(void)        /* enc1 左后：PA4(A) */
{
  if ((EXTI->PR & EXTI_PR_PR4) != 0U)
  {
    EXTI->PR = EXTI_PR_PR4;
    s_enc_step(1U, GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5);
  }
}
void EXTI9_5_IRQHandler(void)      /* enc2 右前：PA6(A) */
{
  if ((EXTI->PR & EXTI_PR_PR6) != 0U)
  {
    EXTI->PR = EXTI_PR_PR6;
    s_enc_step(2U, GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7);
  }
}
void EXTI15_10_IRQHandler(void)    /* enc3 左前：PB12(A) */
{
  if ((EXTI->PR & EXTI_PR_PR12) != 0U)
  {
    EXTI->PR = EXTI_PR_PR12;
    s_enc_step(3U, GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13);
  }
}
#endif

/* ======================== 初始化（调度器启动前） ======================== */

void App_Init(void)
{
  uint8_t i;

  s_clock_init();
  s_motor_gpio_init();
  s_motor_tim_init();
  s_enc_gpio_exti_init();

  AS5600_BusInit();
  PendulumAngle_Init(&s_pendulum, PENDULUM_UPRIGHT_RAW,
                     PENDULUM_ANGLE_SIGN, PENDULUM_ZERO_STORED);

  for (i = 0U; i < APP_WHEEL_COUNT; i++)
  {
    g_enc_count[i] = 0;
    g_enc_delta[i] = 0;
    s_enc_last[i] = 0;
  }
  s_motor_all_coast();   /* 上电默认滑行，g_bal_enable=0 时不会被驱动 */

  /* 读一次版本号做自检；同时防止链接器丢弃“无代码引用”的 g_fw_build 段
   * （Watch 需要该符号，缺失时看不到版本号） */
  g_app_init_done = (g_fw_build == APP_FW_BUILD) ? 1U : 0U;
}

/* ======================== ControlTask（200Hz） ======================== */

void App_ControlTask(void *argument)
{
  uint32_t deadline;
  uint8_t i;

  (void)argument;

  deadline = osKernelGetTickCount();
  for (;;)
  {
    uint32_t started_ms = HAL_GetTick();
    uint32_t now_tick = osKernelGetTickCount();
    float speed_dt;
    float counts_per_rev;

#if defined(STM32F103xB)
    AppCommands_Apply(); /* commit serial commands before this control cycle */
#endif
    if ((int32_t)(now_tick - deadline) > 0)
    {
      g_pendulum_late_wake_count++;
    }
    PendulumAngle_CheckFreshness(&s_pendulum, started_ms);

    /* ---- 零点/符号标定命令（Watch 写入，执行后自清零） ---- */
    {
      int8_t sign_request = g_pendulum_sign_request;
      uint8_t zero_request = g_pendulum_zero_request;
      g_pendulum_sign_request = 0;
      g_pendulum_zero_request = 0U;
      if (sign_request != 0)
      {
        Balance_Stop(BAL_REASON_CONFIG);
        if (PendulumAngle_SetSign(&s_pendulum, sign_request) == 0U)
          g_pendulum_command_errors++;
      }
      if (zero_request == 1U)
      {
        g_speed_enable = 0U; g_motor_test_enable = 0U; g_drive_test = 0;
        g_speed_reset = 1U;
        for (i = 0U; i < APP_WHEEL_COUNT; i++) g_speed_target[i] = 0.0f;
        Balance_BeginZero();
        PendulumAngle_BeginZero(&s_pendulum);
      }
      else if (zero_request != 0U)
      {
        g_pendulum_command_errors++;
      }
    }

    /* ---- AS5600 读取 + 摆角更新 ---- */
    {
      uint16_t raw_angle = 0U;
      uint8_t status = 0U;
      uint32_t read_started_ms = HAL_GetTick();
      HAL_StatusTypeDef result = AS5600_ReadSample(&raw_angle, &status);
      uint32_t now_ms = HAL_GetTick();
      uint8_t sensor_valid;

      g_as5600_read_time_ms = now_ms - read_started_ms;

      sensor_valid = (result == HAL_OK) &&
                     (status & AS5600_STATUS_MAGNET_DETECTED) != 0U &&
                     (status & (AS5600_STATUS_MAGNET_WEAK |
                                AS5600_STATUS_MAGNET_STRONG)) == 0U;

      /* 中值滤波（窗口 5）：只在读取成功时入窗，失败不污染窗口 */
      {
        uint16_t raw_used = raw_angle;
        if (result == HAL_OK)
        {
          raw_used = median3_circular(raw_angle);
          g_as5600_raw_filt = raw_used;
        }
        /* Stationary calibration checks original samples, not median-smoothed motion. */
        if (s_pendulum.zero_result == PENDULUM_ZERO_COLLECTING) raw_used = raw_angle;
        PendulumAngle_Update(&s_pendulum, raw_used, status,
                             (result == HAL_OK) ? 1U : 0U, now_ms);
      }
      PendulumAngle_CheckFreshness(&s_pendulum, HAL_GetTick());

      g_as5600_last_sample_ms = now_ms;
      if (result == HAL_OK)
      {
        g_as5600_status = status;
        g_as5600_raw_angle = raw_angle;
        g_as5600_angle_mdeg = AS5600_RawToMilliDegrees(raw_angle);
        g_as5600_valid = sensor_valid;
        g_as5600_read_ok++;
        if (sensor_valid != 0U)
        {
          g_as5600_last_valid_ms = now_ms;
        }
      }
      else
      {
        g_as5600_valid = 0U;
        g_as5600_read_errors++;
        if (result == HAL_TIMEOUT)
        {
          g_as5600_timeout_count++;
        }
      }

      g_pendulum_angle_rad = s_pendulum.angle_rad;
      g_pendulum_rate_rad_s = s_pendulum.rate_rad_s;
      g_pendulum_angle_valid = s_pendulum.valid;
      g_pendulum_calibrated = s_pendulum.calibrated;
      g_pendulum_upright_raw = s_pendulum.upright_raw;
      g_pendulum_bottom_raw = s_pendulum.bottom_raw;
      {
        float deg = 180.0f + s_pendulum.angle_rad * 57.2957795131f;
        if (deg >= 360.0f) deg -= 360.0f;
        g_pendulum_bottom_deg = deg;
      }
      g_pendulum_angle_sign = s_pendulum.sign;
      g_pendulum_zero_result = (uint32_t)s_pendulum.zero_result;
      g_pendulum_zero_samples = s_pendulum.zero_count;
      if (g_bal_state == BAL_ZEROING)
      {
        if (s_pendulum.zero_result == PENDULUM_ZERO_SUCCESS)
          (void)Balance_Arm(s_pendulum.calibrated);
        else if (s_pendulum.zero_result != PENDULUM_ZERO_COLLECTING)
          Balance_ZeroFailed();
      }

      /* 摆角有效性 1→0 跳变计数（通信/磁铁/时效丢失事件） */
      {
        static uint8_t s_prev_valid = 0U;
        uint8_t now_valid = s_pendulum.valid;
        if (s_prev_valid != 0U && now_valid == 0U)
        {
          g_pendulum_stale_events++;
        }
        s_prev_valid = now_valid;
      }
    }

    /* ---- 编码器测速：实际采样间隔，兼容任务偶发延迟，低通时间常数 20ms ---- */
#if defined(STM32F103xB)
    s_enc_filter_flush_ready();
#endif
    {
      uint32_t sample_ms = HAL_GetTick();
      uint32_t elapsed_ms = sample_ms - s_speed_last_sample_ms;
      if ((s_speed_sample_ready == 0U) || (elapsed_ms == 0U))
      {
        elapsed_ms = PENDULUM_SAMPLE_PERIOD_MS;
      }
      s_speed_last_sample_ms = sample_ms;
      s_speed_sample_ready = 1U;
      g_speed_dt_ms = elapsed_ms;
      speed_dt = (float)elapsed_ms * 0.001f;
    }
    for (i = 0U; i < APP_WHEEL_COUNT; i++)
    {
      int32_t now_count = g_enc_count[i];
      float raw_speed;
      float alpha;

      g_enc_delta[i] = now_count - s_enc_last[i];
      s_enc_last[i] = now_count;

      raw_speed = (float)g_enc_delta[i] / speed_dt;
      alpha = speed_dt / (0.020f + speed_dt);
      g_enc_speed[i] += alpha * (raw_speed - g_enc_speed[i]);
    }
    counts_per_rev = g_enc_counts_per_rev;
    if (!(counts_per_rev > 0.0f))
    {
      /* Watch 误填非正值时沿用资料值，避免除零；不改变模式/电机使能。 */
      counts_per_rev = APP_ENCODER_COUNTS_PER_REV;
    }
    for (i = 0U; i < APP_WHEEL_COUNT; i++)
    {
      int8_t m = g_enc_map[i];
      if ((m < 0) || (m >= (int8_t)APP_WHEEL_COUNT))
      {
        m = (int8_t)i;
      }
      {
        float raw_rpm=g_enc_speed[(uint8_t)m]*60.0f/counts_per_rev;
#if defined(STM32F103xB)
        float estimate=s_enc_speed_read((uint8_t)m,(float)g_enc_delta[(uint8_t)m]/speed_dt);
        float alpha=speed_dt/(0.020f+speed_dt);
        estimate*=60.0f/counts_per_rev;
        g_speed_estimated_rpm[i]+=alpha*(estimate-g_speed_estimated_rpm[i]);
#else
        g_speed_estimated_rpm[i]=raw_rpm;
#endif
        s_speed_feedback(i,g_speed_estimated_rpm[i],speed_dt);
        /* Keep the old count-derived channel for diagnostics/speed cutoff. */
        g_speed_fast[i]=raw_rpm;
      }
      g_speed_error[i] = g_speed_target[i] - g_speed_current[i];
    }

    s_speed_pid_select();

    /* Signed confirmed encoder increments, averaged across all four wheels.
     * Mapping is a permutation, so raw order does not change the chassis mean. */
    {
      float delta=0.0f;
      for(i=0U;i<APP_WHEEL_COUNT;i++) delta+=(float)g_enc_delta[i];
      Balance_AddPosition(delta/(counts_per_rev*(float)APP_WHEEL_COUNT));
    }

    /* Capture/exit checks use the freshest unfiltered angle; PID uses filtered
     * angle/rate. No other mode can silently override an armed balance session. */
    {
      int32_t diff = (int32_t)g_as5600_raw_angle - (int32_t)s_pendulum.upright_raw;
      if (diff >= 2048) diff -= 4096;
      if (diff < -2048) diff += 4096;
      PendulumAngle_CheckFreshness(&s_pendulum, HAL_GetTick());
      g_pendulum_angle_valid = s_pendulum.valid;
      Balance_Update(s_pendulum.angle_rad, s_pendulum.rate_rad_s,
          (float)s_pendulum.sign * (float)diff * (6.28318530718f / 4096.0f),
          (uint8_t)(s_pendulum.valid && g_as5600_valid), s_pendulum.calibrated,
          (uint8_t)(g_speed_enable || g_motor_test_enable || g_drive_test), HAL_GetTick());
      /* Fast feedback already has the 20ms encoder filter; avoid adding the
       * front wheels' extra 60ms speed-PI filter to runaway detection. */
      Balance_CheckMotionFeedback(g_speed_fast,g_speed_estimated_rpm,HAL_GetTick());
    }

    /* ---- 电机输出（互斥优先级：台架测试 > 直行测试 > 速度环 > 直立环 > 滑行） ---- */
    g_speed_active = ((g_speed_enable != 0U) &&
                      (g_motor_test_enable == 0U) && (g_drive_test == 0)) ? 1U : 0U;
    if ((g_speed_active == 0U) || (g_speed_reset != 0U))
    {
      /* 退出速度模式、被高优先级模式接管或 Watch 请求时清历史。 */
      g_speed_reset = 0U;
      for (i = 0U; i < APP_WHEEL_COUNT; i++)
      {
        App_ResetSpeedWheel(i);
      }
    }
    if (g_motor_test_enable != 0U)
    {
      uint8_t w = (g_motor_test_wheel < APP_WHEEL_COUNT) ? g_motor_test_wheel : 0U;
      uint32_t pct = (g_motor_test_pwm_pct > 100U) ? 100U : g_motor_test_pwm_pct;
      uint8_t raw_dir = (g_motor_test_dir != 0U) ? 1U : 0U;
#if defined(STM32F103xB)
      s_motor_requested[w] = (float)pct * 0.01f - ((raw_dir != 0U) ? 1.0f : 0.0f);
      g_bal_u[w] = s_motor_requested[w];
#else
      uint32_t ccr = (pct * s_motor_arr[w] + 50U) / 100U;
      s_motor_pwm_set(w, ccr); s_motor_dir_set(w, raw_dir);
      g_motor_test_ccr = (uint16_t)ccr;
      g_bal_ccr[w] = (uint16_t)ccr; g_bal_dir[w] = raw_dir;
      g_bal_u[w] = (float)ccr / (float)s_motor_arr[w] - ((raw_dir != 0U) ? 1.0f : 0.0f);
#endif
      for (i = 0U; i < APP_WHEEL_COUNT; i++)
      {
        if (i != w)
        {
          s_motor_apply(i, 0.0f);
        }
      }
    }
    else if (g_drive_test != 0)
    {
      /* 四轮开环直行测试（无传感器时的动力链测试）：-100..100 → 归一化控制量 */
      float u = (float)g_drive_test * 0.01f;
      for (i = 0U; i < APP_WHEEL_COUNT; i++)
      {
        s_motor_apply(i, u);
      }
    }
    else if (g_speed_active != 0U)
    {
      /* u = Kp*err + integral - Kd*d(measured)/dt。
       * 对已低通的反馈微分，修改目标值时不产生微分冲击；Kd=0 即 PI。
       * 每周期选取共用或单轮增益；四轮各自保存积分/微分历史。 */
      for (i = 0U; i < APP_WHEEL_COUNT; i++)
      {
        float kp = g_spd_kp_used[i];
        float ki = g_spd_ki_used[i];
        float kd = g_spd_kd_used[i];
        float measured = g_speed_current[i];
        float err = g_speed_error[i];
        float p = kp * err;
        float d = -kd * (measured - s_spd_prev_measured[i]) / speed_dt;
        float integral;
        float u;

        s_spd_prev_measured[i] = measured;
        if (ki == 0.0f)
        {
          g_spd_integral[i] = 0.0f;
        }
        integral = g_spd_integral[i] + ki * err * speed_dt;
        if (integral > 1.0f)  { integral = 1.0f; }
        if (integral < -1.0f) { integral = -1.0f; }
        u = p + integral + d;
        /* 输出饱和时不继续同向累积，允许反向积分消除饱和。 */
        if (!(((u > 1.0f) && (ki * err > 0.0f)) ||
              ((u < -1.0f) && (ki * err < 0.0f))))
        {
          g_spd_integral[i] = integral;
        }
        u = p + g_spd_integral[i] + d;
        if (u > 1.0f)  { u = 1.0f; }
        if (u < -1.0f) { u = -1.0f; }
        g_spd_u[i] = u;
        s_motor_apply(i, u);
      }
    }
    else if (g_bal_enable != 0U)
    {
      if ((g_bal_state == BAL_ACTIVE || g_bal_state == BAL_SWING) &&
          s_pendulum.valid != 0U && s_pendulum.calibrated != 0U)
      {
        for (i = 0U; i < APP_WHEEL_COUNT; i++)
        {
          s_motor_apply(i, Balance_MotorOutput(i));
        }
      }
      else
      {
        /* 使能中但摆角无效（通信/磁铁/时效）→ 安全停 */
        Balance_Stop(BAL_REASON_SENSOR);
        s_motor_all_coast();
      }
    }
    else
    {
      s_motor_all_coast();
    }

#if defined(STM32F103xB)
    s_motor_dma_commit();
    if (g_motor_test_enable != 0U && g_motor_test_wheel < APP_WHEEL_COUNT)
    {
      g_motor_test_ccr = g_bal_ccr[g_motor_test_wheel];
    }
#endif
    s_speed_led_update();

    /* ---- 周期统计与绝对节拍 ---- */
    {
      uint32_t cycle_time = HAL_GetTick() - started_ms;
      g_pendulum_cycle_time_ms = cycle_time;
      if (cycle_time > g_pendulum_max_cycle_time_ms)
      {
        g_pendulum_max_cycle_time_ms = cycle_time;
      }
      g_pendulum_sample_count++;

#if defined(STM32F103xB)
      AppTelemetry_Capture();
#endif

      deadline += PENDULUM_SAMPLE_PERIOD_MS;
      now_tick = osKernelGetTickCount();
      if ((int32_t)(now_tick - deadline) >= 0)
      {
        uint32_t missed = (uint32_t)(now_tick - deadline) /
                          PENDULUM_SAMPLE_PERIOD_MS + 1U;
        g_pendulum_missed_periods += missed;
        deadline += missed * PENDULUM_SAMPLE_PERIOD_MS;
      }
      if (osDelayUntil(deadline) != osOK)
      {
        g_pendulum_delay_errors++;
        osDelay(PENDULUM_SAMPLE_PERIOD_MS);
        deadline = osKernelGetTickCount();
      }
    }
  }
}
