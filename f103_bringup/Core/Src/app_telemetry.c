/* Nonblocking wheel PID telemetry and ASCII commands. No printf float library.
 * ControlTask publishes once per cycle; a lower-priority task formats CSV.
 * Only DMA1 channel 2's completion/error ISR releases the shared TX buffer. */
#include "app_telemetry.h"
#include "app_vehicle.h"
#include "app_commands.h"
#include "as5600.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#if !defined(STM32F103xB)
#error app_telemetry currently targets STM32F103xB
#endif

#define LOG_BAUD             115200U
#define LOG_TX_CAPACITY      2048U
#define LOG_CONFIG_MS        1000U
#define LOG_SCHEMA_MS        5000U
#define LOG_DMA_TIMEOUT_MS   1000U
#define LOG_RX_CAPACITY      512U
#define LOG_SERVICE_MS       5U

volatile uint8_t g_uart_log_enable = 1U;
volatile uint8_t g_uart_bal_trace = 1U;
volatile uint8_t g_uart_log_ready = 0U;
volatile uint8_t g_uart_log_task_ok = 0U;
volatile uint8_t g_uart_log_busy = 0U;
volatile uint32_t g_uart_log_period_ms = 100U;
volatile uint32_t g_uart_log_packets = 0U;
volatile uint32_t g_uart_log_dropped = 0U;
volatile uint32_t g_uart_log_dma_errors = 0U;
volatile uint32_t g_uart_log_dma_timeouts = 0U;
volatile uint32_t g_uart_log_stack_free_bytes = 0U;
volatile uint32_t g_uart_rx_bytes = 0U;
volatile uint32_t g_uart_rx_overruns = 0U;
volatile uint32_t g_uart_rx_dma_errors = 0U;
volatile uint32_t g_uart_rx_uart_errors = 0U;

typedef struct
{
  float target;
  float current;
  float error;
  float u;
  float integral;
  int32_t count;
  int32_t delta;
  uint16_t ccr;
  uint8_t dir;
} LogWheel;

typedef struct
{
  float angle_deg, theta_deg, rate, output, integral, kp, ki, kd, limit, entry, exit;
  uint32_t dwell, hold, zero_result;
  uint16_t bottom, upright, zero_samples;
  uint8_t state, reason, calibrated, valid;
  int8_t sign, angle_sign;
} LogBalance;

typedef struct
{
  float rpm, reference, integral, kp, ki, limit, guard_rpm, max_rpm, slew, trim, robust;
  uint32_t run_ms, max_run_ms;
  uint8_t enabled;
} LogVelocity;

typedef struct
{
  uint32_t ms;
  uint32_t cycle;
  uint32_t dt_ms;
  uint32_t exec_ms;
  uint32_t missed;
  uint32_t late;
  uint32_t fw;
  uint32_t clock_hz;
  uint32_t hse_fail;
  float kp;
  float ki;
  float kd;
  float cpr;
  int16_t drive_test;
  uint8_t mode;
  uint8_t enabled;
  uint8_t active;
  uint8_t reached;
  uint8_t motor_test;
  uint8_t motor_wheel;
  uint8_t motor_pct;
  uint8_t motor_dir;
  uint8_t balance;
  int8_t map[APP_WHEEL_COUNT];
  int8_t enc_sign[APP_WHEEL_COUNT];
  int8_t mot_sign[APP_WHEEL_COUNT];
  uint8_t individual[APP_WHEEL_COUNT];
  float pid_kp[APP_WHEEL_COUNT];
  float pid_ki[APP_WHEEL_COUNT];
  float pid_kd[APP_WHEEL_COUNT];
  float fast_rpm[APP_WHEEL_COUNT];
  uint32_t filter_ms[APP_WHEEL_COUNT];
  uint8_t on_ticks[APP_WHEEL_COUNT],in1_ticks[APP_WHEEL_COUNT],in2_ticks[APP_WHEEL_COUNT];
  uint16_t as_raw;
  uint8_t as_status,as_valid;
  LogBalance bal;
  LogVelocity vel;
  LogWheel wheel[APP_WHEEL_COUNT];
} LogSnapshot;

/* Both buffers have static lifetime. DMA never reads a task's local stack. */
static volatile LogSnapshot s_latest;
static volatile uint8_t s_latest_ready;
static volatile uint8_t s_tx[LOG_TX_CAPACITY];
static volatile uint8_t s_rx[LOG_RX_CAPACITY];
static volatile uint32_t s_rx_wraps;
static volatile uint8_t s_rx_dma_fault;
static volatile uint8_t s_rx_uart_fault;
static uint32_t s_rx_consumed; /* reader position, owned by UartLogTask */
/* 128 bytes per 5ms service exceeds 115200/8N1 ingress; DMA ring stays 512B. */
static uint8_t s_rx_chunk[128];
/* These fields belong exclusively to the logging task. */
static uint32_t s_tx_length;
static uint8_t s_tx_overflow;
static uint32_t s_tx_started_ms;

void AppTelemetry_Init(void)
{
  GPIO_InitTypeDef io = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* SWJ_CFG has undefined readback on F1: explicitly retain the project's
   * JTAG-disabled/SWD-enabled choice while preserving other peripheral remaps. */
  AFIO->MAPR = (AFIO->MAPR & ~(AFIO_MAPR_USART3_REMAP | AFIO_MAPR_SWJ_CFG)) |
                AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  io.Pin = GPIO_PIN_10;
  io.Mode = GPIO_MODE_AF_PP;
  io.Pull = GPIO_NOPULL;
  io.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &io);
  io.Pin = GPIO_PIN_11;
  io.Mode = GPIO_MODE_INPUT;
  io.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &io);

  __HAL_RCC_USART3_FORCE_RESET();
  __HAL_RCC_USART3_RELEASE_RESET();
  USART3->BRR = (HAL_RCC_GetPCLK1Freq() + LOG_BAUD / 2U) / LOG_BAUD;
  USART3->CR2 = 0U;                   /* one stop bit */
  USART3->CR3 = USART_CR3_EIE;         /* no flow control, RX errors interrupt */
  USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

  DMA1_Channel2->CCR = 0U;
  DMA1->IFCR = DMA_IFCR_CGIF2;
  DMA1_Channel2->CPAR = (uint32_t)&USART3->DR;
  DMA1_Channel2->CCR = DMA_CCR_DIR | DMA_CCR_MINC |
                       DMA_CCR_TCIE | DMA_CCR_TEIE;
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 7U, 0U);
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

  DMA1_Channel3->CCR = 0U;
  DMA1->IFCR = DMA_IFCR_CGIF3;
  DMA1_Channel3->CPAR = (uint32_t)&USART3->DR;
  DMA1_Channel3->CMAR = (uint32_t)s_rx;
  DMA1_Channel3->CNDTR = LOG_RX_CAPACITY;
  DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_CIRC |
                       DMA_CCR_TCIE | DMA_CCR_TEIE;
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 7U, 0U);
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel3_IRQn);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  HAL_NVIC_SetPriority(USART3_IRQn, 7U, 0U);
  HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  __DMB();
  DMA1_Channel3->CCR |= DMA_CCR_EN;
  USART3->CR3 |= USART_CR3_DMAR;
  g_uart_log_ready = 1U;
}

/* Runs only in ControlTask, after the current cycle's motor output/stats.
 * The lower-priority reader copies under a short FreeRTOS critical section. */
void AppTelemetry_Capture(void)
{
  uint8_t i;
  s_latest.ms = HAL_GetTick();
  s_latest.cycle = g_pendulum_sample_count;
  s_latest.dt_ms = g_speed_dt_ms;
  s_latest.exec_ms = g_pendulum_cycle_time_ms;
  s_latest.missed = g_pendulum_missed_periods;
  s_latest.late = g_pendulum_late_wake_count;
  s_latest.fw = g_fw_build;
  s_latest.clock_hz = g_clock_hz;
  s_latest.hse_fail = g_hse_fail;
  s_latest.kp = g_spd_kp;
  s_latest.ki = g_spd_ki;
  s_latest.kd = g_spd_kd;
  s_latest.cpr = g_enc_counts_per_rev;
  s_latest.drive_test = g_drive_test;
  s_latest.enabled = g_speed_enable;
  s_latest.active = g_speed_active;
  s_latest.reached = g_speed_reached;
  s_latest.motor_test = g_motor_test_enable;
  s_latest.motor_wheel = g_motor_test_wheel;
  s_latest.motor_pct = g_motor_test_pwm_pct;
  s_latest.motor_dir = g_motor_test_dir;
  s_latest.balance = g_bal_enable;
  s_latest.as_raw = g_as5600_raw_angle;
  s_latest.as_status = g_as5600_status;
  s_latest.as_valid = g_as5600_valid;
  s_latest.bal.angle_deg=g_pendulum_bottom_deg;
  s_latest.bal.theta_deg=g_bal_theta*57.2957795131f;
  s_latest.bal.rate=g_bal_theta_dot;
  s_latest.bal.output=g_bal_output;
  s_latest.bal.integral=g_bal_integral;
  s_latest.bal.kp=g_bal_kp; s_latest.bal.ki=g_bal_ki; s_latest.bal.kd=g_bal_kd;
  s_latest.bal.limit=g_bal_limit; s_latest.bal.entry=g_bal_entry_deg; s_latest.bal.exit=g_bal_exit_deg;
  s_latest.bal.dwell=g_bal_dwell_ms; s_latest.bal.hold=g_bal_hold_ms;
  s_latest.bal.zero_result=g_pendulum_zero_result; s_latest.bal.zero_samples=g_pendulum_zero_samples;
  s_latest.bal.bottom=g_pendulum_bottom_raw; s_latest.bal.upright=g_pendulum_upright_raw;
  s_latest.bal.state=g_bal_state; s_latest.bal.reason=g_bal_reason;
  s_latest.bal.calibrated=g_pendulum_calibrated; s_latest.bal.valid=g_pendulum_angle_valid;
  s_latest.bal.sign=g_bal_sign; s_latest.bal.angle_sign=g_pendulum_angle_sign;
  s_latest.vel.enabled=g_bal_vel_enable;
  s_latest.vel.rpm=g_bal_vel_rpm; s_latest.vel.reference=g_bal_ref_deg;
  s_latest.vel.integral=g_bal_vel_integral_deg;
  s_latest.vel.kp=g_bal_vel_kp; s_latest.vel.ki=g_bal_vel_ki;
  s_latest.vel.limit=g_bal_vel_limit_deg;
  s_latest.vel.guard_rpm=g_bal_guard_rpm; s_latest.vel.max_rpm=g_bal_max_rpm;
  s_latest.vel.run_ms=g_bal_run_ms; s_latest.vel.max_run_ms=g_bal_max_run_ms;
  s_latest.vel.slew=g_bal_vel_slew_deg_s;
  s_latest.vel.trim=g_bal_trim_deg;
  s_latest.vel.robust=g_bal_robust_rpm;
  s_latest.mode = (g_motor_test_enable != 0U) ? 4U :
                  (g_drive_test != 0) ? 3U :
                  (g_speed_active != 0U) ? 2U :
                  (g_bal_enable != 0U) ? 1U : 0U;
  for (i = 0U; i < APP_WHEEL_COUNT; i++)
  {
    s_latest.map[i] = g_enc_map[i];
    s_latest.enc_sign[i] = g_enc_sign[i];
    s_latest.mot_sign[i] = g_mot_sign[i];
    s_latest.individual[i] = g_spd_individual[i];
    s_latest.pid_kp[i] = g_spd_kp_used[i];
    s_latest.pid_ki[i] = g_spd_ki_used[i];
    s_latest.pid_kd[i] = g_spd_kd_used[i];
    s_latest.fast_rpm[i] = g_speed_fast[i];
    s_latest.filter_ms[i] = g_speed_filter_used_ms[i];
    s_latest.on_ticks[i] = g_motor_on_ticks[i];
    s_latest.in1_ticks[i] = g_motor_in1_ticks[i];
    s_latest.in2_ticks[i] = g_motor_in2_ticks[i];
    s_latest.wheel[i].target = g_speed_target[i];
    s_latest.wheel[i].current = g_speed_current[i];
    s_latest.wheel[i].error = g_speed_error[i];
    s_latest.wheel[i].u = g_spd_u[i];
    s_latest.wheel[i].integral = g_spd_integral[i];
    s_latest.wheel[i].count = g_enc_count[i];
    s_latest.wheel[i].delta = g_enc_delta[i];
    s_latest.wheel[i].ccr = g_bal_ccr[i];
    s_latest.wheel[i].dir = g_bal_dir[i];
  }
  s_latest_ready = 1U;
}

static void log_char(char c)
{
  if (s_tx_length < LOG_TX_CAPACITY)
  {
    s_tx[s_tx_length++] = (uint8_t)c;
  }
  else
  {
    s_tx_overflow = 1U;
  }
}

static void log_text(const char *text)
{
  while (*text != '\0')
  {
    log_char(*text++);
  }
}

static void log_u32(uint32_t value)
{
  char digits[10];
  uint8_t count = 0U;
  do
  {
    digits[count++] = (char)('0' + value % 10U);
    value /= 10U;
  } while (value != 0U);
  while (count != 0U)
  {
    log_char(digits[--count]);
  }
}

static void csv_u32(uint32_t value)
{
  log_char(',');
  log_u32(value);
}

static void csv_i32(int32_t value)
{
  uint32_t magnitude = (uint32_t)value;
  log_char(',');
  if (value < 0)
  {
    log_char('-');
    magnitude = 0U - magnitude; /* also handles INT32_MIN */
  }
  log_u32(magnitude);
}

static void csv_fixed(float value, uint32_t scale)
{
  float scaled = value * (float)scale;
  int32_t rounded;
  uint32_t magnitude;
  uint32_t fraction;
  uint32_t divisor;
  log_char(',');
  if ((scaled != scaled) || (scaled >= 2147483520.0f) ||
      (scaled <= -2147483520.0f))
  {
    log_text("nan"); /* never cast NaN/infinity/out-of-range to int */
    return;
  }
  rounded = (int32_t)((scaled >= 0.0f) ? scaled + 0.5f : scaled - 0.5f);
  magnitude = (uint32_t)rounded;
  if (rounded < 0)
  {
    log_char('-');
    magnitude = 0U - magnitude;
  }
  log_u32(magnitude / scale);
  log_char('.');
  fraction = magnitude % scale;
  for (divisor = scale / 10U; divisor != 0U; divisor /= 10U)
  {
    log_char((char)('0' + (fraction / divisor) % 10U));
  }
}

static void log_encoder_header(void)
{
  uint8_t i;
  log_text("#E,ms,qualify_us");
  for(i=0U;i<APP_WHEEL_COUNT;i++)
  {
    log_text(",irq"); log_char((char)('0'+i));
    log_text(",ok"); log_char((char)('0'+i));
    log_text(",glitch"); log_char((char)('0'+i));
    log_text(",dup"); log_char((char)('0'+i));
  }
  log_text("\r\n");
}

static void log_encoder(uint32_t ms)
{
  uint8_t i;
  uint32_t irq, accepted, glitch, duplicate;
  log_text("E"); csv_u32(ms); csv_u32(APP_ENCODER_QUALIFY_US);
  for(i=0U;i<APP_WHEEL_COUNT;i++)
  {
    /* Per-wheel atomic cumulative counters, sampled by the logging task.
     * Their time is approximate relative to the ControlTask T snapshot. */
    taskENTER_CRITICAL();
    irq=g_enc_irq[i]; accepted=g_enc_accepted[i];
    glitch=g_enc_glitch[i]; duplicate=g_enc_duplicate[i];
    taskEXIT_CRITICAL();
    csv_u32(irq); csv_u32(accepted); csv_u32(glitch); csv_u32(duplicate);
  }
  log_text("\r\n");
}

static void log_motion_header(void)
{
  log_text("#P,ms,target_rpm,command_rpm,error_rpm,pos_rev,goal_rev,seq_state,stage_ms\r\n");
  log_text("#Y,ms,target_rpm,command_rpm,diff_rpm,u,kp,limit,ki,i\r\n");
  log_text("#H,ms,rpm0,rpm1,rpm2,rpm3\r\n");
  log_text("#O,ms,kp,max_rpm,error_rev\r\n");
}

/* Event packet is allowed outside ACTIVE and with LOG=0. Formatting stays in
 * UartLogTask; ControlTask only freezes 28 bytes before resetting control. */
static uint8_t log_exit_event(uint32_t *event_ms)
{
  BalanceExitEvent event;
  uint8_t found;
  static const char * const reasons[] = {
    "NONE","STOP","ANGLE","SENSOR","TIMING","ZERO","CONFLICT","CONFIG","SPEED","RUN_TIME","TRAVEL"
  };
  taskENTER_CRITICAL();
  found=Balance_PeekExit(&event);
  taskEXIT_CRITICAL();
  if(!found) return 0U;
  *event_ms=event.ms;
  log_text("#X,ms,reason,code,peak_rpm,max_rpm,overspeed_ms,dwell_ms,raw_angle_deg,run_ms,coalesced\r\n");
  log_text("X"); csv_u32(event.ms); log_char(',');
  log_text((event.reason<sizeof(reasons)/sizeof(reasons[0])) ? reasons[event.reason] : "UNKNOWN");
  csv_u32(event.reason); csv_fixed(event.peak_rpm,100U); csv_fixed(event.max_rpm,100U);
  csv_u32(event.overspeed_ms); csv_u32(BAL_OVERSPEED_DWELL_MS);
  csv_fixed(event.angle_deg,1000U); csv_u32(event.run_ms); csv_u32(event.coalesced);
  log_text("\r\n");
  return 1U;
}

static void log_motion(uint32_t ms)
{
  float target, command, error, position, goal;
  float turn_ki, turn_integral;
  uint8_t state;
  uint32_t elapsed;
  taskENTER_CRITICAL();
  target=g_bal_vel_target_rpm; command=g_bal_vel_command_rpm;
  error=g_bal_vel_rpm-command; position=g_bal_position_rev; goal=g_bal_seq_goal_rev;
  state=g_bal_seq_state; elapsed=g_bal_seq_ms;
  taskEXIT_CRITICAL();
  log_text("P"); csv_u32(ms); csv_fixed(target,100U); csv_fixed(command,100U);
  csv_fixed(error,100U); csv_fixed(position,1000U); csv_fixed(goal,1000U);
  csv_u32(state); csv_u32(elapsed); log_text("\r\n");
  taskENTER_CRITICAL();
  target=g_bal_turn_target_rpm; command=g_bal_turn_command_rpm;
  error=g_bal_turn_rpm; position=g_bal_turn_output; goal=g_bal_turn_kp;
  turn_ki=g_bal_turn_ki; turn_integral=g_bal_turn_integral;
  taskEXIT_CRITICAL();
  log_text("Y"); csv_u32(ms); csv_fixed(target,100U); csv_fixed(command,100U);
  csv_fixed(error,100U); csv_fixed(position,1000U); csv_fixed(goal,1000000U);
  csv_fixed(BAL_TURN_DUTY_LIMIT,1000U); csv_fixed(turn_ki,1000000U);
  csv_fixed(turn_integral,1000U); log_text("\r\n");
  taskENTER_CRITICAL();
  target=g_speed_estimated_rpm[0]; command=g_speed_estimated_rpm[1];
  error=g_speed_estimated_rpm[2]; position=g_speed_estimated_rpm[3];
  taskEXIT_CRITICAL();
  log_text("H"); csv_u32(ms); csv_fixed(target,100U); csv_fixed(command,100U);
  csv_fixed(error,100U); csv_fixed(position,100U); log_text("\r\n");
  taskENTER_CRITICAL();
  target=g_bal_pos_kp; command=g_bal_pos_max_rpm;
  error=g_bal_seq_goal_rev-g_bal_position_rev;
  taskEXIT_CRITICAL();
  log_text("O"); csv_u32(ms); csv_fixed(target,100U); csv_fixed(command,100U);
  csv_fixed(error,1000U); log_text("\r\n");
}

static void log_swing_header(void)
{
  log_text("#U,ms,phase,bottom_deg,rate_rad_s,energy,target_energy,u,pos_rev,mean_rpm,limit,travel_rev,run_ms,max_run_ms,omega2,gain,capture_rate,capture_deg\r\n");
}

static void log_swing(const LogSnapshot *s)
{
  float energy,limit,travel,position,mean,omega,gain,cap,angle;
  uint32_t maximum;uint8_t phase;
  taskENTER_CRITICAL();
  energy=g_swing_energy;limit=g_swing_limit;travel=g_swing_travel_rev;
  position=g_bal_position_rev;mean=g_bal_robust_rpm;omega=g_swing_omega2;
  gain=g_swing_gain;cap=g_swing_capture_rate;maximum=g_swing_max_ms;phase=g_swing_phase;
  angle=g_swing_capture_deg;if(angle>g_bal_entry_deg)angle=g_bal_entry_deg;
  taskEXIT_CRITICAL();
  log_text("U");csv_u32(s->ms);csv_u32(phase);csv_fixed(s->bal.angle_deg,1000U);
  csv_fixed(s->bal.rate,1000U);csv_fixed(energy,1000U);csv_fixed(1.02f,1000U);
  csv_fixed(s->bal.output,1000U);csv_fixed(position,1000U);csv_fixed(mean,100U);
  csv_fixed(limit,1000U);csv_fixed(travel,100U);csv_u32(s->vel.run_ms);csv_u32(maximum);
  csv_fixed(omega,100U);csv_fixed(gain,1000U);csv_fixed(cap,100U);csv_fixed(angle,100U);log_text("\r\n");
}

static void log_schema(void)
{
  uint8_t i;
  log_text("#F103_PID_CSV_V22,baud=115200,format=8N1,rpm=output_shaft\r\n");
  log_text("#D,ms,cycle,dt_ms,mode,speed_en,active,reached,exec_ms,missed,late,log_drop,dma_err");
  for (i = 0U; i < APP_WHEEL_COUNT; i++)
  {
    const char *names[] = {"target", "rpm", "error", "u", "i", "count", "delta", "ccr", "dir"};
    uint8_t j;
    for (j = 0U; j < (uint8_t)(sizeof(names) / sizeof(names[0])); j++)
    {
      log_char(',');
      log_text(names[j]);
      log_char((char)('0' + i));
    }
  }
  log_text("\r\n#C,ms,fw_hex,kp,ki,kd,cpr,clock_hz,hse_fail,map0,map1,map2,map3,enc_sign0,enc_sign1,enc_sign2,enc_sign3,mot_sign0,mot_sign1,mot_sign2,mot_sign3,motor_test,motor_wheel,motor_pct,motor_dir,drive_test,bal_enable,dma_timeout\r\n");
  log_text("#S,ms,log_enable,period_ms,tx_packets,log_drop,tx_dma_err,tx_timeout,rx_bytes,rx_overrun,rx_dma_err,rx_uart_err,cmd_received,cmd_applied,cmd_errors,reply_drop,stack_free,as_ms,as_busy_skips,as_valid,as_ok,as_errors\r\n");
  log_text("#W,ms,individual0,kp0,ki0,kd0,individual1,kp1,ki1,kd1,individual2,kp2,ki2,kd2,individual3,kp3,ki3,kd3\r\n");
  log_text("#F,ms,tau_ms0,fast_rpm0,tau_ms1,fast_rpm1,tau_ms2,fast_rpm2,tau_ms3,fast_rpm3\r\n");
  log_text("#M,ms,slow_decay,slots,pwm_hz,periods,dma_err,refill_late,isr_max_cycles,on0,in1_hi0,in2_hi0,on1,in1_hi1,in2_hi1,on2,in1_hi2,in2_hi2,on3,in1_hi3,in2_hi3\r\n");
  log_text("#Q,ms,raw,angle_deg,status,valid,md,ml,mh\r\n");
  log_text("#B,ms,state,reason,cal,valid,zero_result,zero_samples,bottom_raw,upright_raw,bottom_deg,error_deg,rate_rad_s,u,i,kp,ki,kd,limit,entry_deg,exit_deg,dwell_ms,hold_ms,sign,angle_sign\r\n");
  log_text("#V,ms,en,rpm,ref_deg,i_deg,kp,ki,limit_deg,guard_rpm,max_rpm,run_ms,max_run_ms,slew_deg_s,trim_deg,robust_rpm\r\n");
  log_encoder_header();
  log_motion_header();
  log_swing_header();
}

static void log_velocity(const LogSnapshot *s)
{
  log_text("V"); csv_u32(s->ms); csv_u32(s->vel.enabled);
  csv_fixed(s->vel.rpm,100U); csv_fixed(s->vel.reference,1000U);
  csv_fixed(s->vel.integral,1000U); csv_fixed(s->vel.kp,1000000U);
  csv_fixed(s->vel.ki,1000000U); csv_fixed(s->vel.limit,100U);
  csv_fixed(s->vel.guard_rpm,100U); csv_fixed(s->vel.max_rpm,100U);
  csv_u32(s->vel.run_ms); csv_u32(s->vel.max_run_ms);
  csv_fixed(s->vel.slew,100U);
  csv_fixed(s->vel.trim,1000U);
  csv_fixed(s->vel.robust,100U);
  log_text("\r\n");
}

static void log_balance(const LogSnapshot *s)
{
  log_text("B"); csv_u32(s->ms);
  csv_u32(s->bal.state); csv_u32(s->bal.reason);
  csv_u32(s->bal.calibrated); csv_u32(s->bal.valid);
  csv_u32(s->bal.zero_result); csv_u32(s->bal.zero_samples);
  csv_u32(s->bal.bottom); csv_u32(s->bal.upright);
  csv_fixed(s->bal.angle_deg,1000U); csv_fixed(s->bal.theta_deg,1000U);
  csv_fixed(s->bal.rate,1000U); csv_fixed(s->bal.output,1000U); csv_fixed(s->bal.integral,1000U);
  csv_fixed(s->bal.kp,1000000U); csv_fixed(s->bal.ki,1000000U); csv_fixed(s->bal.kd,1000000U);
  csv_fixed(s->bal.limit,1000U); csv_fixed(s->bal.entry,100U); csv_fixed(s->bal.exit,100U);
  csv_u32(s->bal.dwell); csv_u32(s->bal.hold); csv_i32(s->bal.sign); csv_i32(s->bal.angle_sign);
  log_text("\r\n");
}

static uint32_t log_period_ms(uint8_t trace, uint32_t requested)
{
  if (trace || requested<20U) return 20U;
  if (requested>1000U) return 1000U;
  return requested;
}

static void log_trace(const LogSnapshot *s)
{
  uint8_t wheel;
  float mean=0.0f;
  for (wheel=0U; wheel<APP_WHEEL_COUNT; wheel++) mean+=s->fast_rpm[wheel];
  log_text("T"); csv_u32(s->ms); csv_u32(s->as_raw);
  csv_fixed(s->bal.theta_deg,1000U); csv_fixed(s->bal.rate,1000U);
  csv_fixed(s->bal.output,1000U); csv_fixed(mean/(float)APP_WHEEL_COUNT,100U);
  csv_fixed(s->vel.reference,1000U); csv_fixed(s->vel.guard_rpm,100U);
  csv_u32(s->bal.state); csv_u32(s->bal.reason);
  for (wheel=0U; wheel<APP_WHEEL_COUNT; wheel++) csv_fixed(s->fast_rpm[wheel],100U);
  csv_fixed(s->vel.robust,100U); log_text("\r\n");
}

static void log_trace_header(const LogSnapshot *s)
{
  uint32_t shift;
  log_text((s->bal.state==BAL_SWING) ? "#F103_SWING_TRACE_V2,baud=115200,period_ms=40,fw=" :
                                    "#F103_BAL_TRACE_V2,baud=115200,period_ms=20,fw=");
  log_text("0x");
  for (shift=28U; ; shift-=4U)
  {
    uint32_t digit=(s->fw>>shift)&15U;
    log_char((char)((digit<10U) ? '0'+digit : 'A'+digit-10U));
    if (!shift) break;
  }
  log_text("\r\n");
  log_text("#T,ms,raw,error_deg,rate_rad_s,u,mean_rpm,ref_deg,guard_rpm,state,reason,fast0,fast1,fast2,fast3,robust_rpm\r\n");
  log_encoder_header();
  log_motion_header();
  log_swing_header();
}

static uint8_t log_config_changed(const LogSnapshot *a, const LogSnapshot *b)
{
  uint8_t i;
  if ((a->kp != b->kp) || (a->ki != b->ki) || (a->kd != b->kd) ||
      (a->cpr != b->cpr) || (a->motor_test != b->motor_test) ||
      (a->motor_wheel != b->motor_wheel) || (a->motor_pct != b->motor_pct) ||
      (a->motor_dir != b->motor_dir) || (a->drive_test != b->drive_test) ||
      (a->balance != b->balance))
  {
    return 1U;
  }
  for (i = 0U; i < APP_WHEEL_COUNT; i++)
  {
    if ((a->map[i] != b->map[i]) || (a->enc_sign[i] != b->enc_sign[i]) ||
        (a->mot_sign[i] != b->mot_sign[i]) ||
        (a->individual[i] != b->individual[i]) ||
        (a->pid_kp[i] != b->pid_kp[i]) || (a->pid_ki[i] != b->pid_ki[i]) ||
        (a->pid_kd[i] != b->pid_kd[i]) || (a->filter_ms[i] != b->filter_ms[i]))
    {
      return 1U;
    }
  }
  return 0U;
}

static void log_config(const LogSnapshot *sample)
{
  uint8_t i;
  uint32_t shift;
  log_text("C");
  csv_u32(sample->ms);
  log_text(",0x");
  for (shift = 28U; ; shift -= 4U)
  {
    uint32_t digit = (sample->fw >> shift) & 15U;
    log_char((char)((digit < 10U) ? '0' + digit : 'A' + digit - 10U));
    if (shift == 0U) { break; }
  }
  csv_fixed(sample->kp, 1000000U);
  csv_fixed(sample->ki, 1000000U);
  csv_fixed(sample->kd, 1000000U);
  csv_fixed(sample->cpr, 1000U);
  csv_u32(sample->clock_hz);
  csv_u32(sample->hse_fail);
  for (i = 0U; i < APP_WHEEL_COUNT; i++) { csv_i32(sample->map[i]); }
  for (i = 0U; i < APP_WHEEL_COUNT; i++) { csv_i32(sample->enc_sign[i]); }
  for (i = 0U; i < APP_WHEEL_COUNT; i++) { csv_i32(sample->mot_sign[i]); }
  csv_u32(sample->motor_test);
  csv_u32(sample->motor_wheel);
  csv_u32(sample->motor_pct);
  csv_u32(sample->motor_dir);
  csv_i32(sample->drive_test);
  csv_u32(sample->balance);
  csv_u32(g_uart_log_dma_timeouts);
  log_text("\r\n");
}

static void log_angle(const LogSnapshot *sample)
{
  log_text("Q"); csv_u32(sample->ms); csv_u32(sample->as_raw);
  /* Raw mechanical angle, NOT pendulum zero-relative/calibrated angle. */
  csv_fixed((float)sample->as_raw * (360.0f / 4096.0f), 1000U);
  csv_u32(sample->as_status); csv_u32(sample->as_valid);
  csv_u32((sample->as_status & AS5600_STATUS_MAGNET_DETECTED) != 0U);
  csv_u32((sample->as_status & AS5600_STATUS_MAGNET_WEAK) != 0U);
  csv_u32((sample->as_status & AS5600_STATUS_MAGNET_STRONG) != 0U);
  log_text("\r\n");
}

static void log_motor(const LogSnapshot *sample)
{
  uint8_t i;
  log_text("M"); csv_u32(sample->ms); csv_u32(1U); csv_u32(APP_MOTOR_PWM_SLOTS);
  csv_u32(g_motor_pwm_hz); csv_u32(g_motor_dma_periods); csv_u32(g_motor_dma_errors);
  csv_u32(g_motor_dma_late); csv_u32(g_motor_dma_max_cycles);
  for (i = 0U; i < APP_WHEEL_COUNT; i++)
  {
    csv_u32(sample->on_ticks[i]); csv_u32(sample->in1_ticks[i]); csv_u32(sample->in2_ticks[i]);
  }
  log_text("\r\n");
}

static void log_feedback(const LogSnapshot *sample)
{
  uint8_t i;
  log_text("F");
  csv_u32(sample->ms);
  for (i = 0U; i < APP_WHEEL_COUNT; i++)
  {
    csv_u32(sample->filter_ms[i]);
    csv_fixed(sample->fast_rpm[i], 100U);
  }
  log_text("\r\n");
}

static void log_data(const LogSnapshot *sample)
{
  uint8_t i;
  log_text("D");
  csv_u32(sample->ms);
  csv_u32(sample->cycle);
  csv_u32(sample->dt_ms);
  csv_u32(sample->mode);
  csv_u32(sample->enabled);
  csv_u32(sample->active);
  csv_u32(sample->reached);
  csv_u32(sample->exec_ms);
  csv_u32(sample->missed);
  csv_u32(sample->late);
  csv_u32(g_uart_log_dropped);
  csv_u32(g_uart_log_dma_errors);
  for (i = 0U; i < APP_WHEEL_COUNT; i++)
  {
    csv_fixed(sample->wheel[i].target, 100U);
    csv_fixed(sample->wheel[i].current, 100U);
    csv_fixed(sample->wheel[i].error, 100U);
    csv_fixed(sample->wheel[i].u, 1000U);
    csv_fixed(sample->wheel[i].integral, 1000U);
    csv_i32(sample->wheel[i].count);
    csv_i32(sample->wheel[i].delta);
    csv_u32(sample->wheel[i].ccr);
    csv_u32(sample->wheel[i].dir);
  }
  log_text("\r\n");
}

static void log_status(uint32_t ms)
{
  log_text("S");
  csv_u32(ms);
  csv_u32(g_uart_log_enable);
  csv_u32(g_uart_log_period_ms);
  csv_u32(g_uart_log_packets);
  csv_u32(g_uart_log_dropped);
  csv_u32(g_uart_log_dma_errors);
  csv_u32(g_uart_log_dma_timeouts);
  csv_u32(g_uart_rx_bytes);
  csv_u32(g_uart_rx_overruns);
  csv_u32(g_uart_rx_dma_errors);
  csv_u32(g_uart_rx_uart_errors);
  csv_u32(g_uart_cmd_received);
  csv_u32(g_uart_cmd_applied);
  csv_u32(g_uart_cmd_errors);
  csv_u32(g_uart_cmd_reply_drops);
  csv_u32(g_uart_log_stack_free_bytes);
  csv_u32(g_as5600_read_time_ms);
  csv_u32(g_as5600_busy_skips);
  csv_u32(g_as5600_valid);
  csv_u32(g_as5600_read_ok);
  csv_u32(g_as5600_read_errors);
  log_text("\r\n");
}

/* A single diagnostic query is independent of LOG and balance state. ControlTask
 * already samples AS5600 every cycle, even while stopped: reuse its atomic
 * snapshot instead of issuing competing I2C transactions from UartLogTask.
 * On read failure raw/status retain the last successful values; valid is zero
 * and S reports the error counters. Keep this packet separate from HELP/GET. */
static void log_as5600(const LogSnapshot *sample)
{
  log_config(sample);
  log_status(sample->ms);
  log_angle(sample);
}

static void log_wheel_pid(const LogSnapshot *sample)
{
  uint8_t i;
  log_text("W");
  csv_u32(sample->ms);
  for (i = 0U; i < APP_WHEEL_COUNT; i++)
  {
    csv_u32(sample->individual[i]);
    csv_fixed(sample->pid_kp[i], 1000000U);
    csv_fixed(sample->pid_ki[i], 1000000U);
    csv_fixed(sample->pid_kd[i], 1000000U);
  }
  log_text("\r\n");
}

static void log_start_dma(void)
{
  /* Buffer was filled only while idle. IRQ cannot make it busy on its own. */
  g_uart_log_busy = 1U;
  s_tx_started_ms = HAL_GetTick();
  DMA1_Channel2->CCR &= ~DMA_CCR_EN;
  DMA1->IFCR = DMA_IFCR_CGIF2;
  DMA1_Channel2->CMAR = (uint32_t)s_tx;
  DMA1_Channel2->CNDTR = s_tx_length;
  __DMB();
  USART3->CR3 |= USART_CR3_DMAT;
  DMA1_Channel2->CCR |= DMA_CCR_EN;
}

void DMA1_Channel2_IRQHandler(void)
{
  uint32_t flags = DMA1->ISR;
  if ((flags & (DMA_ISR_TCIF2 | DMA_ISR_TEIF2)) != 0U)
  {
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    USART3->CR3 &= ~USART_CR3_DMAT;
    DMA1->IFCR = DMA_IFCR_CGIF2;
    if ((flags & DMA_ISR_TEIF2) != 0U)
    {
      g_uart_log_dma_errors++;
    }
    else
    {
      g_uart_log_packets++;
    }
    __DMB();
    g_uart_log_busy = 0U;
  }
}

void DMA1_Channel3_IRQHandler(void)
{
  uint32_t flags = DMA1->ISR;
  if ((flags & DMA_ISR_TEIF3) != 0U)
  {
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF3;
    g_uart_rx_dma_errors++;
    s_rx_dma_fault = 1U;
  }
  else if ((flags & DMA_ISR_TCIF3) != 0U)
  {
    s_rx_wraps++;
    DMA1->IFCR = DMA_IFCR_CGIF3;
  }
}

void USART3_IRQHandler(void)
{
  uint32_t status = USART3->SR;
  if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
  {
    uint32_t discarded = USART3->DR; /* SR then DR clears UART RX error flags */
    (void)discarded;
    g_uart_rx_uart_errors++;
    s_rx_uart_fault = 1U;
  }
}

static void log_rx_service(void)
{
  uint32_t wraps;
  uint32_t remaining;
  uint32_t produced;
  uint32_t available;
  uint32_t count;
  uint32_t i;
  uint8_t fault;

  taskENTER_CRITICAL();
  fault = s_rx_uart_fault;
  s_rx_uart_fault = 0U;
  if (s_rx_dma_fault != 0U)
  {
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF3;
    DMA1_Channel3->CMAR = (uint32_t)s_rx;
    DMA1_Channel3->CNDTR = LOG_RX_CAPACITY;
    s_rx_wraps = 0U;
    s_rx_consumed = 0U;
    s_rx_dma_fault = 0U;
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel3_IRQn);
    __DMB();
    DMA1_Channel3->CCR |= DMA_CCR_EN;
    fault = 1U;
  }
  wraps = s_rx_wraps;
  remaining = DMA1_Channel3->CNDTR;
  if ((DMA1->ISR & DMA_ISR_TCIF3) != 0U)
  {
    /* A wrap can occur while this task masks the channel's IRQ. */
    wraps++;
    remaining = DMA1_Channel3->CNDTR;
    if (remaining == 0U) { remaining = LOG_RX_CAPACITY; }
  }
  produced = wraps * LOG_RX_CAPACITY + (LOG_RX_CAPACITY - remaining);
  taskEXIT_CRITICAL();
  if (fault != 0U)
  {
    s_rx_consumed = produced;
    AppCommands_RxError(APP_CMD_RX_ERROR);
    return;
  }
  available = produced - s_rx_consumed;
  if (available >= LOG_RX_CAPACITY)
  {
    g_uart_rx_overruns++;
    s_rx_consumed = produced;
    AppCommands_RxError(APP_CMD_RX_OVERFLOW);
    return;
  }
  count = (available < sizeof(s_rx_chunk)) ? available : sizeof(s_rx_chunk);
  /* Copy bounded bytes promptly so subsequent parsing cannot be lapped by DMA. */
  for (i = 0U; i < count; i++)
  {
    s_rx_chunk[i] = s_rx[(s_rx_consumed + i) % LOG_RX_CAPACITY];
  }
  s_rx_consumed += count;
  for (i = 0U; i < count; i++)
  {
    if (s_rx_uart_fault != 0U || s_rx_dma_fault != 0U) { break; }
    g_uart_rx_bytes++;
    AppCommands_FeedByte(s_rx_chunk[i]);
  }
}

static uint8_t log_replies(uint8_t *help, uint8_t *get, uint8_t *as_query)
{
  AppCommandReply reply;
  uint8_t count = 0U;
  *help = 0U;
  *get = 0U;
  *as_query = 0U;
  while (count < 8U && AppCommands_PopReply(&reply) != 0U)
  {
    log_text("A");
    csv_u32(reply.id);
    log_text((reply.status == APP_CMD_OK) ? ",OK," : ",ERR,");
    log_text(AppCommands_Name(reply.type));
    if (reply.status != APP_CMD_OK)
    {
      log_char(',');
      log_text(AppCommands_Status(reply.status));
    }
    else
    {
      if (reply.type == APP_CMD_HELP) { *help = 1U; }
      if (reply.type == APP_CMD_GET) { *get = 1U; }
      if (reply.type == APP_CMD_AS5600) { *as_query = 1U; }
    }
    log_text("\r\n");
    count++;
  }
  return count;
}

static void log_help(void)
{
  log_text("#HELP,ASCII case-insensitive; CR/LF; speeds RPM\r\n"
           "#HELP,SET wheel rpm | ALL rpm | TARGET r0 r1 r2 r3 | PID kp ki kd\r\n"
           "#HELP,PIDW wheel kp ki kd (only this wheel; PID returns all to common)\r\n"
           "#HELP,OPEN wheel signed_pct (-100..100 integer)\r\n"
           "#HELP,KP value | KI value | KD value | RUN 1 | STOP | RUN 0 | RESET\r\n"
           "#HELP,MAP motor encoder | ENCSIGN encoder +/-1 | MOTSIGN motor +/-1 | CPR counts\r\n"
           "#HELP,LOG 0/1 | PERIOD 20..1000 | LED wheel tolerance_pct | GET | HELP\r\n"
           "#HELP,AS5600 (latest C/S/Q; all states, even LOG=0)\r\n"
           "#HELP,FWD rpm | REV rpm | MOVE signed_rpm (0..280; exits balance)\r\n"
           "#HELP,ZERO (bottom,1s,auto-arm) | ARM (re-arm after trip)\r\n"
           "#HELP,BALPID kp ki kd | BALWIN entry_deg exit_deg dwell_ms | BALLIMIT pct\r\n"
           "#HELP,BALSIGN +/-1 | PENDSIGN +/-1 (stops)\r\n"
           "#HELP,BALSAFE rpm ms (0=off/10..280,0=off/500..60000; dwell200; stops)\r\n"
           "#HELP,BALVEL kp ki deg (0..0.05,0..0.01,0.1..3; stops)\r\n"
           "#HELP,BALSLEW deg_s (0.1..10; stops)\r\n"
           "#HELP,BALTRACE 0/1 (1=50Hz T; 0=CSV; stops)\r\n"
           "#HELP,BALTRIM deg (-1..1; default -0.81; stops)\r\n"
           "#HELP,BALSPD rpm (-30..30, <=half speed cutoff; ACTIVE only; ramped)\r\n"
           "#HELP,BALTURN rpm (-10..10; +right/-left half-difference; ACTIVE; cancels hold)\r\n"
           "#HELP,BALSPIN rpm (-10..10; ACTIVE; yaw with mean-travel hold; 0=stop yaw)\r\n"
           "#HELP,SWING | SWINGCFG pct rev ms | SWINGENERGY gain omega2 capture_rate | SWINGDIR +/-1 | SWINGCATCH deg\r\n"
           "#HELP,BALTURNPI kp ki (0..0.02,0..0.05; default 0.008,0.010; clears turn I)\r\n"
           "#HELP,BALPOS rev (-5..5) | BALHOLD | BALPZERO (ACTIVE, no turn; hold)\r\n"
           "#HELP,BALPOSCFG kp max_rpm (0.1..60,1..15; default 20,6)\r\n"
           "#HELP,BALSEQ [rpm turns pause_ms] (default 8 0.25 2000; ACTIVE; out/hold/home)\r\n"
           "#HELP,BALWHEEL mm (20..300; stops) | BALSTEP [metres rpm] (default 1 20; ACTIVE; hold5/move/hold5)\r\n"
           "#HELP,Parameters ACTIVE/SWING; X exit once even LOG=0; replies always on.\r\n"
           "#HELP,ZERO first; manual +/-8deg 400ms; swing catch +/-2deg; STOP cancels.\r\n");
}

void AppTelemetry_Task(void *argument)
{
  LogSnapshot sample;
  LogSnapshot last_config = {0};
  uint32_t last_config_ms = 0U;
  uint32_t last_schema_ms = 0U;
  uint32_t last_motion_ms = 0U;
  uint32_t deadline = osKernelGetTickCount();
  uint32_t next_log_ms = HAL_GetTick();
  uint8_t started = 0U;
  uint8_t pending_get = 0U;
  uint8_t pending_as5600 = 0U;
  uint8_t last_trace = 0U;
  uint8_t last_state = BAL_OFF;
  (void)argument;

  for (;;)
  {
    uint32_t now_ms = HAL_GetTick();
    uint8_t trace = g_uart_bal_trace != 0U;
    uint32_t period_ms = log_period_ms(trace,g_uart_log_period_ms);
    uint8_t send_schema;
    uint8_t send_config;
    uint8_t help;
    uint8_t get;
    uint8_t as_query;
    uint8_t replies;
    uint8_t log_due;
    uint8_t balance_running;
    uint8_t parameter_packet = 0U;
    uint8_t exit_packet = 0U;
    uint32_t exit_ms = 0U;
    if (trace!=last_trace)
    {
      started=0U; next_log_ms=now_ms; last_trace=trace;
    }
    log_rx_service();

    if ((g_uart_log_busy != 0U) &&
        ((uint32_t)(now_ms - s_tx_started_ms) > LOG_DMA_TIMEOUT_MS))
    {
      /* Recover this logger only; never alter motor modes on log failures. */
      taskENTER_CRITICAL();
      DMA1_Channel2->CCR &= ~DMA_CCR_EN;
      USART3->CR3 &= ~USART_CR3_DMAT;
      DMA1->IFCR = DMA_IFCR_CGIF2;
      HAL_NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
      g_uart_log_busy = 0U;
      g_uart_log_dma_timeouts++;
      taskEXIT_CRITICAL();
    }
    /* Keep RX/replies alive in every mode; parameter packets require an actual
     * balance cycle, never merely ARM/HOLD or a Watch enable request. */
    taskENTER_CRITICAL();
    sample = s_latest;
    balance_running = (s_latest_ready && g_bal_enable &&
        (g_bal_state == BAL_ACTIVE || g_bal_state == BAL_SWING) && sample.balance &&
        sample.bal.state == g_bal_state && sample.mode == 1U) ? 1U : 0U;
    taskEXIT_CRITICAL();
    if(sample.bal.state==BAL_SWING && period_ms<40U) period_ms=40U;
    if(sample.bal.state!=last_state)
    {
      started=0U;next_log_ms=now_ms;last_state=sample.bal.state;
    }
    if (balance_running == 0U) { pending_get = 0U; }
    log_due = ((g_uart_log_enable != 0U) && balance_running &&
               ((int32_t)(now_ms - next_log_ms) >= 0)) ? 1U : 0U;
    if (log_due != 0U) { next_log_ms = now_ms + period_ms; }
    if (g_uart_log_enable == 0U || balance_running == 0U)
    {
      started = 0U;
      next_log_ms = now_ms;
    }
    if (g_uart_log_busy != 0U)
    {
      if (log_due != 0U) { g_uart_log_dropped++; }
    }
    else if (g_uart_log_ready != 0U && s_latest_ready != 0U)
    {
      s_tx_length = 0U;
      s_tx_overflow = 0U;
      replies = log_replies(&help, &get, &as_query);
      if (replies != 0U)
      {
        if (help != 0U) { log_help(); }
        /* HELP and GET are separate packets so neither can exhaust 2 KiB TX. */
        if (get != 0U && balance_running != 0U) { pending_get = 1U; }
        if (as_query != 0U) { pending_as5600 = 1U; }
        if (log_due != 0U) { g_uart_log_dropped++; }
      }
      else if (log_exit_event(&exit_ms))
      {
        exit_packet=1U;
        if(log_due) g_uart_log_dropped++;
      }
      else if (pending_as5600 != 0U)
      {
        /* Explicit read-only query: no ACTIVE/SWING gate at DMA publication. */
        log_as5600(&sample);
        pending_as5600 = 0U;
        if (log_due) { g_uart_log_dropped++; }
      }
      else if (pending_get != 0U)
      {
        parameter_packet = 1U;
        log_config(&sample); log_status(sample.ms); log_wheel_pid(&sample); log_motor(&sample);
        log_data(&sample); log_feedback(&sample); log_angle(&sample); log_balance(&sample); log_velocity(&sample); log_encoder(sample.ms); log_motion(sample.ms);
        if(sample.bal.state==BAL_SWING) log_swing(&sample);
        pending_get = 0U;
      }
      else if (log_due != 0U)
      {
        parameter_packet = 1U;
        if (trace)
        {
          if (!started) log_trace_header(&sample);
          if(sample.bal.state==BAL_SWING)
          {
            log_swing(&sample);
            if(!started || (uint32_t)(now_ms-last_config_ms)>=LOG_CONFIG_MS)
            {
              log_balance(&sample);log_encoder(sample.ms);last_config_ms=now_ms;
            }
            if(!s_tx_overflow)started=1U;
          }
          else
          {
          if(!started && g_swing_phase==3U) log_swing(&sample);
          if (!started || (uint32_t)(now_ms-last_config_ms)>=LOG_CONFIG_MS)
          {
            log_balance(&sample); log_velocity(&sample); log_encoder(sample.ms);
            last_config_ms=now_ms;
          }
          if(!started || (uint32_t)(now_ms-last_motion_ms)>=100U)
          {
            log_motion(sample.ms); last_motion_ms=now_ms;
          }
          log_trace(&sample);
          if (!s_tx_overflow) started=1U;
          }
        }
        else
        {
        send_schema = ((started == 0U) ||
                      ((uint32_t)(now_ms - last_schema_ms) >= LOG_SCHEMA_MS));
        send_config = ((started == 0U) ||
                      ((uint32_t)(now_ms - last_config_ms) >= LOG_CONFIG_MS) ||
                      (log_config_changed(&sample, &last_config) != 0U));
        if (send_schema != 0U) { log_schema(); log_balance(&sample); }
        if (send_schema == 0U)
        {
          if (send_config != 0U) { log_config(&sample); log_status(sample.ms); log_wheel_pid(&sample); log_motor(&sample); log_encoder(sample.ms); }
          log_data(&sample);
          log_feedback(&sample);
          log_angle(&sample);
          log_balance(&sample);
          log_velocity(&sample);
          if(sample.bal.state==BAL_SWING) log_swing(&sample);
          if(!started || (uint32_t)(now_ms-last_motion_ms)>=100U)
          {
            log_motion(sample.ms); last_motion_ms=now_ms;
          }
        }
        if (s_tx_overflow == 0U)
        {
          if (send_schema != 0U) { last_schema_ms = now_ms; }
          if (send_config != 0U && send_schema == 0U)
          {
            last_config_ms = now_ms;
            last_config = sample;
          }
          started = 1U;
        }
        }
      }
      if (s_tx_length != 0U)
      {
        if (s_tx_overflow == 0U)
        {
          /* ControlTask may have exited while this lower-priority task formatted
           * the frame. Recheck at publication; never queue a new stale log. */
          taskENTER_CRITICAL();
          if (!parameter_packet || (g_bal_enable && g_bal_state==sample.bal.state &&
              (g_bal_state == BAL_ACTIVE || g_bal_state == BAL_SWING)))
          {
            log_start_dma();
            if(exit_packet) Balance_AckExit(exit_ms);
          }
          taskEXIT_CRITICAL();
        }
        else { g_uart_log_dropped++; }
      }
      g_uart_log_stack_free_bytes =
          (uint32_t)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);
    }
    deadline += LOG_SERVICE_MS; /* service RX independently of the log period */
    if ((int32_t)(osKernelGetTickCount() - deadline) >= 0)
    {
      deadline = osKernelGetTickCount() + LOG_SERVICE_MS;
    }
    (void)osDelayUntil(deadline);
  }
}
