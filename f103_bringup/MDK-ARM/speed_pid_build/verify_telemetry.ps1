# Test the production CSV formatter on the host, without MCU peripherals.
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$taskSource = [IO.File]::ReadAllText((Join-Path $taskRoot 'Core\Src\app_telemetry.c'))
$taskStateStart = $taskSource.IndexOf('#define LOG_BAUD')
$taskStateEnd = $taskSource.IndexOf('void AppTelemetry_Init(void)')
$taskFormatStart = $taskSource.IndexOf('static void log_char(char c)')
$taskFormatEnd = $taskSource.IndexOf('static void log_start_dma(void)')
if (($taskStateStart -lt 0) -or ($taskFormatStart -lt 0)) { throw 'Production formatter markers missing' }
$taskTestDir = Join-Path $PSScriptRoot 'host_tests'
New-Item -ItemType Directory -Path $taskTestDir -Force | Out-Null
$taskPrefix = @'
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <float.h>
#include "balance_control.h"
#define APP_WHEEL_COUNT 4U
#define APP_MOTOR_PWM_SLOTS 64U
#define APP_ENCODER_QUALIFY_US 100U
#define taskENTER_CRITICAL() do {} while (0)
#define taskEXIT_CRITICAL() do {} while (0)
volatile uint32_t g_enc_irq[4],g_enc_accepted[4],g_enc_glitch[4],g_enc_duplicate[4];
volatile float g_bal_vel_target_rpm,g_bal_vel_command_rpm,g_bal_vel_rpm,g_bal_position_rev,g_bal_seq_goal_rev;
volatile uint8_t g_bal_seq_state;
volatile uint32_t g_bal_seq_ms;
volatile float g_bal_turn_target_rpm,g_bal_turn_command_rpm,g_bal_turn_rpm,g_bal_turn_output,g_bal_turn_kp;
volatile float g_bal_turn_ki,g_bal_turn_integral;
volatile float g_bal_pos_kp,g_bal_pos_max_rpm;
volatile float g_bal_robust_rpm;
volatile float g_swing_limit,g_swing_travel_rev,g_swing_gain,g_swing_omega2,g_swing_energy,g_swing_capture_rate,g_swing_capture_deg;
volatile float g_bal_entry_deg;
volatile uint32_t g_swing_max_ms;
volatile uint8_t g_swing_phase;
volatile float g_speed_estimated_rpm[4];
#define AS5600_STATUS_MAGNET_DETECTED 0x20U
#define AS5600_STATUS_MAGNET_WEAK 0x10U
#define AS5600_STATUS_MAGNET_STRONG 0x08U
volatile uint32_t g_uart_cmd_received,g_uart_cmd_applied,g_uart_cmd_errors,g_uart_cmd_reply_drops;
volatile uint32_t g_as5600_read_time_ms,g_as5600_busy_skips,g_as5600_read_ok,g_as5600_read_errors;
volatile uint8_t g_as5600_valid;
volatile uint32_t g_motor_pwm_hz,g_motor_dma_periods,g_motor_dma_errors,g_motor_dma_late,g_motor_dma_max_cycles;
static BalanceExitEvent test_exit;
uint8_t Balance_PeekExit(BalanceExitEvent *event) {
  if(!test_exit.pending)return 0; *event=test_exit;return 1;
}
'@
$taskTests = @'
static char test_output[LOG_TX_CAPACITY + 1U];
static void reset_output(void) { s_tx_length = 0U; s_tx_overflow = 0U; }
static const char *output(void) {
  uint32_t i;
  for (i=0U; i<s_tx_length; i++) { test_output[i]=(char)s_tx[i]; }
  test_output[s_tx_length]='\0';
  return test_output;
}
static unsigned fields(const char *p) {
  unsigned n=1U;
  while (*p && *p!='\r' && *p!='\n') { if (*p++==',') { n++; } }
  return n;
}
int main(void) {
  LogSnapshot a={0}, b={0};
  uint32_t i, data_bytes, config_bytes, schema_bytes, trace_bytes;
  float invalid;
  uint32_t nan_bits=0x7fc00000U;
  reset_output(); csv_i32(INT32_MIN); csv_u32(UINT32_MAX);
  assert(strcmp(output(),",-2147483648,4294967295")==0);
  reset_output(); csv_fixed(0.001562f,1000000U); csv_fixed(-1.25f,1000U);
  csv_fixed(-0.001f,1000U);
  assert(strcmp(output(),",0.001562,-1.250,-0.001")==0);
  memcpy(&invalid,&nan_bits,sizeof(invalid));
  reset_output(); csv_fixed(invalid,1000U); csv_fixed(FLT_MAX,100U);
  assert(strcmp(output(),",nan,nan")==0);
  {
    uint32_t event_ms=0;
    reset_output();assert(!log_exit_event(&event_ms) && !s_tx_length);
    test_exit.pending=1;test_exit.ms=12345;test_exit.reason=BAL_REASON_SPEED;
    test_exit.peak_rpm=215.25f;test_exit.max_rpm=200;test_exit.overspeed_ms=200;
    test_exit.angle_deg=2.1f;test_exit.run_ms=32000;test_exit.coalesced=0;
    reset_output();assert(log_exit_event(&event_ms) && event_ms==12345);
    assert(fields(strstr(output(),"#X,"))==11 && fields(strstr(output(),"\r\nX,")+2)==11);
    assert(strstr(output(),"X,12345,SPEED,8,215.25,200.00,200,200,2.100,32000,0\r\n")!=NULL);
    assert(s_tx_length<250 && !s_tx_overflow);
    test_exit.reason=BAL_REASON_ANGLE;reset_output();assert(log_exit_event(&event_ms));
    assert(strstr(output(),",ANGLE,2,")!=NULL);test_exit.pending=0;
  }
  a.ms=1000U; a.cycle=200U; a.dt_ms=5U; a.mode=2U; a.enabled=1U; a.active=1U;
  a.fw=0x103C8015U; a.kp=0.001562f; a.ki=0.003905f; a.cpr=468.6f;
  a.clock_hz=72000000U;
  a.wheel[0].target=60.0f; a.wheel[0].current=57.25f;
  a.wheel[0].error=2.75f; a.wheel[0].u=0.1f; a.wheel[0].integral=0.05f;
  a.wheel[0].count=INT32_MIN; a.wheel[0].delta=-3; a.wheel[0].ccr=360U;
  reset_output(); log_data(&a);
  assert(fields(output())==49U);
  assert(strstr(output(),",60.00,57.25,2.75,0.100,0.050,-2147483648,-3,360,0")!=NULL);
  reset_output(); log_config(&a);
  assert(fields(output())==28U);
  assert(strstr(output(),",0x103C8015,0.001562,0.003905,0.000000,468.600,")!=NULL);
  reset_output(); log_schema();
  assert(fields(strstr(output(),"#D,"))==49U);
  assert(fields(strstr(output(),"#C,"))==28U);
  assert(fields(strstr(output(),"#S,"))==22U);
  assert(fields(strstr(output(),"#W,"))==18U);
  assert(fields(strstr(output(),"#F,"))==10U);
  assert(fields(strstr(output(),"#M,"))==21U);
  assert(fields(strstr(output(),"#Q,"))==9U);
  assert(fields(strstr(output(),"#B,"))==25U);
  assert(fields(strstr(output(),"#V,"))==16U);
  assert(fields(strstr(output(),"#P,"))==9U);
  assert(fields(strstr(output(),"#Y,"))==10U);
  assert(fields(strstr(output(),"#H,"))==6U);
  assert(fields(strstr(output(),"#O,"))==5U);
  assert(fields(strstr(output(),"#U,"))==18U);
  assert(!s_tx_overflow && s_tx_length<LOG_TX_CAPACITY);
  schema_bytes=s_tx_length;
  a.as_raw=1024U; a.as_status=32U; a.as_valid=1U;
  reset_output(); log_angle(&a);
  assert(fields(output())==9U && strcmp(output(),"Q,1000,1024,90.000,32,1,1,0,0\r\n")==0);
  a.as_status=0x18U; a.as_valid=0U;
  reset_output(); log_angle(&a);
  assert(strstr(output(),",24,0,0,1,1\r\n")!=NULL);
  a.as_raw=4095U; a.as_status=255U;
  reset_output(); log_angle(&a); assert(strstr(output(),",359.912,")!=NULL);
  a.on_ticks[3]=6U; a.in1_ticks[3]=64U; a.in2_ticks[3]=58U;
  g_motor_pwm_hz=20089U;
  reset_output(); log_motor(&a);
  assert(fields(output())==21U && strstr(output(),",1,64,20089,")!=NULL && strstr(output(),",6,64,58\r\n")!=NULL);
  a.filter_ms[3]=60U; a.fast_rpm[3]=92.68f;
  reset_output(); log_feedback(&a);
  assert(fields(output())==10U && strstr(output(),",60,92.68")!=NULL);
  a.individual[3]=1U; a.pid_kp[3]=0.0008f; a.pid_ki[3]=0.005f;
  reset_output(); log_wheel_pid(&a);
  assert(fields(output())==18U && strstr(output(),",1,0.000800,0.005000,0.000000")!=NULL);
  b=a; assert(log_config_changed(&a,&b)==0U);
  b.enc_sign[0]=1; assert(log_config_changed(&a,&b)==1U);
  b=a; b.map[1]=2; assert(log_config_changed(&a,&b)==1U);
  b=a; b.ki+=0.001f; assert(log_config_changed(&a,&b)==1U);
  b=a; b.pid_ki[3]+=0.001f; assert(log_config_changed(&a,&b)==1U);
  b=a; b.individual[3]=0U; assert(log_config_changed(&a,&b)==1U);
  b=a; b.filter_ms[3]=0U; assert(log_config_changed(&a,&b)==1U);
  a.ms=UINT32_MAX; a.cycle=UINT32_MAX; a.missed=UINT32_MAX; a.late=UINT32_MAX;
  for (i=0U;i<APP_WHEEL_COUNT;i++) {
    a.wheel[i].target=280.0f; a.wheel[i].current=-280.0f; a.wheel[i].error=560.0f;
    a.wheel[i].u=1.0f; a.wheel[i].integral=1.0f;
    a.wheel[i].count=INT32_MIN; a.wheel[i].delta=INT32_MIN;
    a.wheel[i].ccr=3599U; a.wheel[i].dir=1U;
    a.fast_rpm[i]=-280.0f; a.filter_ms[i]=200U;
  }
  a.bal.state=4U; a.bal.angle_deg=359.9f; a.bal.theta_deg=-179.9f;
  a.bal.rate=-200.0f; a.bal.output=-1.0f; a.bal.integral=-1.0f;
  a.bal.kp=100.0f; a.bal.ki=100.0f; a.bal.kd=100.0f;
  a.bal.limit=1.0f; a.bal.entry=45.0f; a.bal.exit=45.0f;
  a.bal.dwell=5000U; a.bal.hold=5000U; a.bal.bottom=4095U; a.bal.upright=2047U;
  a.bal.sign=-1; a.bal.angle_sign=-1; a.bal.zero_samples=200U;
  a.vel.enabled=1U; a.vel.rpm=-280.0f; a.vel.reference=-3.0f;
  a.vel.integral=-3.0f; a.vel.kp=0.05f; a.vel.ki=0.01f; a.vel.limit=3.0f;
  a.vel.guard_rpm=280.0f; a.vel.max_rpm=280.0f;
  a.vel.run_ms=60000U; a.vel.max_run_ms=60000U;
  a.vel.slew=10.0f;
  a.vel.trim=-1.0f;
  a.vel.robust=-280.0f;
  g_swing_phase=2;g_swing_limit=0.35f;g_swing_travel_rev=2;g_swing_gain=0.18f;
  g_swing_omega2=55;g_swing_energy=-1;g_swing_capture_rate=2;g_swing_max_ms=10000;
  g_swing_capture_deg=2;g_bal_entry_deg=8;
  reset_output();log_swing(&a);
  assert(fields(output())==18U && strstr(output(),",-1.000,1.020,")!=NULL);
  assert(strstr(output(),",2.00,2.00\r\n")!=NULL);
  g_bal_entry_deg=0.5f;reset_output();log_swing(&a);
  assert(strstr(output(),",2.00,0.50\r\n")!=NULL);g_bal_entry_deg=8;
  assert(s_tx_length*25U+1800U<11520U*85U/100U);
  b=a;b.bal.state=BAL_SWING;
  reset_output();log_trace_header(&b);log_swing(&b);log_balance(&b);log_encoder(b.ms);
  assert(!s_tx_overflow && s_tx_length<LOG_TX_CAPACITY && strstr(output(),"#F103_SWING_TRACE_V2")!=NULL);
  assert(log_period_ms(1U,1000U)==20U && log_period_ms(0U,5U)==20U);
  assert(log_period_ms(0U,100U)==100U && log_period_ms(0U,2000U)==1000U);
  a.fast_rpm[0]=20; a.fast_rpm[1]=40; a.fast_rpm[2]=-20; a.fast_rpm[3]=0;
  a.vel.robust=10.0f;
  reset_output(); log_trace(&a);
  assert(fields(output())==16U && strstr(output(),",10.00,-3.000,280.00,4,")!=NULL);
  assert(strstr(output(),",20.00,40.00,-20.00,0.00,10.00\r\n")!=NULL);
  for(i=0U;i<APP_WHEEL_COUNT;i++) a.fast_rpm[i]=-280.0f;
  a.vel.robust=-280.0f;
  reset_output(); log_trace(&a); trace_bytes=s_tx_length;
  assert(trace_bytes*50U+1800U < 11520U*85U/100U);
  for(i=0U;i<4U;i++) {
    g_enc_irq[i]=UINT32_MAX; g_enc_accepted[i]=UINT32_MAX;
    g_enc_glitch[i]=UINT32_MAX; g_enc_duplicate[i]=UINT32_MAX;
  }
  reset_output(); log_encoder(a.ms);
  assert(fields(output())==19U && strstr(output(),",100,4294967295,4294967295,")!=NULL);
  assert(s_tx_length<210U);
  g_bal_vel_target_rpm=30;g_bal_vel_command_rpm=-30;g_bal_vel_rpm=280;
  g_bal_position_rev=-1000;g_bal_seq_goal_rev=0.5f;g_bal_seq_state=4;g_bal_seq_ms=UINT32_MAX;
  reset_output(); log_motion(a.ms);
  assert(fields(output())==9U && strstr(output(),",30.00,-30.00,310.00,-1000.000,0.500,4,4294967295")!=NULL);
  assert(fields(strstr(output(),"Y,"))==10U);
  g_bal_turn_target_rpm=3; g_bal_turn_command_rpm=-2;g_bal_turn_rpm=-1;
  g_bal_turn_output=0.08f;g_bal_turn_kp=0.003f;
  g_bal_turn_ki=0.010f;g_bal_turn_integral=0.02f;
  reset_output();log_motion(a.ms);
  assert(strstr(output(),",3.00,-2.00,-1.00,0.080,0.003000,0.120,0.010000,0.020\r\n")!=NULL);
  g_speed_estimated_rpm[0]=-5;g_speed_estimated_rpm[1]=3;
  g_speed_estimated_rpm[2]=4;g_speed_estimated_rpm[3]=2;
  reset_output();log_motion(a.ms);
  assert(fields(strstr(output(),"H,"))==6U && strstr(output(),",-5.00,3.00,4.00,2.00\r\n")!=NULL);
  g_bal_pos_kp=20;g_bal_pos_max_rpm=6;g_bal_seq_state=5;
  g_bal_position_rev=0.1f;g_bal_seq_goal_rev=0;
  reset_output();log_motion(a.ms);
  assert(fields(strstr(output(),"O,"))==5U && strstr(output(),",20.00,6.00,-0.100\r\n")!=NULL);
  reset_output(); log_trace_header(&a); log_balance(&a); log_velocity(&a); log_encoder(a.ms); log_motion(a.ms); log_trace(&a);
  /* Automatic swing capture publishes one phase-3 U alongside first balance packet. */
  g_swing_phase=3U;log_swing(&a);
  assert(!s_tx_overflow && s_tx_length<LOG_TX_CAPACITY);
  assert(strstr(output(),"#F103_BAL_TRACE_V2")!=NULL);
  assert(strstr(output(),"fw=0x103C8015")!=NULL && fields(strstr(output(),"#T,"))==16U);
  printf("PASS: compact trace has 16 fields; first packet %u/2048 bytes; steady <=%u bytes/s at 50Hz.\n",
         s_tx_length,trace_bytes*50U+1800U);
  reset_output(); log_velocity(&a);
  assert(fields(output())==16U && strstr(output(),",1,-280.00,-3.000,-3.000,0.050000,0.010000,3.00,280.00,280.00,60000,60000,10.00,-1.000,-280.00")!=NULL);
  reset_output(); log_balance(&a); assert(fields(output())==25U);
  assert(strstr(output(),",359.900,-179.900,-200.000,-1.000,-1.000,")!=NULL);
  reset_output(); log_schema(); log_balance(&a);
  assert(!s_tx_overflow && s_tx_length<LOG_TX_CAPACITY);
  reset_output(); log_data(&a); log_feedback(&a); log_angle(&a); log_balance(&a); log_velocity(&a); log_motion(a.ms); data_bytes=s_tx_length;
  g_motor_dma_periods=UINT32_MAX; g_motor_dma_errors=UINT32_MAX;
  g_motor_dma_late=UINT32_MAX; g_motor_dma_max_cycles=UINT32_MAX;
  reset_output(); log_config(&a); log_status(a.ms); log_wheel_pid(&a); log_motor(&a); log_encoder(a.ms); config_bytes=s_tx_length;
  assert(fields(strstr(output(),"S,"))==22U);
  assert(data_bytes*10U+config_bytes+schema_bytes/5U < 11520U*85U/100U);
  /* Schema, full GET/data and HELP are independently bounded DMA packets. */
  reset_output(); log_config(&a); log_status(a.ms); log_wheel_pid(&a); log_motor(&a); log_data(&a); log_feedback(&a); log_angle(&a); log_balance(&a); log_velocity(&a); log_encoder(a.ms); log_motion(a.ms);
  assert(s_tx_overflow==0U && s_tx_length<LOG_TX_CAPACITY);
  printf("PASS: legacy column counts retained; E diagnostic has 19 fields, AS5600/balance/velocity telemetry preserved.\n");
  printf("PASS: stressed D+F+Q+B+V=%u bytes; default 10Hz traffic<=%u bytes/s; full GET=%u/%u bytes.\n",
         data_bytes,data_bytes*10U+config_bytes+schema_bytes/5U,s_tx_length,LOG_TX_CAPACITY);
  log_swing(&a);
  assert(!s_tx_overflow && s_tx_length<LOG_TX_CAPACITY);
  printf("PASS: full GET plus swing U=%u/%u bytes.\n",s_tx_length,LOG_TX_CAPACITY);
  reset_output(); for (i=0U;i<LOG_TX_CAPACITY+50U;i++) { log_char('x'); }
  assert(s_tx_overflow==1U && s_tx_length==LOG_TX_CAPACITY);
  printf("PASS: TX overflow is detected without writing past the buffer.\n");
  return 0;
}
'@
$taskC = $taskPrefix + "`r`n" + $taskSource.Substring($taskStateStart,$taskStateEnd-$taskStateStart) + $taskSource.Substring($taskFormatStart,$taskFormatEnd-$taskFormatStart) + $taskTests
[IO.File]::WriteAllText((Join-Path $taskTestDir 'telemetry_formatter_test.c'),$taskC,[Text.UTF8Encoding]::new($false))
$taskVswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
$taskVcvars = & $taskVswhere -utf8 -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find VC\Auxiliary\Build\vcvars64.bat
if (!$taskVcvars) { throw 'MSVC host compiler unavailable' }
$taskEnvironmentLines = & cmd.exe /d /c ('call "' + $taskVcvars + '" >nul && set')
if ($LASTEXITCODE -ne 0) { throw 'MSVC environment setup failed' }
foreach ($taskEnvironmentLine in $taskEnvironmentLines) {
    $taskSeparator = $taskEnvironmentLine.IndexOf('=')
    if ($taskSeparator -gt 0) {
        [Environment]::SetEnvironmentVariable($taskEnvironmentLine.Substring(0,$taskSeparator),$taskEnvironmentLine.Substring($taskSeparator+1),'Process')
    }
}
& cl.exe /nologo /W3 /WX /utf-8 /TC /std:c11 ('/I'+(Join-Path $taskRoot 'Core\Inc')) (Join-Path $taskTestDir 'telemetry_formatter_test.c') ('/Fo:' + (Join-Path $taskTestDir 'telemetry_formatter_test.obj')) ('/Fe:' + (Join-Path $taskTestDir 'telemetry_formatter_test.exe'))
if ($LASTEXITCODE -ne 0) { throw 'Telemetry formatter host build failed' }
& (Join-Path $taskTestDir 'telemetry_formatter_test.exe')
if ($LASTEXITCODE -ne 0) { throw 'Telemetry formatter regression failed' }
