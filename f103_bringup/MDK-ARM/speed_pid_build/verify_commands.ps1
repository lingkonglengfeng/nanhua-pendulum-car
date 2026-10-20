# Compile production command parser and extracted UART framing/formatting code
# on the host. Register stubs exercise circular-buffer arithmetic, not hardware.
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$taskTestDir = Join-Path $PSScriptRoot 'host_tests\commands'
New-Item -ItemType Directory -Path $taskTestDir -Force | Out-Null
$taskStubHeader = @'
#ifndef APP_VEHICLE_H
#define APP_VEHICLE_H
#include <stdint.h>
#include "balance_control.h"
#define APP_WHEEL_COUNT 4U
#define APP_MOTOR_PWM_SLOTS 64U
#define APP_ENCODER_QUALIFY_US 100U
#define AS5600_STATUS_MAGNET_DETECTED 0x20U
#define AS5600_STATUS_MAGNET_WEAK 0x10U
#define AS5600_STATUS_MAGNET_STRONG 0x08U
extern volatile float g_speed_target[4],g_spd_kp,g_spd_ki,g_spd_kd;
extern volatile float g_enc_counts_per_rev,g_speed_tolerance_pct;
extern volatile uint8_t g_speed_reset,g_speed_enable,g_motor_test_enable,g_bal_enable,g_speed_led_wheel;
extern volatile uint8_t g_motor_test_wheel,g_motor_test_pwm_pct,g_motor_test_dir;
extern volatile int16_t g_drive_test;
extern volatile int8_t g_enc_map[4],g_enc_sign[4],g_mot_sign[4];
extern volatile uint8_t g_spd_individual[4];
extern volatile float g_spd_kp_w[4],g_spd_ki_w[4],g_spd_kd_w[4];
extern volatile float g_spd_kp_used[4],g_spd_ki_used[4],g_spd_kd_used[4];
extern volatile float g_spd_integral[4],g_spd_u[4],g_speed_current[4];
void App_ResetSpeedWheel(uint8_t wheel);
extern volatile uint8_t g_pendulum_zero_request,g_pendulum_calibrated;
extern volatile uint8_t g_pendulum_angle_valid,g_as5600_valid;
extern volatile float g_speed_fast[4];
extern volatile int8_t g_pendulum_sign_request;
extern volatile uint32_t g_pendulum_zero_result;
#endif
'@
[IO.File]::WriteAllText((Join-Path $taskTestDir 'app_vehicle.h'),$taskStubHeader)
[IO.File]::WriteAllText((Join-Path $taskTestDir 'FreeRTOS.h'),"#include <stdint.h>`n")
[IO.File]::WriteAllText((Join-Path $taskTestDir 'task.h'),"#define taskENTER_CRITICAL() do {} while (0)`n#define taskEXIT_CRITICAL() do {} while (0)`n")
$taskSource = [IO.File]::ReadAllText((Join-Path $taskRoot 'Core\Src\app_telemetry.c'))
function Get-TaskSection([string]$Start,[string]$End) {
    $taskBegin = $taskSource.IndexOf($Start)
    $taskFinish = $taskSource.IndexOf($End,$taskBegin)
    if ($taskBegin -lt 0 -or $taskFinish -lt 0) { throw "Section missing: $Start" }
    $taskSource.Substring($taskBegin,$taskFinish-$taskBegin)
}
$taskPrefix = @'
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include "app_vehicle.h"
#include "app_commands.h"
#include "pendulum_angle.h"
#include "task.h"
/* Production code contains a 32-bit MCU bus address cast; host is 64-bit. */
#pragma warning(disable:4311)
volatile float g_speed_target[4],g_spd_kp,g_spd_ki,g_spd_kd;
volatile float g_enc_counts_per_rev,g_speed_tolerance_pct;
volatile uint8_t g_speed_reset,g_speed_enable,g_motor_test_enable,g_speed_led_wheel;
volatile uint8_t g_pendulum_zero_request,g_pendulum_calibrated;
volatile uint8_t g_pendulum_angle_valid;
volatile int8_t g_pendulum_sign_request;
volatile uint32_t g_pendulum_zero_result;
volatile uint8_t g_motor_test_wheel,g_motor_test_pwm_pct,g_motor_test_dir;
volatile int16_t g_drive_test;
volatile int8_t g_enc_map[4],g_enc_sign[4],g_mot_sign[4];
volatile uint8_t g_spd_individual[4];
volatile float g_spd_kp_w[4],g_spd_ki_w[4],g_spd_kd_w[4];
volatile float g_spd_kp_used[4],g_spd_ki_used[4],g_spd_kd_used[4];
volatile float g_spd_integral[4],g_spd_u[4],g_speed_current[4];
volatile float g_speed_fast[4];
volatile float g_speed_estimated_rpm[4];
volatile uint32_t g_speed_filter_ms[4],g_speed_filter_used_ms[4];
static float s_spd_prev_measured[4];
volatile uint32_t g_as5600_read_time_ms,g_as5600_busy_skips,g_as5600_read_ok,g_as5600_read_errors;
volatile uint8_t g_as5600_valid;
volatile uint32_t g_motor_pwm_hz,g_motor_dma_periods,g_motor_dma_errors,g_motor_dma_late,g_motor_dma_max_cycles;
volatile uint32_t g_enc_irq[4],g_enc_accepted[4],g_enc_glitch[4],g_enc_duplicate[4];
typedef struct { uint32_t CCR,CMAR,CNDTR; } TestChannel;
typedef struct { uint32_t ISR,IFCR; } TestDma;
static TestChannel test_channel;
static TestDma test_dma;
#define DMA1_Channel3 (&test_channel)
#define DMA1 (&test_dma)
#define DMA_CCR_EN 1U
#define DMA_IFCR_CGIF3 0x100U
#define DMA_ISR_TCIF3 0x200U
#define DMA1_Channel3_IRQn 13U
#define HAL_NVIC_ClearPendingIRQ(x) ((void)(x))
#define __DMB() ((void)0)
'@
$taskTests = @'
static void send_text(const char *p) { while (*p) { AppCommands_FeedByte((uint8_t)*p++); } }
static void expect(AppCommandType type, AppCommandStatus status) {
  AppCommandReply reply;
  assert(AppCommands_PopReply(&reply));
  assert(reply.type==type && reply.status==status);
}
static void empty_replies(void) { AppCommandReply reply; assert(!AppCommands_PopReply(&reply)); }
static void apply(const char *line, AppCommandType type) {
  send_text(line); AppCommands_Apply(); expect(type,APP_CMD_OK); empty_replies();
}
static void bad(const char *line, AppCommandType type) {
  send_text(line); AppCommands_Apply(); expect(type,APP_CMD_BAD_ARGS); empty_replies();
}
int main(void) {
  uint32_t n, old_drops, old_overruns;
  float fast_energy=0.0f, filtered_energy=0.0f;
  uint8_t help,get,as_query;
  LogSnapshot sample={0};
  assert(g_uart_log_enable==1U && g_uart_bal_trace==1U);
  assert(g_bal_state==BAL_OFF && !g_bal_enable && !g_pendulum_calibrated);
  assert(g_bal_kp==8.4f && g_bal_ki==0.0f && g_bal_kd==0.17f);
  assert(g_bal_vel_enable==1U && g_bal_vel_kp==0.040f && g_bal_vel_ki==0.0015f);
  assert(g_bal_trim_deg==-0.81f);
  g_bal_trim_deg=0.0f; /* Explicit zero-trim fixture for legacy command scenarios. */
  assert(g_bal_max_rpm==200.0f && g_bal_max_run_ms==0U);
  assert(g_bal_entry_deg==8.0f && g_bal_exit_deg==30.0f && g_bal_limit==0.80f);
  /* AS5600 is read-only in every state and independent of LOG/calibration. */
  {
    uint8_t state,log_enabled;
    char packet[LOG_TX_CAPACITY+1U];
    sample.ms=1000;sample.fw=0x103C8042U;sample.as_raw=1024;
    sample.as_status=39;sample.as_valid=1;
    for(state=BAL_OFF;state<=BAL_SWING;state++)
    {
      for(log_enabled=0;log_enabled<=1;log_enabled++)
      {
        g_bal_state=state;g_bal_enable=(state==BAL_ACTIVE||state==BAL_SWING);
        g_uart_log_enable=log_enabled;g_speed_enable=1;g_motor_test_enable=1;
        g_drive_test=25;g_speed_reset=0;g_pendulum_zero_request=0;
        g_speed_target[0]=17;g_bal_output=0.125f;
        send_text("as5600\r\n");AppCommands_Apply();
        s_tx_length=0;s_tx_overflow=0;
        assert(log_replies(&help,&get,&as_query)==1 && as_query && !help && !get);
        assert(g_bal_state==state && g_bal_enable==(state==BAL_ACTIVE||state==BAL_SWING));
        assert(g_uart_log_enable==log_enabled && g_speed_enable && g_motor_test_enable);
        assert(g_drive_test==25 && !g_speed_reset && !g_pendulum_zero_request);
        assert(g_speed_target[0]==17 && g_bal_output==0.125f);
        /* Query data is a distinct DMA packet, including when OFF + LOG=0. */
        s_tx_length=0;log_as5600(&sample);
        assert(!s_tx_overflow && s_tx_length<LOG_TX_CAPACITY);
        for(n=0;n<s_tx_length;n++)packet[n]=(char)s_tx[n];packet[s_tx_length]='\0';
        assert(strstr(packet,"C,1000,0x103C8042,")!=NULL);
        assert(strstr(packet,"\r\nS,1000,")!=NULL);
        assert(strstr(packet,"\r\nQ,1000,1024,90.000,39,1,1,0,0\r\n")!=NULL);
        assert(strstr(packet,"\r\nD,")==NULL && strstr(packet,"\r\nB,")==NULL);
        empty_replies();
      }
    }
    sample.as_valid=0;s_tx_length=0;log_as5600(&sample);
    for(n=0;n<s_tx_length;n++)packet[n]=(char)s_tx[n];packet[s_tx_length]='\0';
    assert(strstr(packet,"Q,1000,1024,90.000,39,0,1,0,0\r\n")!=NULL);
    bad("AS5600 1\n",APP_CMD_AS5600);
    bad("AS5600 GET\n",APP_CMD_AS5600);
    g_bal_state=BAL_OFF;g_bal_enable=0;g_uart_log_enable=1;
    g_speed_enable=0;g_motor_test_enable=0;g_drive_test=0;
    g_speed_target[0]=0;g_bal_output=0;
    memset(&sample,0,sizeof(sample));
  }
  apply("BALTURNPI 0.008 0.010\n",APP_CMD_BALTURNPI);
  assert(g_bal_turn_kp==0.008f && g_bal_turn_ki==0.010f && !g_bal_turn_integral);
  bad("BALTURNPI nan 0.01\n",APP_CMD_BALTURNPI);
  bad("BALTURNPI 0.021 0.01\n",APP_CMD_BALTURNPI);
  bad("BALTURNPI 0.008 0.051\n",APP_CMD_BALTURNPI);
  bad("BALTURNPI 0.008\n",APP_CMD_BALTURNPI);
  send_text("ARM\n"); AppCommands_Apply(); expect(APP_CMD_ARM,APP_CMD_NOT_READY); empty_replies();
  send_text("BALSPD 5\n"); AppCommands_Apply(); expect(APP_CMD_BALSPD,APP_CMD_NOT_READY); empty_replies();
  send_text("BALSPIN 3\n"); AppCommands_Apply(); expect(APP_CMD_BALSPIN,APP_CMD_NOT_READY); empty_replies();
  send_text("SWING\n");AppCommands_Apply();expect(APP_CMD_SWING,APP_CMD_NOT_READY);empty_replies();
  apply("SWINGCFG 35 2 10000\n",APP_CMD_SWINGCFG);
  assert(g_swing_limit==0.35f && g_swing_travel_rev==2 && g_swing_max_ms==10000);
  apply("SWINGENERGY 0.18 55 2\n",APP_CMD_SWINGENERGY);
  apply("SWINGDIR -1\n",APP_CMD_SWINGDIR);assert(g_swing_kick_sign==-1);
  apply("SWINGDIR 1\n",APP_CMD_SWINGDIR);
  apply("SWINGCATCH 0.5\n",APP_CMD_SWINGCATCH);assert(g_swing_capture_deg==0.5f);
  apply("SWINGCATCH 4\n",APP_CMD_SWINGCATCH);assert(g_swing_capture_deg==4);
  apply("SWINGCATCH 2\n",APP_CMD_SWINGCATCH);assert(g_swing_capture_deg==2);
  bad("SWINGCATCH 0.49\n",APP_CMD_SWINGCATCH);bad("SWINGCATCH 4.01\n",APP_CMD_SWINGCATCH);
  bad("SWINGCATCH nan\n",APP_CMD_SWINGCATCH);bad("SWINGCATCH\n",APP_CMD_SWINGCATCH);
  bad("SWINGDIR 0\n",APP_CMD_SWINGDIR);bad("SWINGCFG 61 2 10000\n",APP_CMD_SWINGCFG);
  bad("SWINGCFG 35 0.1 10000\n",APP_CMD_SWINGCFG);bad("SWINGCFG 35 2 999.5\n",APP_CMD_SWINGCFG);
  bad("SWINGENERGY nan 55 2\n",APP_CMD_SWINGENERGY);bad("SWINGENERGY 0.18 5 2\n",APP_CMD_SWINGENERGY);
  bad("SWING 1\n",APP_CMD_SWING);
  send_text("BALSEQ\n"); AppCommands_Apply(); expect(APP_CMD_BALSEQ,APP_CMD_NOT_READY); empty_replies();
  send_text("BALPOS 0\n"); AppCommands_Apply(); expect(APP_CMD_BALPOS,APP_CMD_NOT_READY); empty_replies();
  send_text("BALHOLD\n"); AppCommands_Apply(); expect(APP_CMD_BALHOLD,APP_CMD_NOT_READY); empty_replies();
  send_text("BALPZERO\n"); AppCommands_Apply(); expect(APP_CMD_BALPZERO,APP_CMD_NOT_READY); empty_replies();
  assert(g_bal_state==BAL_OFF && !g_bal_enable);
  apply("BALPID 1.2 0 0.08\n",APP_CMD_BALPID);
  assert(g_bal_kp==1.2f && g_bal_ki==0.0f && g_bal_kd==0.08f);
  bad("BALPID 1 nan 0\n",APP_CMD_BALPID); assert(g_bal_kp==1.2f);
  apply("BALWIN 5 20 400\n",APP_CMD_BALWIN);
  bad("BALWIN 5 4 400\n",APP_CMD_BALWIN); bad("BALWIN 5 20 399\n",APP_CMD_BALWIN);
  bad("BALWIN 5 20 400.5\n",APP_CMD_BALWIN); bad("BALLIMIT 0\n",APP_CMD_BALLIMIT);
  apply("BALLIMIT 25\n",APP_CMD_BALLIMIT); assert(g_bal_limit==0.25f);
  apply("BALSAFE 45 2000\n",APP_CMD_BALSAFE);
  assert(g_bal_max_rpm==45.0f && g_bal_max_run_ms==2000U && g_bal_state==BAL_OFF);
  bad("BALSAFE 9 1500\n",APP_CMD_BALSAFE); bad("BALSAFE 281 1500\n",APP_CMD_BALSAFE);
  bad("BALSAFE 40 1\n",APP_CMD_BALSAFE); bad("BALSAFE 40 499\n",APP_CMD_BALSAFE);
  bad("BALSAFE 40 60001\n",APP_CMD_BALSAFE); bad("BALSAFE 40 1500.5\n",APP_CMD_BALSAFE);
  bad("BALSAFE nan 1500\n",APP_CMD_BALSAFE); bad("BALSAFE 40\n",APP_CMD_BALSAFE);
  assert(g_bal_max_rpm==45.0f && g_bal_max_run_ms==2000U);
  apply("BALSAFE 40 0\n",APP_CMD_BALSAFE); assert(g_bal_max_run_ms==0U);
  apply("BALSAFE 0 0\n",APP_CMD_BALSAFE);
  assert(g_bal_max_rpm==0 && g_bal_max_run_ms==0 && g_bal_state==BAL_OFF);
  bad("BALSAFE -1 0\n",APP_CMD_BALSAFE);bad("BALSAFE 0.1 0\n",APP_CMD_BALSAFE);
  bad("BALSAFE 0 499\n",APP_CMD_BALSAFE);bad("BALSAFE inf 0\n",APP_CMD_BALSAFE);
  assert(g_bal_max_rpm==0 && g_bal_max_run_ms==0);
  apply("BALSAFE 0 1500\n",APP_CMD_BALSAFE);assert(g_bal_max_rpm==0 && g_bal_max_run_ms==1500);
  apply("BALSAFE 40 1500\n",APP_CMD_BALSAFE);
  apply("BALVEL 0.008 0.002 2\n",APP_CMD_BALVEL);
  assert(g_bal_vel_enable && g_bal_vel_kp==0.008f && g_bal_vel_ki==0.002f);
  assert(g_bal_vel_limit_deg==2.0f && g_bal_state==BAL_OFF);
  bad("BALVEL -0.001 0 2\n",APP_CMD_BALVEL); bad("BALVEL 0.051 0 2\n",APP_CMD_BALVEL);
  bad("BALVEL 0.008 0.011 2\n",APP_CMD_BALVEL); bad("BALVEL 0 0 0\n",APP_CMD_BALVEL);
  bad("BALVEL 0 0 3.1\n",APP_CMD_BALVEL); bad("BALVEL nan 0 2\n",APP_CMD_BALVEL);
  bad("BALVEL 0.008 0.002\n",APP_CMD_BALVEL);
  assert(g_bal_vel_enable && g_bal_vel_kp==0.008f && g_bal_vel_ki==0.002f);
  apply("BALVEL 0 0 2\n",APP_CMD_BALVEL); assert(!g_bal_vel_enable);
  apply("BALSLEW 10\n",APP_CMD_BALSLEW);
  assert(g_bal_vel_slew_deg_s==10.0f && g_bal_state==BAL_OFF);
  bad("BALSLEW 0\n",APP_CMD_BALSLEW); bad("BALSLEW 10.1\n",APP_CMD_BALSLEW);
  bad("BALSLEW nan\n",APP_CMD_BALSLEW); bad("BALSLEW 2 3\n",APP_CMD_BALSLEW);
  assert(g_bal_vel_slew_deg_s==10.0f);
  apply("BALSLEW 1\n",APP_CMD_BALSLEW);
  apply("BALTRACE 1\n",APP_CMD_BALTRACE);
  assert(g_uart_bal_trace==1U && g_bal_state==BAL_OFF && !g_bal_enable);
  bad("BALTRACE 2\n",APP_CMD_BALTRACE); bad("BALTRACE -1\n",APP_CMD_BALTRACE);
  bad("BALTRACE 1 1\n",APP_CMD_BALTRACE); assert(g_uart_bal_trace==1U);
  apply("BALTRACE 0\n",APP_CMD_BALTRACE); assert(!g_uart_bal_trace);
  apply("BALTRIM 0.1\n",APP_CMD_BALTRIM);
  assert(g_bal_trim_deg==0.1f && g_bal_state==BAL_OFF && !g_bal_enable);
  bad("BALTRIM 1.01\n",APP_CMD_BALTRIM); bad("BALTRIM -1.01\n",APP_CMD_BALTRIM);
  bad("BALTRIM nan\n",APP_CMD_BALTRIM); bad("BALTRIM 0 0\n",APP_CMD_BALTRIM);
  assert(g_bal_trim_deg==0.1f);
  apply("BALTRIM -0.1\n",APP_CMD_BALTRIM); assert(g_bal_trim_deg==-0.1f);
  apply("ZERO\n",APP_CMD_ZERO); assert(g_bal_trim_deg==-0.1f);
  apply("STOP\n",APP_CMD_STOP);
  apply("BALTRIM 0\n",APP_CMD_BALTRIM); assert(g_bal_trim_deg==0.0f);
  apply("BALSIGN -1\n",APP_CMD_BALSIGN); assert(g_bal_sign==-1);
  bad("BALSIGN 0\n",APP_CMD_BALSIGN);
  apply("BALSIGN 1\n",APP_CMD_BALSIGN);
  apply("PENDSIGN -1\n",APP_CMD_PENDSIGN); assert(g_pendulum_sign_request==-1);
  apply("ZERO\n",APP_CMD_ZERO); assert(g_pendulum_zero_request==1U && !g_speed_enable);
  send_text("ARM\n"); AppCommands_Apply(); expect(APP_CMD_ARM,APP_CMD_NOT_READY); empty_replies();
  apply("STOP\n",APP_CMD_STOP); assert(!g_pendulum_zero_request && g_bal_state==BAL_OFF);
  g_pendulum_calibrated=1U; g_pendulum_zero_result=2U;
  apply("ARM\n",APP_CMD_ARM); assert(g_bal_state==BAL_WAIT_TOP && !g_bal_enable);
  send_text("BALSEQ\n"); AppCommands_Apply(); expect(APP_CMD_BALSEQ,APP_CMD_NOT_READY); empty_replies();
  send_text("BALTURN 3\n"); AppCommands_Apply(); expect(APP_CMD_BALTURN,APP_CMD_NOT_READY); empty_replies();
  apply("BALVEL 0.038 0.0009 3\n",APP_CMD_BALVEL);
  apply("ARM\n",APP_CMD_ARM);
  for(n=0U;n<=400U;n+=5U) Balance_Update(0,0,0,1,1,0,50000U+n);
  assert(g_bal_state==BAL_ACTIVE && g_bal_enable);
  assert(g_bal_seq_state==BAL_SEQ_HOLD && !g_bal_position_rev && !g_bal_seq_goal_rev);
  apply("BALSPIN 3\n",APP_CMD_BALSPIN);
  assert(g_bal_seq_state==BAL_SEQ_SPIN && g_bal_turn_target_rpm==3 && !g_bal_seq_goal_rev);
  Balance_AddPosition(0.1f);apply("BALSPIN 0\n",APP_CMD_BALSPIN);
  assert(g_bal_seq_state==BAL_SEQ_SPIN && !g_bal_turn_target_rpm && !g_bal_seq_goal_rev);
  assert(g_bal_position_rev==0.1f);
  /* SWING takes over from speed mode only after successful precondition check. */
  bad("BALSPIN nan\n",APP_CMD_BALSPIN);bad("BALSPIN 11\n",APP_CMD_BALSPIN);
  bad("BALSPIN -11\n",APP_CMD_BALSPIN);bad("BALSPIN 1 2\n",APP_CMD_BALSPIN);
  apply("BALPZERO\n",APP_CMD_BALPZERO);
  apply("BALPOSCFG 20 6\n",APP_CMD_BALPOSCFG);
  assert(g_bal_pos_kp==20 && g_bal_pos_max_rpm==6);
  bad("BALPOSCFG nan 6\n",APP_CMD_BALPOSCFG);bad("BALPOSCFG 0 6\n",APP_CMD_BALPOSCFG);
  bad("BALPOSCFG 20 16\n",APP_CMD_BALPOSCFG);bad("BALPOSCFG 20\n",APP_CMD_BALPOSCFG);
  Balance_AddPosition(0.1f);
  apply("BALPOS 0\n",APP_CMD_BALPOS);
  assert(g_bal_seq_state==BAL_SEQ_HOLD && g_bal_position_rev==0.1f && !g_bal_seq_goal_rev);
  bad("BALPOS 5.01\n",APP_CMD_BALPOS);bad("BALPOS nan\n",APP_CMD_BALPOS);
  bad("BALPOS 0 1\n",APP_CMD_BALPOS);bad("BALHOLD 0\n",APP_CMD_BALHOLD);
  apply("BALHOLD\n",APP_CMD_BALHOLD);assert(g_bal_seq_goal_rev==0.1f);
  apply("BALPZERO\n",APP_CMD_BALPZERO);
  assert(!g_bal_position_rev && !g_bal_seq_goal_rev && g_bal_seq_state==BAL_SEQ_HOLD && g_bal_enable);
  apply("BALTURN 3\n",APP_CMD_BALTURN);assert(g_bal_seq_state==BAL_SEQ_NONE);
  send_text("BALPOS 0\n");AppCommands_Apply();expect(APP_CMD_BALPOS,APP_CMD_BAD_ARGS);empty_replies();
  apply("BALTURN 0\n",APP_CMD_BALTURN);
  g_bal_vel_integral_deg=0.04f;
  apply("BALSPD 5\n",APP_CMD_BALSPD);
  assert(g_bal_enable && g_bal_vel_target_rpm==5.0f && g_bal_vel_integral_deg==0.04f);
  bad("BALSPD 31\n",APP_CMD_BALSPD); bad("BALSPD nan\n",APP_CMD_BALSPD);
  bad("BALSPD 1 2\n",APP_CMD_BALSPD);
  assert(g_bal_vel_target_rpm==5.0f);
  send_text("BALSPD 21\n"); AppCommands_Apply(); expect(APP_CMD_BALSPD,APP_CMD_BAD_ARGS); empty_replies();
  assert(g_bal_vel_target_rpm==5.0f); /* 40RPM guard allows at most 20RPM target. */
  apply("BALSEQ\n",APP_CMD_BALSEQ);
  assert(g_bal_seq_state==BAL_SEQ_OUT && g_bal_seq_goal_rev==0.25f && g_bal_enable);
  bad("BALSEQ 0 0.25 2000\n",APP_CMD_BALSEQ); bad("BALSEQ 0.5 0.25 2000\n",APP_CMD_BALSEQ);
  bad("BALSEQ 21 0.25 2000\n",APP_CMD_BALSEQ); bad("BALSEQ 8 0.51 2000\n",APP_CMD_BALSEQ);
  bad("BALSEQ 8 nan 2000\n",APP_CMD_BALSEQ); bad("BALSEQ 8 0.25 400.5\n",APP_CMD_BALSEQ);
  bad("BALSEQ 8 0.25 399\n",APP_CMD_BALSEQ); bad("BALSEQ 8 0.25\n",APP_CMD_BALSEQ);
  assert(g_bal_seq_goal_rev==0.25f && g_bal_seq_state==BAL_SEQ_OUT);
  apply("BALSEQ -8 0.1 500\n",APP_CMD_BALSEQ);
  assert(g_bal_seq_goal_rev==-0.1f);
  apply("BALSPD 0\n",APP_CMD_BALSPD);
  assert(g_bal_seq_state==BAL_SEQ_NONE && g_bal_enable);
  apply("BALTURN 3\n",APP_CMD_BALTURN);
  assert(g_bal_turn_target_rpm==3 && g_bal_enable);
  g_bal_turn_integral=0.01f;
  apply("BALTURNPI 0.008 0.010\n",APP_CMD_BALTURNPI);
  assert(g_bal_enable && g_bal_turn_target_rpm==3 && !g_bal_turn_integral);
  bad("BALTURN 11\n",APP_CMD_BALTURN); bad("BALTURN nan\n",APP_CMD_BALTURN);
  bad("BALTURN -11\n",APP_CMD_BALTURN); bad("BALTURN 1 2\n",APP_CMD_BALTURN);
  send_text("BALSEQ\n"); AppCommands_Apply(); expect(APP_CMD_BALSEQ,APP_CMD_BAD_ARGS); empty_replies();
  apply("BALTURN -3\n",APP_CMD_BALTURN); assert(g_bal_turn_target_rpm==-3);
  apply("BALTURN 0\n",APP_CMD_BALTURN); assert(!g_bal_turn_target_rpm);
  apply("BALSEQ\n",APP_CMD_BALSEQ);
  apply("BALTURN 3\n",APP_CMD_BALTURN);
  assert(g_bal_seq_state==BAL_SEQ_NONE && g_bal_vel_target_rpm==0 && g_bal_enable);
  apply("STOP\n",APP_CMD_STOP);
  assert(!g_bal_vel_target_rpm && !g_bal_vel_command_rpm && !g_bal_enable && !g_bal_turn_target_rpm && !g_bal_turn_output);
  apply("ARM\n",APP_CMD_ARM);
  for(n=0U;n<=400U;n+=5U) Balance_Update(0,0,0,1,1,0,51000U+n);
  apply("BALSEQ\n",APP_CMD_BALSEQ);
  Balance_Update(0,0,0,1,1,0,51405U);
  assert(g_bal_seq_state==BAL_SEQ_OUT && g_bal_vel_target_rpm>0);
  apply("FWD 30\n",APP_CMD_FWD);
  assert(g_bal_state==BAL_OFF && g_speed_enable && g_speed_target[0]==30 && g_speed_target[3]==30);
  assert(!g_bal_vel_target_rpm && !g_bal_vel_command_rpm && g_bal_seq_state==BAL_SEQ_NONE);
  g_speed_reset=0U; g_spd_integral[0]=0.15f;
  apply("FWD 40\n",APP_CMD_FWD); assert(!g_speed_reset && g_spd_integral[0]==0.15f);
  apply("REV 30\n",APP_CMD_REV); assert(g_speed_target[0]==-30 && g_speed_target[3]==-30);
  apply("MOVE -60\n",APP_CMD_MOVE); assert(g_speed_target[2]==-60);
  bad("FWD -30\n",APP_CMD_FWD); bad("MOVE 281\n",APP_CMD_MOVE);
  apply("FWD 0\n",APP_CMD_FWD); assert(!g_speed_enable);
  apply("BALWIN 20 20 400\n",APP_CMD_BALWIN); assert(g_bal_entry_deg==20 && g_bal_exit_deg==20);
  apply("BALWIN 5 20 400\n",APP_CMD_BALWIN);
  apply("ZERO\n",APP_CMD_ZERO); apply("RUN 1\n",APP_CMD_RUN);
  assert(!g_pendulum_zero_request && g_bal_state==BAL_OFF && g_speed_enable);
  apply("STOP\n",APP_CMD_STOP);
  g_speed_filter_ms[3]=60U;
  for(n=0;n<400;n++) { s_speed_feedback(3,60.0f,0.005f); }
  assert(fabsf(g_speed_current[3]-60.0f)<0.001f);
  for(n=0;n<1000;n++) {
    float fast=60.0f+((n&1U)?20.0f:-20.0f);
    s_speed_feedback(3,fast,0.005f); s_speed_feedback(0,fast,0.005f);
    assert(g_speed_current[0]==fast && g_speed_fast[3]==fast);
    if(n>100U) { fast_energy+=(fast-60.0f)*(fast-60.0f); filtered_energy+=(g_speed_current[3]-60.0f)*(g_speed_current[3]-60.0f); }
  }
  assert(filtered_energy<fast_energy*0.01f);
  App_ResetSpeedWheel(3); assert(g_speed_current[3]>59.0f);
  for(n=0;n<400;n++) { s_speed_feedback(3,-60.0f,(n&1U)?0.004f:0.006f); }
  assert(fabsf(g_speed_current[3]+60.0f)<0.001f);
  for(n=0;n<80;n++) { s_speed_feedback(3,0.0f,0.005f); }
  assert(fabsf(g_speed_current[3])<0.1f);
  g_speed_filter_ms[3]=UINT32_MAX; s_speed_feedback(3,60.0f,0.005f);
  assert(g_speed_filter_used_ms[3]==200U && isfinite(g_speed_current[3]));
  g_speed_filter_ms[3]=0U; s_speed_feedback(3,60.0f,0.005f);
  assert(g_speed_current[3]==60.0f && !g_speed_filter_used_ms[3]);
  send_text("SET 0 6"); AppCommands_Apply(); assert(g_speed_target[0]==0.0f); empty_replies();
  send_text("0\r\n"); assert(g_speed_target[0]==0.0f);
  AppCommands_Apply(); assert(g_speed_target[0]==60.0f); expect(APP_CMD_SET,APP_CMD_OK); empty_replies();
  apply("pid 1.562e-3 0.003905 0\n",APP_CMD_PID);
  assert(fabsf(g_spd_kp-0.001562f)<1e-8f && fabsf(g_spd_ki-0.003905f)<1e-8f);
  assert(g_speed_reset==1U);
  g_drive_test=20; g_motor_test_enable=1U; g_bal_enable=1U;
  apply("RUN 1\n",APP_CMD_RUN);
  assert(g_speed_enable==1U && g_drive_test==0 && !g_motor_test_enable && !g_bal_enable);
  assert(g_speed_target[0]==60.0f);
  g_speed_reset=0U;
  apply("RUN 1\n",APP_CMD_RUN);
  assert(g_speed_reset==0U && g_speed_enable==1U && g_speed_target[0]==60.0f);
  g_motor_test_enable=1U;
  apply("RUN 1\n",APP_CMD_RUN);
  assert(g_speed_reset==1U && !g_motor_test_enable && g_speed_enable==1U);
  g_speed_reset=0U; g_drive_test=20;
  apply("RUN 1\n",APP_CMD_RUN);
  assert(g_speed_reset==1U && g_drive_test==0 && g_speed_enable==1U);
  g_speed_reset=0U; send_text("RESET\nRUN 1\n"); AppCommands_Apply();
  assert(g_speed_reset==1U); expect(APP_CMD_RESET,APP_CMD_OK); expect(APP_CMD_RUN,APP_CMD_OK); empty_replies();
  apply("PID 0.001562 0.01 0\n",APP_CMD_PID);
  g_speed_reset=0U;
  for(n=0U;n<4U;n++) { g_spd_integral[n]=0.2f+(float)n*0.1f; g_spd_u[n]=0.4f; g_speed_current[n]=(float)n*10.0f; }
  send_text("PIDW 3 0.0008 0.005 0\n");
  assert(!g_spd_individual[3]);
  AppCommands_Apply(); expect(APP_CMD_PIDW,APP_CMD_OK); empty_replies();
  assert(g_spd_individual[3] && !g_spd_individual[0] && !g_spd_individual[2]);
  assert(fabsf(g_spd_kp_w[3]-0.0008f)<1e-8f && fabsf(g_spd_ki_w[3]-0.005f)<1e-8f);
  assert(g_spd_integral[3]==0 && g_spd_u[3]==0 && s_spd_prev_measured[3]==30.0f);
  assert(fabsf(g_spd_integral[0]-0.2f)<1e-8f && fabsf(g_spd_integral[2]-0.4f)<1e-8f && g_speed_reset==0U);
  s_speed_pid_select();
  assert(fabsf(g_spd_kp_used[0]-0.001562f)<1e-8f && g_spd_ki_used[2]==0.01f && g_spd_ki_used[3]==0.005f);
  bad("PIDW 4 0.001 0.01 0\n",APP_CMD_PIDW);
  bad("PIDW 3 0.001 nan 0\n",APP_CMD_PIDW);
  bad("PIDW 3 0.001 0.01 0 extra\n",APP_CMD_PIDW);
  assert(g_spd_ki_w[3]==0.005f);
  apply("PIDW 1 0.0009 0.004 0\n",APP_CMD_PIDW);
  apply("KP 0.001\n",APP_CMD_KP); s_speed_pid_select();
  assert(g_spd_kp_used[1]==0.001f && g_spd_kp_used[3]==0.001f && g_spd_ki_used[1]==0.004f && g_spd_ki_used[3]==0.005f);
  apply("PID 0.001562 0.01 0\n",APP_CMD_PID); s_speed_pid_select();
  for(n=0U;n<4U;n++) { assert(!g_spd_individual[n] && g_spd_ki_used[n]==0.01f); }
  apply("TARGET 60 -30 0 12.5\n",APP_CMD_TARGET);
  assert(g_speed_target[0]==60 && g_speed_target[1]==-30 && g_speed_target[3]==12.5f);
  apply("ALL 35\n",APP_CMD_ALL);
  for(n=0U;n<4U;n++) { assert(g_speed_target[n]==35.0f); }
  apply("STOP\n",APP_CMD_STOP);
  assert(!g_speed_enable && !g_bal_enable && !g_motor_test_enable && g_drive_test==0);
  for(n=0U;n<4U;n++) { assert(g_speed_target[n]==0.0f); }
  apply("MAP 1 3\n",APP_CMD_MAP); assert(g_enc_map[1]==3);
  apply("ENCSIGN 0 +1\n",APP_CMD_ENCSIGN); assert(g_enc_sign[0]==1);
  apply("MOTSIGN 0 -1\n",APP_CMD_MOTSIGN); assert(g_mot_sign[0]==-1);
  g_mot_sign[3]=1; g_speed_enable=1; g_bal_enable=1; g_drive_test=20;
  for(n=0;n<4;n++) { g_speed_target[n]=60; }
  apply("OPEN 3 59\n",APP_CMD_OPEN);
  assert(!g_speed_enable && !g_bal_enable && !g_drive_test && g_motor_test_enable);
  assert(g_motor_test_wheel==3 && g_motor_test_pwm_pct==59 && !g_motor_test_dir);
  for(n=0;n<4;n++) { assert(g_speed_target[n]==0); }
  apply("OPEN 3 -59\n",APP_CMD_OPEN); assert(g_motor_test_dir==1 && g_motor_test_pwm_pct==41);
  apply("OPEN 0 59\n",APP_CMD_OPEN); assert(g_motor_test_wheel==0 && g_motor_test_dir==1 && g_motor_test_pwm_pct==41);
  apply("OPEN 3 0\n",APP_CMD_OPEN); assert(!g_motor_test_dir && !g_motor_test_pwm_pct);
  apply("OPEN 3 100\n",APP_CMD_OPEN); assert(!g_motor_test_dir && g_motor_test_pwm_pct==100);
  apply("OPEN 3 -100\n",APP_CMD_OPEN); assert(g_motor_test_dir==1 && !g_motor_test_pwm_pct);
  bad("OPEN 4 59\n",APP_CMD_OPEN); bad("OPEN 3 101\n",APP_CMD_OPEN);
  bad("OPEN 3 -101\n",APP_CMD_OPEN); bad("OPEN 3 59.5\n",APP_CMD_OPEN);
  bad("OPEN 3 nan\n",APP_CMD_OPEN); bad("OPEN 3 59 extra\n",APP_CMD_OPEN);
  assert(g_motor_test_wheel==3 && g_motor_test_dir==1 && !g_motor_test_pwm_pct && g_motor_test_enable);
  apply("STOP\n",APP_CMD_STOP); assert(!g_motor_test_enable);
  apply("CPR 468.6\n",APP_CMD_CPR); assert(fabsf(g_enc_counts_per_rev-468.6f)<0.001f);
  apply("LED 2 5\n",APP_CMD_LED); assert(g_speed_led_wheel==2 && g_speed_tolerance_pct==5);
  apply("PERIOD 100\n",APP_CMD_PERIOD); assert(g_uart_log_period_ms==100);
  apply("KP 0.002\n",APP_CMD_KP); apply("KI 0.004\n",APP_CMD_KI); apply("KD 0\n",APP_CMD_KD);
  apply("RESET\n",APP_CMD_RESET);
  bad("SET 4 20\n",APP_CMD_SET); bad("SET 0 nan\n",APP_CMD_SET);
  bad("SET 0 inf\n",APP_CMD_SET); bad("SET 0 1e999\n",APP_CMD_SET);
  bad("SET 0 0x10\n",APP_CMD_SET); bad("SET 0 60junk\n",APP_CMD_SET);
  bad("ENCSIGN 0 0\n",APP_CMD_ENCSIGN); bad("RUN 1.0\n",APP_CMD_RUN);
  bad("MAP 0 4\n",APP_CMD_MAP); bad("CPR 0\n",APP_CMD_CPR);
  bad("PERIOD 20.5\n",APP_CMD_PERIOD); bad("PID 1 2 nan\n",APP_CMD_PID);
  assert(fabsf(g_spd_kp-0.002f)<1e-8f && fabsf(g_spd_ki-0.004f)<1e-8f);
  bad("SET 0 60 EXTRA\n",APP_CMD_SET);
  send_text("UNRECOGNIZED\n"); expect(APP_CMD_UNKNOWN,APP_CMD_UNKNOWN_NAME); empty_replies();
  for(n=0U;n<100U;n++) { AppCommands_FeedByte('x'); }
  send_text("SET 0 99\n"); AppCommands_Apply();
  expect(APP_CMD_UNKNOWN,APP_CMD_LINE_TOO_LONG); empty_replies(); assert(g_speed_target[0]==0);
  send_text("SET 0 "); AppCommands_FeedByte(0U); send_text("99\n"); AppCommands_Apply();
  expect(APP_CMD_UNKNOWN,APP_CMD_BAD_CHAR); empty_replies(); assert(g_speed_target[0]==0);
  send_text("SET 0 "); AppCommands_RxError(APP_CMD_RX_OVERFLOW); send_text("99\n");
  AppCommands_Apply(); expect(APP_CMD_UNKNOWN,APP_CMD_RX_OVERFLOW); empty_replies();
  apply("SET 0 15\n",APP_CMD_SET); assert(g_speed_target[0]==15);
  apply("LOG 0\n",APP_CMD_LOG); assert(g_uart_log_enable==0U);
  apply("GET\n",APP_CMD_GET); apply("HELP\n",APP_CMD_HELP);
  for(n=0U;n<9U;n++) { send_text("SET 0 20\n"); }
  expect(APP_CMD_SET,APP_CMD_QUEUE_FULL); AppCommands_Apply();
  for(n=0U;n<8U;n++) { expect(APP_CMD_SET,APP_CMD_OK); } empty_replies();
  old_drops=g_uart_cmd_reply_drops;
  for(n=0U;n<17U;n++) { send_text("GET\n"); AppCommands_Apply(); }
  assert(g_uart_cmd_reply_drops==old_drops+1U);
  for(n=0U;n<16U;n++) { expect(APP_CMD_GET,APP_CMD_OK); } empty_replies();
  /* A complete command straddles the circular DMA buffer's wrap boundary. */
  send_text("\n"); s_rx_consumed=506U; s_rx_wraps=0U;
  { const char *p="SET 0 75\n"; for(n=0U;p[n];n++) { s_rx[(506U+n)%512U]=(uint8_t)p[n]; } }
  test_dma.ISR=DMA_ISR_TCIF3; test_channel.CNDTR=512U-(506U+n-512U);
  log_rx_service(); assert(s_rx_consumed==506U+n);
  AppCommands_Apply(); expect(APP_CMD_SET,APP_CMD_OK); empty_replies(); assert(g_speed_target[0]==75);
  /* Pending TC plus transient NDTR=0 means one completed wrap, not two. */
  s_rx_consumed=511U; s_rx_wraps=0U; s_rx[511]='\n'; test_channel.CNDTR=0U;
  old_overruns=g_uart_rx_overruns; log_rx_service();
  assert(s_rx_consumed==512U && g_uart_rx_overruns==old_overruns); empty_replies();
  /* Overrun and UART errors discard partial lines without changing motor mode. */
  test_dma.ISR=0U; test_channel.CNDTR=500U; s_rx_wraps=2U; s_rx_consumed=0U;
  g_speed_enable=1U; log_rx_service(); expect(APP_CMD_UNKNOWN,APP_CMD_RX_OVERFLOW);
  assert(g_speed_enable==1U); send_text("\n");
  s_rx_uart_fault=1U; log_rx_service(); expect(APP_CMD_UNKNOWN,APP_CMD_RX_ERROR);
  assert(g_speed_enable==1U); send_text("\n"); empty_replies();
  /* Replies still serialize while LOG=0. HELP and GET use separate packets. */
  apply("STOP\n",APP_CMD_STOP);
  g_pendulum_calibrated=1;g_pendulum_angle_valid=1;g_as5600_valid=1;
  g_pendulum_zero_result=PENDULUM_ZERO_SUCCESS;
  Balance_Update(3.1415926f,0,3.1415926f,1,1,0,60000);
  for(n=0;n<4;n++)g_speed_fast[n]=0;
  g_speed_enable=1;g_speed_fast[0]=6;
  send_text("SWING\n");AppCommands_Apply();expect(APP_CMD_SWING,APP_CMD_BAD_ARGS);empty_replies();
  assert(g_speed_enable && g_bal_state==BAL_OFF); /* invalid start leaves current mode */
  g_speed_fast[0]=0;
  apply("SWING\n",APP_CMD_SWING);
  assert(g_bal_state==BAL_SWING && g_bal_enable && !g_speed_enable && g_swing_phase==1);
  send_text("BALSPIN 2\n");AppCommands_Apply();expect(APP_CMD_BALSPIN,APP_CMD_NOT_READY);empty_replies();
  assert(g_bal_state==BAL_SWING);
  apply("SWINGCATCH 1.5\n",APP_CMD_SWINGCATCH);
  assert(!g_bal_enable && g_bal_state==BAL_OFF && g_swing_capture_deg==1.5f);
  apply("SWINGCATCH 2\n",APP_CMD_SWINGCATCH);
  apply("STOP\n",APP_CMD_STOP);assert(!g_bal_enable && g_swing_phase==4);
  /* RX DMA remains 512B. Bounded 128B task copies drain a longer burst in two
   * services, retaining a command that begins beyond the first copy. */
  s_rx_consumed=0;s_rx_wraps=0;test_dma.ISR=0;test_channel.CNDTR=LOG_RX_CAPACITY-229;
  for(n=0;n<224;n++)s_rx[n]='\n';
  s_rx[224]='S';s_rx[225]='T';s_rx[226]='O';s_rx[227]='P';s_rx[228]='\n';
  g_speed_enable=1;log_rx_service();AppCommands_Apply();empty_replies();
  assert(s_rx_consumed==128 && g_speed_enable);
  log_rx_service();AppCommands_Apply();expect(APP_CMD_STOP,APP_CMD_OK);empty_replies();
  assert(s_rx_consumed==229 && !g_speed_enable);
  send_text("HELP\nGET\nLOG 0\nSET 0 60\nRESET\nKP 0.002\nKI 0.004\nKD 0\n");
  AppCommands_Apply(); s_tx_length=0U; s_tx_overflow=0U;
  assert(log_replies(&help,&get,&as_query)==8U && help && get);
  log_help(); assert(!s_tx_overflow && s_tx_length<LOG_TX_CAPACITY);
  s_tx_length=0U; log_schema(); assert(!s_tx_overflow && s_tx_length<LOG_TX_CAPACITY);
  s_tx_length=0U; sample.fw=0x103C8015U; log_config(&sample); log_status(sample.ms); log_wheel_pid(&sample); log_motor(&sample); log_data(&sample); log_feedback(&sample); log_angle(&sample); log_balance(&sample); log_velocity(&sample); log_encoder(sample.ms); log_motion(sample.ms);
  assert(!s_tx_overflow && s_tx_length<LOG_TX_CAPACITY && !g_uart_log_enable); empty_replies();
  /* Diagnostic packet must remain usable after reset/rearm and with LOG=0.
   * Exercise real control event storage through the production formatter. */
  {
    BalanceExitEvent exit;
    uint32_t event_ms=0;
    uint32_t packet_length=s_tx_length;
    assert(Balance_PeekExit(&exit));
    assert(!g_uart_log_enable && g_bal_state!=BAL_ACTIVE);
    s_tx_length=0;s_tx_overflow=0;
    assert(log_exit_event(&event_ms) && event_ms==exit.ms);
    assert(!s_tx_overflow && s_tx_length<250);
    Balance_AckExit(event_ms);
    s_tx_length=0;
    assert(!log_exit_event(&event_ms) && !s_tx_length);
    s_tx_length=packet_length;
  }
  /* Metre sequence parsing uses production controller readiness and preserves
   * an accepted sequence when a malformed/repeated request is rejected. */
  apply("STOP\n",APP_CMD_STOP);
  send_text("BALSTEP\n");AppCommands_Apply();expect(APP_CMD_BALSTEP,APP_CMD_NOT_READY);empty_replies();
  bad("BALWHEEL nan\n",APP_CMD_BALWHEEL);bad("BALWHEEL 0\n",APP_CMD_BALWHEEL);
  bad("BALWHEEL 301\n",APP_CMD_BALWHEEL);bad("BALWHEEL 65 1\n",APP_CMD_BALWHEEL);
  apply("BALWHEEL 65\n",APP_CMD_BALWHEEL);assert(g_bal_wheel_diameter_mm==65 && !g_bal_enable);
  bad("BALSTEP 0 8\n",APP_CMD_BALSTEP);bad("BALSTEP nan 8\n",APP_CMD_BALSTEP);
  bad("BALSTEP 1 inf\n",APP_CMD_BALSTEP);bad("BALSTEP 2.01 8\n",APP_CMD_BALSTEP);
  bad("BALSTEP 1 0\n",APP_CMD_BALSTEP);bad("BALSTEP 1 21\n",APP_CMD_BALSTEP);
  bad("BALSTEP 1\n",APP_CMD_BALSTEP);bad("BALSTEP 1 8 5\n",APP_CMD_BALSTEP);
  g_pendulum_calibrated=1;g_bal_vel_enable=1;g_bal_max_run_ms=0;
  g_bal_entry_deg=8;g_bal_exit_deg=30;g_bal_max_rpm=200;
  assert(Balance_Arm(1));
  for(n=0;n<=80;n++)Balance_Update(0,0,0,1,1,0,70000+n*5);
  assert(g_bal_state==BAL_ACTIVE);
  g_bal_wheel_diameter_mm=0;
  send_text("BALSTEP\n");AppCommands_Apply();expect(APP_CMD_BALSTEP,APP_CMD_NOT_READY);empty_replies();
  g_bal_wheel_diameter_mm=65;
  apply("BALSTEP\n",APP_CMD_BALSTEP);assert(g_bal_seq_state==BAL_SEQ_STEP_WAIT);
  bad("BALSTEP\n",APP_CMD_BALSTEP);assert(g_bal_seq_state==BAL_SEQ_STEP_WAIT);
  apply("BALSPD 0\n",APP_CMD_BALSPD);assert(!g_bal_seq_state);
  apply("BALSTEP -1 12\n",APP_CMD_BALSTEP);assert(g_bal_seq_state==BAL_SEQ_STEP_WAIT);
  apply("BALWHEEL 70\n",APP_CMD_BALWHEEL);assert(!g_bal_enable && !g_bal_seq_state && g_bal_wheel_diameter_mm==70);
  printf("PASS: BALWHEEL configuration and BALSTEP default/custom bounds, readiness, atomic rejection and cancellation.\n");
  printf("PASS: AS5600 query in all 7 states with LOG 0/1, read-only application, C/S/Q payload and failed-read validity.\n");
  printf("PASS: commands/STOP, idempotent RUN, mode-transition reset, and explicit RESET preservation.\n");
  printf("PASS: feedback filter preserves rear bypass, DC/sign/stop response, PID-reset continuity, jittered dt and tau bounds.\n");
  printf("PASS: OPEN isolates one wheel, exits speed mode, converts direction/duty and rejects invalid args atomically.\n");
  printf("PASS: PIDW commits one profile, resets only its history, preserves rear gains, validates args and returns to common.\n");
  printf("PASS: malformed/NaN/Inf/hex/suffix rejection, atomic PID, overflow recovery and queues.\n");
  printf("PASS: balance/motion commands, NOT_READY, argument bounds, ZERO/STOP/RUN cancellation.\n");
  printf("PASS: circular RX wrap/NDTR=0, receive faults, LOG-off replies; separate HELP/schema/GET fit; GET=%u/%u bytes.\n",s_tx_length,LOG_TX_CAPACITY);
  return 0;
}
'@
$taskVehicleSource=[IO.File]::ReadAllText((Join-Path $taskRoot 'Core\Src\app_vehicle.c'))
$taskResetStart=$taskVehicleSource.IndexOf('static void s_speed_feedback(uint8_t wheel, float fast_rpm, float dt)')
$taskResetEnd=$taskVehicleSource.IndexOf('/* ======================== 滑动中值', $taskResetStart)
$taskResetCode=$taskVehicleSource.Substring($taskResetStart,$taskResetEnd-$taskResetStart)
$taskC = $taskPrefix + "`r`n" + $taskResetCode + (Get-TaskSection '#define LOG_BAUD' 'void AppTelemetry_Init(void)') +
    (Get-TaskSection 'static void log_char(char c)' 'static void log_start_dma(void)') +
    (Get-TaskSection 'static void log_rx_service(void)' 'void AppTelemetry_Task(void *argument)') + $taskTests
[IO.File]::WriteAllText((Join-Path $taskTestDir 'commands_test.c'),$taskC,[Text.UTF8Encoding]::new($false))
$taskVcvars = & 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe' -utf8 -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find VC\Auxiliary\Build\vcvars64.bat
if (!$taskVcvars) { throw 'MSVC host compiler unavailable' }
$taskEnvironmentLines = & cmd.exe /d /c ('call "'+$taskVcvars+'" >nul && set')
if ($LASTEXITCODE -ne 0) { throw 'MSVC environment setup failed' }
foreach ($taskEnvironmentLine in $taskEnvironmentLines) {
    $taskSeparator=$taskEnvironmentLine.IndexOf('=')
    if ($taskSeparator -gt 0) { [Environment]::SetEnvironmentVariable($taskEnvironmentLine.Substring(0,$taskSeparator),$taskEnvironmentLine.Substring($taskSeparator+1),'Process') }
}
& cl.exe /nologo /W3 /WX /utf-8 /TC /std:c11 ('/I'+$taskTestDir) ('/I'+(Join-Path $taskRoot 'Core\Inc')) (Join-Path $taskTestDir 'commands_test.c') (Join-Path $taskRoot 'Core\Src\app_commands.c') (Join-Path $taskRoot 'Core\Src\balance_control.c') ('/Fo:'+$taskTestDir+'\') ('/Fe:'+(Join-Path $taskTestDir 'commands_test.exe'))
if ($LASTEXITCODE -ne 0) { throw 'Command tests host build failed' }
& (Join-Path $taskTestDir 'commands_test.exe')
if ($LASTEXITCODE -ne 0) { throw 'Command regression failed' }
