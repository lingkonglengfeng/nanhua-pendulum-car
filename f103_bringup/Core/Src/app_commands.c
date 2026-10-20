/* Bounded ASCII command parser; no ISR parsing, heap, or Flash writes. */
#include "app_commands.h"
#include "app_vehicle.h"
#include "app_telemetry.h"
#include "balance_control.h"
#include "pendulum_angle.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include <string.h>

#define COMMAND_QUEUE_SIZE 8U
#define REPLY_QUEUE_SIZE 16U
#define COMMAND_MAX_TOKENS 6U

volatile uint32_t g_uart_cmd_received = 0U;
volatile uint32_t g_uart_cmd_applied = 0U;
volatile uint32_t g_uart_cmd_errors = 0U;
volatile uint32_t g_uart_cmd_reply_drops = 0U;

typedef struct
{
  uint32_t id;
  AppCommandType type;
  float value[4];
  uint8_t index;
} Command;

/* Shared between ControlTask and UartLogTask. Queue operations use BASEPRI. */
static volatile Command s_commands[COMMAND_QUEUE_SIZE];
static volatile AppCommandReply s_replies[REPLY_QUEUE_SIZE];
static volatile uint8_t s_command_head;
static volatile uint8_t s_command_tail;
static volatile uint8_t s_command_count;
static volatile uint8_t s_reply_head;
static volatile uint8_t s_reply_tail;
static volatile uint8_t s_reply_count;
static uint32_t s_next_id; /* only UartLogTask generates command IDs */
static char s_line[96];
static uint8_t s_line_length;
static uint8_t s_discard_line;

const char *AppCommands_Name(AppCommandType type)
{
  static const char * const names[] = {
    "UNKNOWN", "HELP", "GET", "SET", "ALL", "TARGET", "PID", "KP", "KI", "KD",
    "RUN", "STOP", "RESET", "MAP", "ENCSIGN", "MOTSIGN", "CPR", "LOG", "PERIOD", "LED", "PIDW", "OPEN",
    "ZERO", "ARM", "BALPID", "BALWIN", "BALLIMIT", "BALSIGN", "PENDSIGN", "FWD", "REV", "MOVE", "BALSAFE", "BALVEL", "BALSLEW", "BALTRACE", "BALTRIM", "BALSPD", "BALSEQ", "BALTURN", "BALTURNPI", "BALPOS", "BALHOLD", "BALPZERO", "BALPOSCFG", "BALSPIN", "SWING", "SWINGCFG", "SWINGENERGY", "SWINGDIR", "SWINGCATCH", "AS5600", "BALWHEEL", "BALSTEP"
  };
  return ((uint32_t)type < sizeof(names) / sizeof(names[0])) ? names[type] : "UNKNOWN";
}

const char *AppCommands_Status(AppCommandStatus status)
{
  static const char * const names[] = {
    "OK", "BAD_ARGS", "UNKNOWN_COMMAND", "QUEUE_FULL", "RX_OVERFLOW",
    "LINE_TOO_LONG", "BAD_CHAR", "RX_ERROR", "NOT_READY"
  };
  return ((uint32_t)status < sizeof(names) / sizeof(names[0])) ? names[status] : "BAD_ARGS";
}

static void command_reply(uint32_t id, AppCommandType type, AppCommandStatus status)
{
  taskENTER_CRITICAL();
  if (s_reply_count < REPLY_QUEUE_SIZE)
  {
    s_replies[s_reply_head].id = id;
    s_replies[s_reply_head].type = type;
    s_replies[s_reply_head].status = status;
    s_reply_head = (uint8_t)((s_reply_head + 1U) % REPLY_QUEUE_SIZE);
    s_reply_count++;
  }
  else
  {
    g_uart_cmd_reply_drops++;
  }
  taskEXIT_CRITICAL();
}

uint8_t AppCommands_PopReply(AppCommandReply *reply)
{
  uint8_t found = 0U;
  taskENTER_CRITICAL();
  if (s_reply_count != 0U)
  {
    *reply = s_replies[s_reply_tail];
    s_reply_tail = (uint8_t)((s_reply_tail + 1U) % REPLY_QUEUE_SIZE);
    s_reply_count--;
    found = 1U;
  }
  taskEXIT_CRITICAL();
  return found;
}

void AppCommands_Reject(AppCommandStatus status)
{
  g_uart_cmd_received++;
  g_uart_cmd_errors++;
  command_reply(++s_next_id, APP_CMD_UNKNOWN, status);
}

void AppCommands_RxError(AppCommandStatus status)
{
  s_line_length = 0U;
  s_discard_line = 1U;
  AppCommands_Reject(status);
}

void AppCommands_FeedByte(uint8_t byte)
{
  if (byte == '\r' || byte == '\n')
  {
    if (s_discard_line == 0U && s_line_length != 0U)
    {
      s_line[s_line_length] = '\0';
      AppCommands_Submit(s_line);
    }
    s_line_length = 0U;
    s_discard_line = 0U;
  }
  else if (s_discard_line == 0U)
  {
    if (byte == 8U || byte == 127U)
    {
      if (s_line_length != 0U) { s_line_length--; }
    }
    else if ((byte < 32U && byte != '\t') || byte > 126U)
    {
      AppCommands_RxError(APP_CMD_BAD_CHAR);
    }
    else if (s_line_length >= sizeof(s_line) - 1U)
    {
      AppCommands_RxError(APP_CMD_LINE_TOO_LONG);
    }
    else
    {
      s_line[s_line_length++] = (char)byte;
    }
  }
}

static uint8_t command_number(const char *text, float min, float max, float *value)
{
  char *end;
  /* Accept decimal/exponent input only, reject hex, NaN, infinity and suffixes. */
  const char *p = text;
  if ((*p == '+') || (*p == '-')) { p++; }
  if (!((*p >= '0' && *p <= '9') || *p == '.')) { return 0U; }
  if ((p[0] == '0') && ((p[1] == 'x') || (p[1] == 'X'))) { return 0U; }
  *value = strtof(text, &end);
  return (end != text && *end == '\0' && *value >= min && *value <= max) ? 1U : 0U;
}

static uint8_t command_index(const char *text, uint8_t *index)
{
  if (text[0] < '0' || text[0] > '3' || text[1] != '\0') { return 0U; }
  *index = (uint8_t)(text[0] - '0');
  return 1U;
}

void AppCommands_Submit(char *line)
{
  char *token[COMMAND_MAX_TOKENS];
  char *p = line;
  uint8_t count = 0U;
  uint8_t i;
  uint8_t valid = 0U;
  Command command = {0};
  AppCommandStatus error = APP_CMD_BAD_ARGS;

  command.id = ++s_next_id;
  g_uart_cmd_received++;
  while (*p != '\0')
  {
    while (*p == ' ' || *p == '\t') { p++; }
    if (*p == '\0') { break; }
    if (count == COMMAND_MAX_TOKENS) { goto reject; }
    token[count++] = p;
    while (*p != '\0' && *p != ' ' && *p != '\t') { p++; }
    if (*p != '\0') { *p++ = '\0'; }
  }
  if (count == 0U) { goto reject; }
  for (p = token[0]; *p != '\0'; p++)
  {
    if (*p >= 'a' && *p <= 'z') { *p = (char)(*p - 'a' + 'A'); }
  }
  for (i = (uint8_t)APP_CMD_HELP; i <= (uint8_t)APP_CMD_BALSTEP; i++)
  {
    if (strcmp(token[0], AppCommands_Name((AppCommandType)i)) == 0)
    {
      command.type = (AppCommandType)i;
      break;
    }
  }
  switch (command.type)
  {
    case APP_CMD_HELP: case APP_CMD_GET: case APP_CMD_STOP: case APP_CMD_RESET:
    case APP_CMD_ZERO: case APP_CMD_ARM: case APP_CMD_BALHOLD: case APP_CMD_BALPZERO: case APP_CMD_SWING:
    case APP_CMD_AS5600:
      valid = (count == 1U);
      break;
    case APP_CMD_SET:
      valid = (count == 3U) && command_index(token[1], &command.index) &&
                command_number(token[2], -1000000.0f, 1000000.0f, &command.value[0]);
      break;
    case APP_CMD_OPEN:
      valid = (count == 3U) && command_index(token[1], &command.index) &&
                command_number(token[2], -100.0f, 100.0f, &command.value[0]);
      if (valid != 0U && command.value[0] != (float)(int32_t)command.value[0]) { valid = 0U; }
      break;
    case APP_CMD_ALL:
      valid = (count == 2U) && command_number(token[1], -1000000.0f, 1000000.0f, &command.value[0]);
      break;
    case APP_CMD_TARGET:
      valid = (count == 5U);
      if (valid != 0U)
      {
        for (i = 0U; i < APP_WHEEL_COUNT; i++)
        {
          if (!command_number(token[i+1U], -1000000.0f, 1000000.0f, &command.value[i])) { valid = 0U; }
        }
      }
      break;
    case APP_CMD_FWD: case APP_CMD_REV: case APP_CMD_MOVE:
      valid = (count == 2U) && command_number(token[1],
                (command.type == APP_CMD_MOVE) ? -280.0f : 0.0f, 280.0f, &command.value[0]);
      break;
    case APP_CMD_BALLIMIT:
      valid = (count == 2U) && command_number(token[1], 1.0f, 100.0f, &command.value[0]);
      break;
    case APP_CMD_BALSAFE:
      valid = (count == 3U) && command_number(token[1], 0.0f, 280.0f, &command.value[0]) &&
        command_number(token[2], 0.0f, 60000.0f, &command.value[1]);
      if (valid && command.value[0]>0.0f && command.value[0]<10.0f) valid=0U;
      if (valid && command.value[1] != (float)(uint32_t)command.value[1]) valid=0U;
      if (valid && command.value[1]>0.0f && command.value[1]<500.0f) valid=0U;
      break;
    case APP_CMD_BALVEL:
      valid = (count == 4U) && command_number(token[1], 0.0f, 0.05f, &command.value[0]) &&
        command_number(token[2], 0.0f, 0.01f, &command.value[1]) &&
        command_number(token[3], 0.1f, 3.0f, &command.value[2]);
      break;
    case APP_CMD_BALSPD:
      valid=(count==2U) && command_number(token[1],-30.0f,30.0f,&command.value[0]);
      break;
    case APP_CMD_BALPOS:
      valid=(count==2U) && command_number(token[1],-5.0f,5.0f,&command.value[0]);
      break;
    case APP_CMD_BALPOSCFG:
      valid=(count==3U) && command_number(token[1],0.1f,60.0f,&command.value[0]) &&
        command_number(token[2],1.0f,15.0f,&command.value[1]);
      break;
    case APP_CMD_BALWHEEL:
      valid=(count==2U) && command_number(token[1],20.0f,300.0f,&command.value[0]);
      break;
    case APP_CMD_BALSTEP:
      if(count==1U)
      {
        command.value[0]=1.0f; command.value[1]=20.0f; valid=1U;
      }
      else
      {
        valid=(count==3U) && command_number(token[1],-2.0f,2.0f,&command.value[0]) &&
          command_number(token[2],1.0f,20.0f,&command.value[1]);
        if(valid && command.value[0]>-0.05f && command.value[0]<0.05f) valid=0U;
      }
      break;
    case APP_CMD_SWINGCFG:
      valid=(count==4U) && command_number(token[1],10.0f,60.0f,&command.value[0]) &&
        command_number(token[2],0.5f,8.0f,&command.value[1]) &&
        command_number(token[3],500.0f,15000.0f,&command.value[2]);
      if(valid && command.value[2]!=(float)(uint32_t)command.value[2]) valid=0U;
      break;
    case APP_CMD_SWINGENERGY:
      valid=(count==4U) && command_number(token[1],0.01f,0.8f,&command.value[0]) &&
        command_number(token[2],10.0f,200.0f,&command.value[1]) &&
        command_number(token[3],0.2f,4.0f,&command.value[2]);
      break;
    case APP_CMD_SWINGDIR:
      valid=(count==2U) && command_number(token[1],-1.0f,1.0f,&command.value[0]) &&
        (command.value[0]==-1.0f || command.value[0]==1.0f);
      break;
    case APP_CMD_SWINGCATCH:
      valid=(count==2U) && command_number(token[1],0.5f,4.0f,&command.value[0]);
      break;
    case APP_CMD_BALTURN: case APP_CMD_BALSPIN:
      valid=(count==2U) && command_number(token[1],-10.0f,10.0f,&command.value[0]);
      break;
    case APP_CMD_BALTURNPI:
      valid=(count==3U) && command_number(token[1],0.0f,0.02f,&command.value[0]) &&
        command_number(token[2],0.0f,0.05f,&command.value[1]);
      break;
    case APP_CMD_BALSEQ:
      if(count==1U)
      {
        command.value[0]=8.0f; command.value[1]=0.25f; command.value[2]=2000.0f;
        valid=1U;
      }
      else
      {
        valid=(count==4U) && command_number(token[1],-20.0f,20.0f,&command.value[0]) &&
          command_number(token[2],0.05f,0.5f,&command.value[1]) &&
          command_number(token[3],400.0f,5000.0f,&command.value[2]);
        if(valid && ((command.value[0]>-1.0f && command.value[0]<1.0f) ||
          command.value[2]!=(float)(uint32_t)command.value[2])) valid=0U;
      }
      break;
    case APP_CMD_BALSLEW:
      valid = (count == 2U) && command_number(token[1], 0.1f, 10.0f, &command.value[0]);
      break;
    case APP_CMD_BALTRIM:
      valid = (count == 2U) && command_number(token[1], -1.0f, 1.0f, &command.value[0]);
      break;
    case APP_CMD_BALSIGN: case APP_CMD_PENDSIGN:
      valid = (count == 2U) && command_number(token[1], -1.0f, 1.0f, &command.value[0]);
      if (valid && command.value[0] != -1.0f && command.value[0] != 1.0f) valid = 0U;
      break;
    case APP_CMD_BALWIN:
      valid = (count == 4U) && command_number(token[1], 0.1f, 45.0f, &command.value[0]) &&
        command_number(token[2], 0.1f, 45.0f, &command.value[1]) &&
        command_number(token[3], 400.0f, 5000.0f, &command.value[2]);
      if (valid && (command.value[1] < command.value[0] ||
          command.value[2] != (float)(uint32_t)command.value[2])) valid = 0U;
      break;
    case APP_CMD_PID: case APP_CMD_BALPID:
      valid = (count == 4U);
      if (valid != 0U)
      {
        for (i = 0U; i < 3U; i++)
        {
          if (!command_number(token[i+1U], 0.0f, 100.0f, &command.value[i])) { valid = 0U; }
        }
      }
      break;
    case APP_CMD_PIDW:
      valid = (count == 5U) && command_index(token[1], &command.index);
      if (valid != 0U)
      {
        for (i = 0U; i < 3U; i++)
        {
          if (!command_number(token[i+2U], 0.0f, 100.0f, &command.value[i])) { valid = 0U; }
        }
      }
      break;
    case APP_CMD_KP: case APP_CMD_KI: case APP_CMD_KD:
      valid = (count == 2U) && command_number(token[1], 0.0f, 100.0f, &command.value[0]);
      break;
    case APP_CMD_RUN: case APP_CMD_LOG: case APP_CMD_BALTRACE:
      valid = (count == 2U) && (token[1][0] == '0' || token[1][0] == '1') && token[1][1] == '\0';
      if (valid != 0U) { command.value[0] = (float)(token[1][0] - '0'); }
      break;
    case APP_CMD_MAP:
      valid = (count == 3U) && command_index(token[1], &command.index);
      if (valid != 0U)
      {
        uint8_t mapped = 0U;
        valid = command_index(token[2], &mapped);
        command.value[0] = (float)mapped;
      }
      break;
    case APP_CMD_ENCSIGN: case APP_CMD_MOTSIGN:
      valid = (count == 3U) && command_index(token[1], &command.index) &&
                command_number(token[2], -1.0f, 1.0f, &command.value[0]);
      if (valid != 0U && command.value[0] != -1.0f && command.value[0] != 1.0f) { valid = 0U; }
      break;
    case APP_CMD_CPR:
      valid = (count == 2U) && command_number(token[1], 0.001f, 1000000.0f, &command.value[0]);
      break;
    case APP_CMD_PERIOD:
      valid = (count == 2U) && command_number(token[1], 20.0f, 1000.0f, &command.value[0]);
      if (valid != 0U && command.value[0] != (float)(uint32_t)command.value[0]) { valid = 0U; }
      break;
    case APP_CMD_LED:
      valid = (count == 3U) && command_index(token[1], &command.index) &&
                command_number(token[2], 0.0f, 100.0f, &command.value[0]);
      break;
    default:
      error = APP_CMD_UNKNOWN_NAME;
      break;
  }
  if (valid == 0U) { goto reject; }
  taskENTER_CRITICAL();
  if (s_command_count < COMMAND_QUEUE_SIZE)
  {
    s_commands[s_command_head] = command;
    s_command_head = (uint8_t)((s_command_head + 1U) % COMMAND_QUEUE_SIZE);
    s_command_count++;
    taskEXIT_CRITICAL();
    return;
  }
  taskEXIT_CRITICAL();
  error = APP_CMD_QUEUE_FULL;
reject:
  g_uart_cmd_errors++;
  command_reply(command.id, command.type, error);
}

static void command_stop(void)
{
  uint8_t i;
  g_speed_enable = 0U;
  g_motor_test_enable = 0U;
  g_drive_test = 0;
  g_bal_enable = 0U;
  Balance_Stop(BAL_REASON_STOP);
  g_pendulum_zero_request = 0U;
  g_speed_reset = 1U;
  for (i = 0U; i < APP_WHEEL_COUNT; i++) { g_speed_target[i] = 0.0f; }
}

static void command_open(const Command *command)
{
  int32_t pct = (int32_t)command->value[0] * (int32_t)g_mot_sign[command->index];
  if (pct > 100) { pct = 100; }
  if (pct < -100) { pct = -100; }
  command_stop(); /* no other speed loop may react during this single-wheel test */
  g_motor_test_wheel = command->index;
  if (pct < 0)
  {
    /* AT8236 reverse: IN2=1, PWM=1-|u|; never use raw |u| here. */
    g_motor_test_dir = 1U;
    g_motor_test_pwm_pct = (uint8_t)(100 + pct);
  }
  else
  {
    g_motor_test_dir = 0U;
    g_motor_test_pwm_pct = (uint8_t)pct;
  }
  g_motor_test_enable = 1U;
}

void AppCommands_Apply(void)
{
  Command command;
  uint8_t budget;
  uint8_t i;
  for (budget = 0U; budget < COMMAND_QUEUE_SIZE; budget++)
  {
    AppCommandStatus status = APP_CMD_OK;
    taskENTER_CRITICAL();
    if (s_command_count == 0U)
    {
      taskEXIT_CRITICAL();
      break;
    }
    command = s_commands[s_command_tail];
    s_command_tail = (uint8_t)((s_command_tail + 1U) % COMMAND_QUEUE_SIZE);
    s_command_count--;
    taskEXIT_CRITICAL();
    switch (command.type)
    {
      case APP_CMD_ZERO:
        command_stop(); g_pendulum_zero_request = 1U; break;
      case APP_CMD_ARM:
        if (!g_pendulum_calibrated || g_pendulum_zero_request ||
            g_pendulum_zero_result == PENDULUM_ZERO_COLLECTING)
          status = APP_CMD_NOT_READY;
        else
        {
          command_stop(); (void)Balance_Arm(g_pendulum_calibrated);
        }
        break;
      case APP_CMD_SWING:
      {
        float peak=0.0f;
        for(i=0U;i<APP_WHEEL_COUNT;i++)
        {
          float v=g_speed_fast[i];if(v<0.0f)v=-v;
          if(!(v>=0.0f && v<=100000.0f)) peak=100001.0f;
          else if(v>peak)peak=v;
        }
        if(!g_pendulum_calibrated || g_pendulum_zero_request ||
           g_pendulum_zero_result==PENDULUM_ZERO_COLLECTING || !g_pendulum_angle_valid || !g_as5600_valid)
          status=APP_CMD_NOT_READY;
        else if(!Balance_StartSwing(g_pendulum_calibrated,g_pendulum_angle_valid,peak)) status=APP_CMD_BAD_ARGS;
        else
        {
          g_speed_enable=0U;g_motor_test_enable=0U;g_drive_test=0;g_speed_reset=1U;
          for(i=0U;i<APP_WHEEL_COUNT;i++)g_speed_target[i]=0.0f;
        }
        break;
      }
      case APP_CMD_SWINGCFG:
        command_stop();g_swing_limit=command.value[0]*0.01f;
        g_swing_travel_rev=command.value[1];g_swing_max_ms=(uint32_t)command.value[2];break;
      case APP_CMD_SWINGENERGY:
        command_stop();g_swing_gain=command.value[0];g_swing_omega2=command.value[1];
        g_swing_capture_rate=command.value[2];break;
      case APP_CMD_SWINGDIR:
        command_stop();g_swing_kick_sign=(int8_t)command.value[0];break;
      case APP_CMD_SWINGCATCH:
        command_stop();g_swing_capture_deg=command.value[0];break;
      case APP_CMD_BALPID:
        g_bal_kp=command.value[0]; g_bal_ki=command.value[1]; g_bal_kd=command.value[2];
        g_bal_integral=0.0f; break;
      case APP_CMD_BALWIN:
        command_stop(); g_bal_entry_deg=command.value[0]; g_bal_exit_deg=command.value[1];
        g_bal_dwell_ms=(uint32_t)command.value[2]; break;
      case APP_CMD_BALLIMIT:
        g_bal_limit=command.value[0]*0.01f; g_bal_integral=0.0f; break;
      case APP_CMD_BALSAFE:
        command_stop(); g_bal_max_rpm=command.value[0];
        g_bal_max_run_ms=(uint32_t)command.value[1]; break;
      case APP_CMD_BALVEL:
        command_stop(); g_bal_vel_kp=command.value[0]; g_bal_vel_ki=command.value[1];
        g_bal_vel_limit_deg=command.value[2];
        g_bal_vel_enable=(command.value[0]>0.0f || command.value[1]>0.0f) ? 1U : 0U;
        break;
      case APP_CMD_BALTURNPI:
        g_bal_turn_kp=command.value[0]; g_bal_turn_ki=command.value[1];
        g_bal_turn_integral=0.0f; break;
      case APP_CMD_BALPOSCFG:
        g_bal_pos_kp=command.value[0]; g_bal_pos_max_rpm=command.value[1]; break;
      case APP_CMD_BALWHEEL:
        command_stop(); g_bal_wheel_diameter_mm=command.value[0]; break;
      case APP_CMD_BALSTEP:
        if(!g_pendulum_calibrated || g_bal_state!=BAL_ACTIVE || !g_bal_enable || !g_bal_vel_enable ||
           !(g_bal_wheel_diameter_mm>=20.0f && g_bal_wheel_diameter_mm<=300.0f))
          status=APP_CMD_NOT_READY;
        else if(!Balance_StartStep(command.value[0],command.value[1])) status=APP_CMD_BAD_ARGS;
        break;
      case APP_CMD_BALPOS: case APP_CMD_BALHOLD: case APP_CMD_BALPZERO:
        if(!g_pendulum_calibrated || g_bal_state!=BAL_ACTIVE || !g_bal_enable || !g_bal_vel_enable)
          status=APP_CMD_NOT_READY;
        else if(command.type==APP_CMD_BALPOS)
        {
          if(!Balance_SetPosition(command.value[0])) status=APP_CMD_BAD_ARGS;
        }
        else if(!Balance_HoldPosition((command.type==APP_CMD_BALPZERO)?1U:0U))
          status=APP_CMD_BAD_ARGS;
        break;
      case APP_CMD_BALSPD: case APP_CMD_BALSEQ: case APP_CMD_BALTURN: case APP_CMD_BALSPIN:
        if(!g_pendulum_calibrated || g_bal_state!=BAL_ACTIVE || !g_bal_enable || !g_bal_vel_enable)
          status=APP_CMD_NOT_READY;
        else if(command.type==APP_CMD_BALSPIN)
        {
          if(!Balance_StartSpin(command.value[0])) status=APP_CMD_BAD_ARGS;
        }
        else if(command.type==APP_CMD_BALTURN)
        {
          if(!Balance_SetTurn(command.value[0])) status=APP_CMD_BAD_ARGS;
        }
        else if(command.type==APP_CMD_BALSPD)
        {
          if(!Balance_SetSpeed(command.value[0])) status=APP_CMD_BAD_ARGS;
        }
        else if(!Balance_StartSequence(command.value[0],command.value[1],(uint32_t)command.value[2]))
          status=APP_CMD_BAD_ARGS;
        break;
      case APP_CMD_BALSLEW:
        command_stop(); g_bal_vel_slew_deg_s=command.value[0]; break;
      case APP_CMD_BALTRIM:
        command_stop(); g_bal_trim_deg=command.value[0]; break;
      case APP_CMD_BALSIGN:
        command_stop(); g_bal_sign=(int8_t)command.value[0]; break;
      case APP_CMD_PENDSIGN:
        command_stop(); g_pendulum_sign_request=(int8_t)command.value[0]; break;
      case APP_CMD_FWD: case APP_CMD_REV: case APP_CMD_MOVE:
        if (command.value[0] == 0.0f) { command_stop(); break; }
        /* Repeated remote speed updates preserve the established wheel PI state. */
        if (!g_speed_enable || g_motor_test_enable || g_drive_test) g_speed_reset=1U;
        Balance_Stop(BAL_REASON_STOP); g_pendulum_zero_request=0U;
        g_motor_test_enable=0U; g_drive_test=0;
        for (i=0U; i<APP_WHEEL_COUNT; i++)
          g_speed_target[i]=(command.type==APP_CMD_REV) ? -command.value[0] : command.value[0];
        g_speed_enable=1U;
        break;
      case APP_CMD_SET: g_speed_target[command.index] = command.value[0]; break;
      case APP_CMD_OPEN: command_open(&command); break;
      case APP_CMD_ALL:
        for (i = 0U; i < APP_WHEEL_COUNT; i++) { g_speed_target[i] = command.value[0]; }
        break;
      case APP_CMD_TARGET:
        for (i = 0U; i < APP_WHEEL_COUNT; i++) { g_speed_target[i] = command.value[i]; }
        break;
      case APP_CMD_PID:
        g_spd_kp = command.value[0]; g_spd_ki = command.value[1]; g_spd_kd = command.value[2];
        for (i = 0U; i < APP_WHEEL_COUNT; i++) { g_spd_individual[i] = 0U; }
        g_speed_reset = 1U; break;
      case APP_CMD_PIDW:
        g_spd_kp_w[command.index] = command.value[0];
        g_spd_ki_w[command.index] = command.value[1];
        g_spd_kd_w[command.index] = command.value[2];
        g_spd_individual[command.index] = 1U;
        App_ResetSpeedWheel(command.index);
        break;
      case APP_CMD_KP:
        g_spd_kp = command.value[0];
        for (i = 0U; i < APP_WHEEL_COUNT; i++) { g_spd_kp_w[i] = command.value[0]; }
        g_speed_reset = 1U; break;
      case APP_CMD_KI:
        g_spd_ki = command.value[0];
        for (i = 0U; i < APP_WHEEL_COUNT; i++) { g_spd_ki_w[i] = command.value[0]; }
        g_speed_reset = 1U; break;
      case APP_CMD_KD:
        g_spd_kd = command.value[0];
        for (i = 0U; i < APP_WHEEL_COUNT; i++) { g_spd_kd_w[i] = command.value[0]; }
        g_speed_reset = 1U; break;
      case APP_CMD_RUN:
        if (command.value[0] == 0.0f) { command_stop(); }
        else
        {
          /* Repeated enable while already in speed mode is idempotent.
           * Preserve I/D state during target changes or periodic RUN 1. */
          if (g_speed_enable == 0U || g_motor_test_enable != 0U || g_drive_test != 0)
          {
            g_speed_reset = 1U;
          }
          g_motor_test_enable = 0U; g_drive_test = 0; g_bal_enable = 0U;
          Balance_Stop(BAL_REASON_STOP);
          g_pendulum_zero_request = 0U;
          g_speed_enable = 1U;
        }
        break;
      case APP_CMD_STOP: command_stop(); break;
      case APP_CMD_RESET: g_speed_reset = 1U; break;
      case APP_CMD_MAP:
        command_stop();
        g_enc_map[command.index] = (int8_t)command.value[0]; g_speed_reset = 1U; break;
      case APP_CMD_ENCSIGN:
        command_stop();
        g_enc_sign[command.index] = (int8_t)command.value[0]; g_speed_reset = 1U; break;
      case APP_CMD_MOTSIGN:
        command_stop();
        g_mot_sign[command.index] = (int8_t)command.value[0]; g_speed_reset = 1U; break;
      case APP_CMD_CPR: g_enc_counts_per_rev = command.value[0]; g_speed_reset = 1U; break;
      case APP_CMD_LOG: g_uart_log_enable = (uint8_t)command.value[0]; break;
      case APP_CMD_BALTRACE:
        command_stop(); g_uart_bal_trace=(uint8_t)command.value[0]; break;
      case APP_CMD_PERIOD: g_uart_log_period_ms = (uint32_t)command.value[0]; break;
      case APP_CMD_LED:
        g_speed_led_wheel = command.index; g_speed_tolerance_pct = command.value[0]; break;
      default: break; /* GET/HELP/AS5600 produce replies in the logger task. */
    }
    if (status == APP_CMD_OK) g_uart_cmd_applied++;
    else g_uart_cmd_errors++;
    command_reply(command.id, command.type, status);
  }
}
