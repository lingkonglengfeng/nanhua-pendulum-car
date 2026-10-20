#ifndef APP_COMMANDS_H
#define APP_COMMANDS_H

#include <stdint.h>

typedef enum
{
  APP_CMD_UNKNOWN = 0,
  APP_CMD_HELP, APP_CMD_GET, APP_CMD_SET, APP_CMD_ALL, APP_CMD_TARGET,
  APP_CMD_PID, APP_CMD_KP, APP_CMD_KI, APP_CMD_KD, APP_CMD_RUN,
  APP_CMD_STOP, APP_CMD_RESET, APP_CMD_MAP, APP_CMD_ENCSIGN,
  APP_CMD_MOTSIGN, APP_CMD_CPR, APP_CMD_LOG, APP_CMD_PERIOD, APP_CMD_LED,
  APP_CMD_PIDW, APP_CMD_OPEN, APP_CMD_ZERO, APP_CMD_ARM, APP_CMD_BALPID,
  APP_CMD_BALWIN, APP_CMD_BALLIMIT, APP_CMD_BALSIGN, APP_CMD_PENDSIGN,
  APP_CMD_FWD, APP_CMD_REV, APP_CMD_MOVE, APP_CMD_BALSAFE, APP_CMD_BALVEL, APP_CMD_BALSLEW,
  APP_CMD_BALTRACE, APP_CMD_BALTRIM, APP_CMD_BALSPD, APP_CMD_BALSEQ, APP_CMD_BALTURN, APP_CMD_BALTURNPI,
  APP_CMD_BALPOS, APP_CMD_BALHOLD, APP_CMD_BALPZERO, APP_CMD_BALPOSCFG, APP_CMD_BALSPIN,
  APP_CMD_SWING, APP_CMD_SWINGCFG, APP_CMD_SWINGENERGY, APP_CMD_SWINGDIR, APP_CMD_SWINGCATCH,
  APP_CMD_AS5600, APP_CMD_BALWHEEL, APP_CMD_BALSTEP
} AppCommandType;

typedef enum
{
  APP_CMD_OK = 0, APP_CMD_BAD_ARGS, APP_CMD_UNKNOWN_NAME, APP_CMD_QUEUE_FULL,
  APP_CMD_RX_OVERFLOW, APP_CMD_LINE_TOO_LONG, APP_CMD_BAD_CHAR, APP_CMD_RX_ERROR,
  APP_CMD_NOT_READY
} AppCommandStatus;

typedef struct
{
  uint32_t id;
  AppCommandType type;
  AppCommandStatus status;
} AppCommandReply;

/* Logger task submits complete ASCII lines; ControlTask commits at its boundary. */
void AppCommands_Submit(char *line);
void AppCommands_FeedByte(uint8_t byte);
void AppCommands_RxError(AppCommandStatus status);
void AppCommands_Reject(AppCommandStatus status);
void AppCommands_Apply(void);
uint8_t AppCommands_PopReply(AppCommandReply *reply);
const char *AppCommands_Name(AppCommandType type);
const char *AppCommands_Status(AppCommandStatus status);

extern volatile uint32_t g_uart_cmd_received;
extern volatile uint32_t g_uart_cmd_applied;
extern volatile uint32_t g_uart_cmd_errors;
extern volatile uint32_t g_uart_cmd_reply_drops;

#endif
