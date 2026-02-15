#ifndef COMMANDS_H
#define COMMANDS_H

typedef enum {
    CMD_FILL_N2 = 0,
    CMD_PRE_PRESSURIZE,
    CMD_FILL_OX,
    CMD_POST_PRESSURIZE,
    fill_cmd_count,
} fill_command_t;

typedef enum
{
    CMD_NONE = 0,
    CMD_STATUS,
    CMD_ABORT,
    CMD_STOP,
    CMD_ARM,
    CMD_FIRE,
    CMD_FILL_EXEC,
    CMD_MANUAL_EXEC,
    CMD_ACK,
    CMD_NACK,
    cmd_count,
} command_t;

// Manual commands
typedef enum
{
    CMD_MANUAL_SD_LOG_START,
    CMD_MANUAL_SD_LOG_STOP,
    CMD_MANUAL_SD_STATUS,
    CMD_MANUAL_VALVE_STATE, 
    CMD_MANUAL_VALVE_MS, // milliseconds to open/close valve
    manual_cmd_count,
} manual_command_t;

// Reading errors
#define CMD_READ_OK 0
#define CMD_READ_TIMEOUT 1
#define CMD_READ_BAD_CRC 2
#define CMD_READ_DEFAULT_ERROR 3
#define CMD_READ_NO_CMD 4


// Running errors
#define CMD_RUN_OK 0
#define CMD_RUN_ARM_ERROR 1
#define CMD_RUN_DEFAULT_ERROR 2
#define CMD_RUN_NO_ACTION 3
#define CMD_RUN_OUT_OF_BOUND 4
#define CMD_RUN_STATE_ERROR 5

#endif // COMMANDS_H
