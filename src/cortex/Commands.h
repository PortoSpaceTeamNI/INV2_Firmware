#ifndef COMMANDS_H
#define COMMANDS_H

// Top-level command set (command_t) and the CMD_MANUAL_* sub-commands
// (manual_command_t) are the single source of truth in lib/rocket_core, shared
// with HYDRA/LIFT so master and slaves agree on the wire numbering.
#include "rc_commands.h"
#include "rc_manual.h"

// Fill-program sub-commands carried in payload[0] of a CMD_FILL_EXEC frame
// (Mission Control GUI -> CORTEX only; never forwarded to a slave).
typedef enum {
    CMD_FILL_NONE = 0,
    CMD_FILL_N2,
    CMD_PRE_PRESSURIZE,
    CMD_FILL_OX,
    CMD_POST_PRESSURIZE,
    fill_cmd_count,
} fill_command_t;

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
