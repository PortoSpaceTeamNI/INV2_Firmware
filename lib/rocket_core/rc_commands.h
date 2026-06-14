/**
 * @file rc_commands.h
 * @brief THE canonical top-level command set for the INV2 bus frame `cmd` byte.
 *
 * Background / why this file exists
 * ---------------------------------
 * Before this refactor the command enum was duplicated and had DIVERGED:
 *   - OBC/include/DataModels.h  `cmd_type_t`  : NONE,STATUS,ABORT,STOP,READY,ARM,
 *                                               FIRE,LAUNCH_OVERRIDE,FILL_EXEC,
 *                                               FILL_RESUME,MANUAL_EXEC,ACK
 *   - HYDRA & LIFT DataModels.h `command_t`   : NONE,STATUS,ABORT,STOP,ARM,FIRE,
 *                                               FILL_EXEC,MANUAL_EXEC,ACK,NACK
 * Because OBC inserts READY / LAUNCH_OVERRIDE / FILL_RESUME, the *same name* had a
 * *different integer value* on OBC vs a slave (e.g. CMD_ARM = 5 on OBC, 4 on a
 * slave; CMD_MANUAL_EXEC = 10 on OBC, 7 on a slave). That is a latent wire bug.
 *
 * This enum is the single agreed numbering. It keeps OBC's (ground-facing)
 * ordering, which the operator GUI already targets, and appends NACK. All boards
 * include this header instead of a local copy (see
 * Documentation/monorepo-shared-lib-refactor.md).
 *
 * The old OBC<->slave sub-protocol enums (hydra_cmd_t HCMD_*, lift_cmd_t LCMD_*)
 * have been folded into this set: OBC now polls slaves with CMD_STATUS and drives
 * valves with CMD_MANUAL_EXEC (sub-command in payload[0], see rc_manual.h), which
 * is exactly what the HYDRA/LIFT receivers already expect.
 */
#ifndef ROCKET_CORE_RC_COMMANDS_H
#define ROCKET_CORE_RC_COMMANDS_H

typedef enum
{
    CMD_NONE = 0,
    CMD_STATUS,          // 1
    CMD_ABORT,           // 2
    CMD_STOP,            // 3
    CMD_READY,           // 4
    CMD_ARM,             // 5
    CMD_FIRE,            // 6
    CMD_LAUNCH_OVERRIDE, // 7
    CMD_FILL_EXEC,       // 8
    CMD_FILL_RESUME,     // 9
    CMD_MANUAL_EXEC,     // 10
    CMD_ACK,             // 11
    CMD_NACK,            // 12
    CMD_COUNT            // keep last
} command_t;

#endif // ROCKET_CORE_RC_COMMANDS_H
