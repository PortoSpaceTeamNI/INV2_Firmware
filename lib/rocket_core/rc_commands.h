/**
 * @file rc_commands.h
 * @brief THE canonical top-level command set for the INV2 bus frame `cmd` byte.
 *
 * Background / why this file exists
 * ---------------------------------
 * The command enum used to be duplicated and DIVERGED across boards, so the same
 * command name had a different integer value depending on which board you asked
 * (e.g. CMD_ARM / CMD_MANUAL_EXEC). This header is the single agreed numbering;
 * every board includes it instead of a local copy
 * (see Documentation/monorepo-shared-lib-refactor.md).
 *
 * Canonical numbering: the CORTEX flight computer is the bus master and this is
 * its (leaner) command set. The earlier-generation OBC's extra states
 * (READY / LAUNCH_OVERRIDE / FILL_RESUME) are gone -- CORTEX drives those phases
 * through its event-based state machine, not dedicated wire commands. HYDRA/LIFT
 * pick up these values automatically (they only reference CMD_STATUS,
 * CMD_MANUAL_EXEC and CMD_ACK by name), keeping master and slaves wire-compatible.
 *
 * OBC<->slave traffic uses CMD_STATUS for polling and CMD_MANUAL_EXEC (sub-command
 * in payload[0], see rc_manual.h) for valves, which the HYDRA/LIFT receivers expect.
 */
#ifndef ROCKET_CORE_RC_COMMANDS_H
#define ROCKET_CORE_RC_COMMANDS_H

typedef enum
{
    CMD_NONE = 0,
    CMD_STATUS,      // 1
    CMD_ABORT,       // 2
    CMD_STOP,        // 3
    CMD_ARM,         // 4
    CMD_FIRE,        // 5
    CMD_FILL_EXEC,   // 6
    CMD_MANUAL_EXEC, // 7
    CMD_ACK,         // 8
    CMD_NACK,        // 9
    CMD_COUNT        // keep last
} command_t;

#endif // ROCKET_CORE_RC_COMMANDS_H
