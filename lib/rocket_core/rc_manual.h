/**
 * @file rc_manual.h
 * @brief Canonical sub-command set carried in payload[0] of a CMD_MANUAL_EXEC frame.
 *
 * A CMD_MANUAL_EXEC frame's payload is laid out as:
 *
 *   payload[0] = manual_command_t   (which manual action)
 *   payload[1..] = action arguments (e.g. valve index, state, duration)
 *
 * Single source of truth. Replaces the per-board copies in OBC/HYDRA DataModels.h,
 * which already agreed on the valve sub-commands (VALVE_STATE = 3, VALVE_MS = 4)
 * but disagreed on the tail (OBC had CMD_MANUAL_ACK, HYDRA did not).
 */
#ifndef ROCKET_CORE_RC_MANUAL_H
#define ROCKET_CORE_RC_MANUAL_H

typedef enum
{
    CMD_MANUAL_SD_LOG_START = 0,
    CMD_MANUAL_SD_LOG_STOP,
    CMD_MANUAL_SD_STATUS,
    CMD_MANUAL_VALVE_STATE, // payload: [VALVE_STATE, valve, state]
    CMD_MANUAL_VALVE_MS,    // payload: [VALVE_MS, valve, state, ms]
    CMD_MANUAL_ACK,
    manual_cmd_size         // keep last
} manual_command_t;

#endif // ROCKET_CORE_RC_MANUAL_H
