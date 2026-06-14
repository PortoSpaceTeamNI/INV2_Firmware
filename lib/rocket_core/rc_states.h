/**
 * @file rc_states.h
 * @brief Canonical flight/ground state enum (master state machine on OBC).
 *
 * Mirrors OBC/include/DataModels.h `state_t`. Kept here so any board that needs to
 * reason about the system state (or report/parse it on the wire) shares one
 * definition. Target-agnostic.
 *
 * `state_count` must stay last (used to size the transition table). S_NONE = -1.
 */
#ifndef ROCKET_CORE_RC_STATES_H
#define ROCKET_CORE_RC_STATES_H

typedef enum
{
    IDLE = 0,
    FILLING,
    SAFE_IDLE,
    FILLING_N2,
    PRE_PRESSURE,
    FILLING_N2O,
    POST_PRESSURE,
    READY,
    ARMED,
    IGNITION,
    LAUNCH,
    FLIGHT,
    RECOVERY,
    ABORT,
    state_count,   // keep last for table sizing
    S_NONE = -1
} state_t;

#endif // ROCKET_CORE_RC_STATES_H
