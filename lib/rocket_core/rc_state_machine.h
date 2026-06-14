/**
 * @file rc_state_machine.h
 * @brief Pure (I/O-free, FreeRTOS-free) state machine transition function.
 *
 * Defines the RocketState and RocketEvent enums and the rc_update_state()
 * function that maps (currentState, event) → nextState. This file is
 * target-agnostic so it can be compiled on the native host for unit tests.
 *
 * The CORTEX board's StateMachine.h includes this header and delegates its
 * updateState() wrapper to rc_update_state(). FreeRTOS types, mutexes, and
 * queue handles stay in StateMachine.h and StateMachine.cpp.
 */
#ifndef ROCKET_CORE_RC_STATE_MACHINE_H
#define ROCKET_CORE_RC_STATE_MACHINE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IDLE = 0,
    ABORT,
    FILL_N2,
    PRE_PRESSURIZE,
    FILL_OX,
    POST_PRESSURIZE,
    ARMED,
    IGNITION,
    LAUNCH,
    BOOST,
    COAST,
    DROGUE_DESCENT,
    MAIN_DESCENT,
    TOUCHDOWN,
    _STATE_COUNT,
} RocketState;

typedef enum {
    EV_NONE = 0,
    EV_DATA_UPDATE,
    EV_STOP_CMD,
    EV_ABORT_CMD,
    EV_FILL_N2_CMD,
    EV_PRE_PRESSURIZE_CMD,
    EV_FILL_OX_CMD,
    EV_POST_PRESSURIZE_CMD,
    EV_ARM_CMD,
    EV_FIRE_CMD,
    EV_LAUNCH,
    EV_TAKEOFF,
    EV_BOOST_END,
    EV_APOGEE,
    EV_MAIN_ALTITUDE,
    EV_TOUCHDOWN,
    _EVENT_COUNT,
} RocketEvent;

/**
 * Pure state transition function.
 * EV_ABORT_CMD always returns ABORT; EV_STOP_CMD always returns IDLE.
 * All other events follow the sequential filling/flight transition table.
 * Returns currentState when no valid transition exists for the given event.
 */
RocketState rc_update_state(RocketState currentState, RocketEvent event);

#ifdef __cplusplus
}
#endif

#endif // ROCKET_CORE_RC_STATE_MACHINE_H
