#include <Arduino.h>

#include <time.h>

#include "StateMachine.h"
#include "StMWork.h"
#include "StMEvent.h"
#include "Valves.h"
#include <stdarg.h>
#include <limits.h>

extern system_data_t system_data;


state_t expected_state[state_count][cmd_size] = {
    //                       NONE           STATUS         ABORT  STOP   READY          ARM            FIRE           LAUNCH_OVERRIDE FILL_EXEC      FILL_RESUME    MANUAL_EXEC    ACK
    /* IDLE */             { IDLE,          IDLE,          ABORT, IDLE,  READY,         IDLE,          IDLE,          IDLE,           FILLING,       FILLING,       IDLE,          IDLE }, 
    /* FILLING */          { FILLING,       FILLING,       ABORT, IDLE,  FILLING,       FILLING,       FILLING,       FILLING,        FILLING,       FILLING,       FILLING,       FILLING },
    /* SAFE IDLE */        { SAFE_IDLE,     SAFE_IDLE,     ABORT, IDLE,  SAFE_IDLE,     SAFE_IDLE,     SAFE_IDLE,     SAFE_IDLE,      SAFE_IDLE,     SAFE_IDLE,     SAFE_IDLE,     SAFE_IDLE }, 
    /* FILLING N2 */       { FILLING_N2,    FILLING_N2,    ABORT, IDLE,  FILLING_N2,    FILLING_N2,    FILLING_N2,    FILLING_N2,     FILLING_N2,    FILLING_N2,    FILLING_N2,    FILLING_N2 }, 
    /* PRE PRESSURE */     { PRE_PRESSURE,  PRE_PRESSURE,  ABORT, IDLE,  PRE_PRESSURE,  PRE_PRESSURE,  PRE_PRESSURE,  PRE_PRESSURE,   PRE_PRESSURE,  PRE_PRESSURE,  PRE_PRESSURE,  PRE_PRESSURE }, 
    /* FILLING N2O */      { FILLING_N2O,   FILLING_N2O,   ABORT, IDLE,  FILLING_N2O,   FILLING_N2O,   FILLING_N2O,   FILLING_N2O,    FILLING_N2O,   FILLING_N2O,   FILLING_N2O,   FILLING_N2O }, 
    /* POST PRESSURE */    { POST_PRESSURE, POST_PRESSURE, ABORT, IDLE,  POST_PRESSURE, POST_PRESSURE, POST_PRESSURE, POST_PRESSURE,  POST_PRESSURE, POST_PRESSURE, POST_PRESSURE, POST_PRESSURE },
    /* READY */            { READY,         READY,         ABORT, IDLE,  READY,         ARMED,         READY,         READY,          READY,         READY,         READY,         READY }, 
    /* ARMED */            { ARMED,         ARMED,         ABORT, IDLE,  ARMED,         ARMED,         IGNITION,      ARMED,          ARMED,         ARMED,         ARMED,         ARMED }, 
    /* IGNITION */         { IGNITION,      IGNITION,      ABORT, IDLE,  IGNITION,      IGNITION,      IGNITION,      LAUNCH,         IGNITION,      IGNITION,      IGNITION,      IGNITION }, 
    /* LAUNCH */           { LAUNCH,        LAUNCH,        ABORT, IDLE,  LAUNCH,        LAUNCH,        LAUNCH,        LAUNCH,         LAUNCH,        LAUNCH,        LAUNCH,        LAUNCH }, 
    /* FLIGHT */           { FLIGHT,        FLIGHT,        ABORT, IDLE,  FLIGHT,        FLIGHT,        FLIGHT,        FLIGHT,         FLIGHT,        FLIGHT,        FLIGHT,        FLIGHT },
    /* RECOVERY */         { RECOVERY,      RECOVERY,      ABORT, IDLE,  RECOVERY,      RECOVERY,      RECOVERY,      RECOVERY,       RECOVERY,      RECOVERY,      RECOVERY,      RECOVERY }, 
    /* ABORT */            { ABORT,         ABORT,         ABORT, IDLE,  IDLE,          ABORT,         ABORT,         ABORT,          ABORT,         ABORT,         ABORT,         ABORT }, 
};     

sm_state_t state_machine[state_count] =
{
    // IDLE
    {
        .work = {
        },

        .events = {
        },

        .next_states = expected_state[IDLE],
    },
    // FILLING
    {
        .work = {
        },

        .events = {
        },

        .next_states = expected_state[FILLING],
    },
    // SAFE_IDLE
    {
        .work = {
        },

        .events = {
            {.condition = safe_active_cond, .reaction = enter_safety_vent, .next_state = S_NONE},
            {.condition = safe_inactive_cond, .reaction = exit_safety_vent, .next_state = S_NONE},
        },

        .next_states = expected_state[SAFE_IDLE],
    },
    // FILLING_N2
    {
        .work = {
        },

        .events = {
        },

        .next_states = expected_state[FILLING_N2],
    },
    // PRE_PRESSURE
    {
        .work = {
        },

        .events = {
        },

        .next_states = expected_state[PRE_PRESSURE],
    },
    // FILLING_N2O
    {
        .work = {
        },

        .events = {
        },

        .next_states = expected_state[FILLING_N2O],
    },
    // POST_PRESSURE
    {
        .work = {
        },

        .events = {

        },

        .next_states = expected_state[POST_PRESSURE],
    },
    // READY
    {
        .work = {
        },

        .events = {
        },

        .next_states = expected_state[READY],

    },
    // ARMED
    {
        .work = {

        },

        .events = {

        },

        .next_states = expected_state[ARMED],

    },
    // IGNITION
    {
        .work = {

        },

        .events = {
            {.condition = ignition_ready, .reaction = launch_override, .next_state=LAUNCH}
        },

        .next_states = expected_state[IGNITION],

    },
    // LAUNCH
    {
        .work = {
        },

        .events = {
        },

        .next_states = expected_state[LAUNCH],

    },
    
    // FLIGHT
    {
        .work = {
        },

        .events = {
        }, 

        .next_states = expected_state[FLIGHT],
    },
    // RECOVERY 
    {
        .work = {
        },

        .events = {
        }, 

        .next_states = expected_state[FLIGHT],
    },
    // ABORT
    {
        .work = {
            
        },

        .events = {
            
        },

        .next_states = expected_state[ABORT],

    },
};

state_t event_handler()
{
    state_t next_state_event = S_NONE;

    for (int i = 0; i < MAX_EVENT_SIZE; i++)
    {
        bool (*cond)(void) = state_machine[system_data.state].events[i].condition;
        void (*react)(void) = state_machine[system_data.state].events[i].reaction;
        state_t next_state = state_machine[system_data.state].events[i].next_state;

        if (cond == NULL)
            continue;

        if (cond())
        {
            if (react != NULL)
                react();
            if (next_state != system_data.state && next_state_event == S_NONE)
                next_state_event = next_state;
        }
    }

    return next_state_event;
}

bool work_handler()
{
    bool change = false;
    for (int i = 0; i < MAX_WORK_SIZE; i++)
    {
        unsigned long end = millis() - state_machine[system_data.state].entry_time;

        if (state_machine[system_data.state].work[i].channel == NULL)
            continue;

        unsigned long msec = (end - state_machine[system_data.state].work[i].begin);

        // avoid overflows when it produce negative msec
        if (end < state_machine[system_data.state].work[i].begin)
            continue;

        if (msec > state_machine[system_data.state].work[i].sample)
        {
            state_machine[system_data.state].work[i].begin = end;
            state_machine[system_data.state].work[i].channel(); // execute sample function
            change = true;
        }
    }

    return change;
}