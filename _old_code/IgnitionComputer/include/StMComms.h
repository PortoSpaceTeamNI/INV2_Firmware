#ifndef STM_COMMS_H_
#define STM_COMMS_H_

#include "StateMachine.h"

int run_command(command_t* cmd, rocket_state_t state, interface_t interface);

#endif