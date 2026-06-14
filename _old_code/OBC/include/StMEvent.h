#ifndef STM_EVENT_H_
#define STM_EVENT_H_

#include "StateMachine.h"

bool safe_active_cond(void);
bool safe_inactive_cond(void);
bool ignition_ready(void);

void launch_override(void);
void enter_safety_vent(void);
void exit_safety_vent(void);

#endif