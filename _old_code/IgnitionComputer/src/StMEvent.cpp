#include <Arduino.h>

#include "HardwareCfg.h"
#include "GlobalVars.h"
#include "StMEvent.h"

int16_t FP1 = 0, FP2 = 0; //target and trigger pressures
int16_t FL1 = 0; //target liquid

bool IdleCond(void) { return false; }
bool TrueCond(void) { return true; }

bool prog1_finish_cond(void)
{
    if(tank_pressure > FP1)
        return true;
    return false;
}

bool prog2_finish_cond(void)
{
    if(tank_liquid_kg > FL1)
        return true;
    return false;
}

bool prog3_finish_cond(void)
{
    if(Line_Module.pressure > FP1)
        return true;
    return false;
}

bool prog2_stop_cond(void)
{
    if(tank_pressure > FP2)
        return true;
    return false;
}

void prog2_stop_reaction(void)
{
    return;
}

void close_valves(void)
{
    return;
}

bool chamber_temp_cond(void)
{
    return N2O_Module.temperature > CHAMBER_TEMP_THREASHOLD;
}

//---------TIMERS---------------
bool arm_timer_event(void)
{
    return (arm_reset_timer > ARM_TIMER_TRIGGER);
}

bool fire_timer_event(void)
{
    return ((N2O_Module.temperature > CHAMBER_TEMP_THREASHOLD_ALLOW)
            || (fire_reset_timer > FIRE_TIMER_TRIGGER));
    //return ((fire_reset_timer > FIRE_TIMER_TRIGGER));
}

bool launch_timer_event(void)
{
    return launch_reset_timer > LAUNCH_TIMER_TRIGGER;
}

bool timeout_timer_event(void)
{
    return launch_reset_timer > TIMEOUT_TIMER_TRIGGER;
}