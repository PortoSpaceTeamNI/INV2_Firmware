#include <Arduino.h>

#include "HardwareCfg.h"
#include "GlobalVars.h"
#include "StMEvent.h"
#include "StMWork.h"
#include "Valves.h"

extern filling_params_t filling_params;
extern system_data_t system_data;

bool safe_active_cond(void)
{
    if(system_data.pressures.n2o_tank_pressure > filling_params.trigger_pressure && system_data.thermocouples.n2o_tank_uf_t1 > filling_params.target_temperature)
    {
        return true;
    }
    return false;
}

bool safe_inactive_cond(void)
{
    if(system_data.pressures.n2o_tank_pressure < filling_params.target_pressure || system_data.thermocouples.n2o_tank_uf_t1 < filling_params.trigger_temperature)
    {
        return true;
    }
    return false;
}

void enter_safety_vent(void)
{
    if(system_data.actuators.v_vent == VALVE_CLOSE) {
        valve_set(VALVE_VENT, VALVE_OPEN);
    }
}

void exit_safety_vent(void)
{
    if(system_data.actuators.v_vent == VALVE_OPEN) {
        valve_set(VALVE_VENT, VALVE_CLOSE);
    }
}

