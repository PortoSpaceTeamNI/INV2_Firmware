#include "Valves.h"
#include "HYDRA.h"

extern hydra_t hydras[];

int valve_set(valve_t valve, int state)
{
    switch (valve)
    {
    // Check HYDRA.cpp for valve mapping
    case VALVE_ABORT:
        set_hydra_valve(&hydras[HYDRA_LF], H_VALVE_CONTROLLED_1, state);
        break;
    case VALVE_N2_PURGE:
        set_hydra_valve(&hydras[HYDRA_FS], H_VALVE_STEEL_BALL_1, state);
        break;
    case VALVE_N2O_PURGE:
        set_hydra_valve(&hydras[HYDRA_FS], H_VALVE_CONTROLLED_3, state);
        break;
    case VALVE_MAIN:
        set_hydra_valve(&hydras[HYDRA_LF], H_VALVE_CONTROLLED_2, state);
        break;
    case VALVE_N2_FILL:
        set_hydra_valve(&hydras[HYDRA_FS], H_VALVE_STEEL_BALL_2, state);
        break;
    case VALVE_N2O_FILL:
        set_hydra_valve(&hydras[HYDRA_FS], H_VALVE_CONTROLLED_2, state);
        break;
    case VALVE_N2_QUICK_DC:
        set_hydra_valve(&hydras[HYDRA_FS], H_VALVE_QUICK_DC_2, state);
        break;
    case VALVE_N2O_QUICK_DC:
        set_hydra_valve(&hydras[HYDRA_FS], H_VALVE_QUICK_DC_1, state);
        break;
    case VALVE_PRESSURIZING:
        set_hydra_valve(&hydras[HYDRA_UF], H_VALVE_CONTROLLED_3, state);
        break;
    case VALVE_VENT:
        set_hydra_valve(&hydras[HYDRA_UF], H_VALVE_CONTROLLED_1, state);
        break;
    default:
        // valve not defined
        return -1;
    }
    return 0;
}

int close_all_valves()
{
    for (int v = 0; v < _VALVE_COUNT; v++)
    {
        valve_set((valve_t)v, VALVE_CLOSE);
    }
    return 0;
}

int close_all_valves_except(valve_t except_valve)
{
    for (int v = 0; v < _VALVE_COUNT; v++)
    {
        if (v != except_valve)
        {
            valve_set((valve_t)v, VALVE_CLOSE);
        }
    }
    return 0;
}