#include "rc_valve_routing.h"

#include <stddef.h>

int rc_get_valve_route(valve_t valve, ValveRoute *route)
{
    if (route == NULL)
        return -1;

    switch (valve)
    {
    case VALVE_ABORT:
        route->hydraId    = HYDRA_LF_ID;
        route->hydraValve = H_VALVE_CONTROLLED_1;
        break;
    case VALVE_N2_PURGE:
        route->hydraId    = HYDRA_FS_ID;
        route->hydraValve = H_VALVE_STEEL_BALL_1;
        break;
    case VALVE_N2O_PURGE:
        route->hydraId    = HYDRA_FS_ID;
        route->hydraValve = H_VALVE_CONTROLLED_3;
        break;
    case VALVE_MAIN:
        route->hydraId    = HYDRA_LF_ID;
        route->hydraValve = H_VALVE_CONTROLLED_2;
        break;
    case VALVE_N2_FILL:
        route->hydraId    = HYDRA_FS_ID;
        route->hydraValve = H_VALVE_STEEL_BALL_2;
        break;
    case VALVE_N2O_FILL:
        route->hydraId    = HYDRA_FS_ID;
        route->hydraValve = H_VALVE_SERVO;
        break;
    case VALVE_N2_QUICK_DC:
        route->hydraId    = HYDRA_FS_ID;
        route->hydraValve = H_VALVE_QUICK_DC_2;
        break;
    case VALVE_N2O_QUICK_DC:
        route->hydraId    = HYDRA_FS_ID;
        route->hydraValve = H_VALVE_QUICK_DC_1;
        break;
    case VALVE_PRESSURIZING:
        route->hydraId    = HYDRA_LF_ID;
        route->hydraValve = H_VALVE_CONTROLLED_3;
        break;
    case VALVE_VENT:
        route->hydraId    = HYDRA_UF_ID;
        route->hydraValve = H_VALVE_CONTROLLED_1;
        break;
    default:
        return -1;
    }

    return 0;
}
