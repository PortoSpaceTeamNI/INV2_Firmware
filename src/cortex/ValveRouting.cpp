#include "ValveRouting.h"

int getValveRoute(valve_t valve, ValveRoute *route)
{
    return rc_get_valve_route(valve, route);
}
