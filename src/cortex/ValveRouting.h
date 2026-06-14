#ifndef VALVEROUTING_H
#define VALVEROUTING_H

#include <inttypes.h>

// valve_t, hydra_valve_t, ValveRoute, and the pure rc_get_valve_route() lookup
// live in lib/rocket_core so they can be unit-tested on the native host.
#include "rc_valve_routing.h"

int getValveRoute(valve_t valve, ValveRoute *route);

#endif // VALVEROUTING_H
