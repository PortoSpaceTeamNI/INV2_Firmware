#ifndef VALVEROUTING_H
#define VALVEROUTING_H

#include <inttypes.h>
#include "Communications.h"

typedef enum {
    VALVE_PRESSURIZING = 0,
    VALVE_VENT,
    VALVE_ABORT,
    VALVE_MAIN,
    VALVE_N2O_FILL,
    VALVE_N2O_PURGE,
    VALVE_N2_FILL,
    VALVE_N2_PURGE,
    VALVE_N2O_QUICK_DC,
    VALVE_N2_QUICK_DC,
    valve_count,
} valve_t;

typedef enum {
    H_VALVE_QUICK_DC_1 = 0,
    H_VALVE_QUICK_DC_2,
    H_VALVE_CONTROLLED_1,
    H_VALVE_CONTROLLED_2,
    H_VALVE_CONTROLLED_3,
    H_VALVE_STEEL_BALL_1,
    H_VALVE_STEEL_BALL_2,
    H_VALVE_SERVO,
    h_valve_count,
} hydra_valve_t;

struct ValveRoute {
    uint8_t hydraId;
    uint8_t hydraValve;
};

int getValveRoute(valve_t valve, ValveRoute *route);

#endif // VALVEROUTING_H
