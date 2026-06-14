#ifndef DATA_MODELS_H
#define DATA_MODELS_H

#include <inttypes.h>
#include <stdbool.h>

// Canonical command set shared across all boards (lib/rocket_core).
#include "rc_commands.h"

typedef union {
    struct {
        int32_t loadcell1;
        int32_t loadcell2;
        int32_t loadcell3;
    };
    int32_t raw[3];
} loadcells_t;

struct __attribute__((__packed__)) data_t {
    loadcells_t loadcells;
};

struct __attribute__((__packed__)) LiftTankData {
    int32_t tank_weight; // unit * 100
};

struct __attribute__((__packed__)) LiftBottleData {
    int32_t bottle_weight; // unit * 100
};

struct __attribute__((__packed__)) LiftThrustData {
    int32_t thrust_1; // unit * 100
    int32_t thrust_2; // unit * 100
    int32_t thrust_3; // unit * 100
};

#endif // DATA_MODELS_H
