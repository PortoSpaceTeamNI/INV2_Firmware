#ifndef DATA_MODELS_H
#define DATA_MODELS_H

#include <inttypes.h>
#include <stdbool.h>

typedef enum {
    CMD_STATUS = 0,
    CMD_VALVE_SET,
    CMD_VALVE_MS,
    CMD_ACK,
} cmd_t;

typedef union {
    struct {
        int64_t loadcell1;
        int64_t loadcell2;
        int64_t loadcell3;
    };
    int64_t raw[3];
} loadcells_t;

typedef struct {
    loadcells_t loadcells;
} data_t;

#endif // DATA_MODELS_H
