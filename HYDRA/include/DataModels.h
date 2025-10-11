#ifndef DATA_MODELS_H
#define DATA_MODELS_H

#include <inttypes.h>
#include <stdbool.h>

typedef enum {
    HYDRA_UF = 1,
    HYDRA_LF,
    HYDRA_FS,
    hydra_id_count,
} hydra_id_t;

typedef enum {
    CMD_STATUS = 0,
    CMD_VALVE_SET,
    CMD_VALVE_MS,
    CMD_ACK,
} cmd_t;

typedef enum {
    VALVE_QUICK_DC_1 = 0,
    VALVE_QUICK_DC_2,
    VALVE_CONTROLLED_1,
    VALVE_CONTROLLED_2,
    VALVE_CONTROLLED_3,
    VALVE_STEEL_BALL_1,
    VALVE_STEEL_BALL_2,
    VALVE_SERVO,
    hydra_valve_count,
} valve_t;

typedef union {
    uint8_t raw;
    struct {
        uint8_t v_quick_dc_1 : 1;
        uint8_t v_quick_dc_2 : 1;
        uint8_t v_controlled_1 : 1;
        uint8_t v_controlled_2 : 1;
        uint8_t v_controlled_3 : 1;
        uint8_t v_steel_ball_1 : 1;
        uint8_t v_steel_ball_2 : 1;
        uint8_t v_servo : 1;
    };
} valve_states_t;

typedef struct {
    int16_t thermo1, thermo2, thermo3;
    int16_t pressure1, pressure2, pressure3;
    valve_states_t valve_states;
    bool cam_enable;
} data_t;

#endif // DATA_MODELS_H
