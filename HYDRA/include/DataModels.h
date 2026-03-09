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

typedef enum
{
    CMD_NONE = 0,
    CMD_STATUS,
    CMD_ABORT,
    CMD_STOP,
    CMD_ARM,
    CMD_FIRE,
    CMD_FILL_EXEC,
    CMD_MANUAL_EXEC,
    CMD_ACK,
    CMD_NACK,
    cmd_count,
} command_t;

// Manual commands
typedef enum
{
    CMD_MANUAL_SD_LOG_START,
    CMD_MANUAL_SD_LOG_STOP,
    CMD_MANUAL_SD_STATUS,
    CMD_MANUAL_VALVE_STATE, 
    CMD_MANUAL_VALVE_MS, // milliseconds to open/close valve
    manual_cmd_count,
} manual_command_t;

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
