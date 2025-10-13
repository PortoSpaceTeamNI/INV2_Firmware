#ifndef DATA_MODELS_H_
#define DATA_MODELS_H_

#include <stdint.h>
#include <Arduino.h>

//use the enum below as the values of state_t
//use -1 for default behavior
typedef enum 
{
    IDLE,
    FILLING,
    SAFE_IDLE,
    FILLING_N2,
    PRE_PRESSURE,
    FILLING_N2O,
    POST_PRESSURE,
    READY,
    ARMED,
    IGNITION,
    LAUNCH,
    FLIGHT,
    RECOVERY,
    ABORT,
    state_count, //this needs to be the last state for size to work
    S_NONE = -1
} state_t;

/*
    Commands
*/
typedef enum
{
    // shared commands
    CMD_NONE = 0,
    CMD_STATUS,
    CMD_ABORT,
    CMD_STOP,
    CMD_READY,
    CMD_ARM,
    CMD_FIRE,
    CMD_LAUNCH_OVERRIDE,
    CMD_FILL_EXEC,
    CMD_FILL_RESUME,
    CMD_MANUAL_EXEC,
    CMD_ACK,
    cmd_size,
} cmd_type_t;

typedef enum
{
    CMD_FILL_NONE = 0,
    CMD_SAFE_IDLE,
    CMD_FILLING_N2,
    CMD_PRE_PRESSURE,
    CMD_FILLING_N2O,
    CMD_POST_PRESSURE,
} fill_cmd_t;

typedef enum
{
    CMD_MANUAL_SD_LOG_START,
    CMD_MANUAL_SD_LOG_STOP,
    CMD_MANUAL_SD_STATUS,
    CMD_MANUAL_VALVE_STATE,
    CMD_MANUAL_VALVE_MS,
    CMD_MANUAL_ACK,
    manual_cmd_size,
} manual_command_t;

typedef enum
{
    // Rocket Valves
    VALVE_PRESSURIZING,
    VALVE_VENT,
    VALVE_ABORT,
    VALVE_MAIN,

    // Filling Station Valves
    VALVE_N2O_FILL,
    VALVE_N2O_PURGE,
    VALVE_N2_FILL,
    VALVE_N2_PURGE,

    // Filling Station Quick Disconnects
    VALVE_N2O_QUICK_DC,
    VALVE_N2_QUICK_DC,

    _VALVE_COUNT,
} valve_t;

// Actuators bitfield definition (13 bits -> store in 2 bytes)
typedef union
{
    struct
    {
        // Rocket valves
        uint16_t v_pressurizing: 1;
        uint16_t v_vent: 1;
        uint16_t v_abort: 1;
        uint16_t v_main: 1;

        // Filling station valves
        uint16_t v_n2o_fill: 1;
        uint16_t v_n2o_purge: 1;
        uint16_t v_n2_fill: 1;
        uint16_t v_n2_purge: 1;

        // E-matches: ignition, drogue, main chute (3 bits)
        uint16_t ematch_ignition: 1;
        uint16_t ematch_drogue: 1;
        uint16_t ematch_main: 1;

        // Filling station Quick Release
        //  NOTE : After launch these should be interpreted
        //       as reserved bits as their state is no longer
        //       relevant.
        uint16_t v_n2o_quick_dc: 1;
        uint16_t v_n2_quick_dc: 1;

        // remaining bits reserved for alignment
        uint16_t reserved: 3;
    };

    struct
    {
        uint16_t rocket_valves_mask: 4;
        uint16_t fill_station_valves_mask: 4;
        uint16_t ematches_mask: 3;
        uint16_t quick_dc_mask: 2;
        uint16_t reserved_mask: 3;
    };

    uint16_t raw;
} actuators_bitmap_t;

typedef union
{
    struct
    {
        int16_t n2o_tank_uf_t1;
        int16_t n2o_tank_uf_t2;
        int16_t n2o_tank_uf_t3;
        int16_t n2o_tank_lf_t1;
        int16_t n2o_tank_lf_t2;
        int16_t chamber_thermo;
        int16_t n2o_line_thermo;
    };
    int16_t raw[7];
} thermocouples_t;

typedef union
{
    struct
    {
        uint16_t n2o_tank_pressure;
        uint16_t chamber_pressure;
        uint16_t n2o_line_pressure;
        uint16_t n2_line_pressure;
        uint16_t quick_dc_pressure;
    };
    uint16_t raw[5];
} pressures_t;

typedef union {
    struct {
        int16_t n2o_bottle_weight;
        int16_t main_tank_weight;
        int16_t thrust_loadcell1;
        int16_t thrust_loadcell2;
        int16_t thrust_loadcell3;
    };
    int16_t raw[5];
} loadcells_t;

typedef struct
{
    state_t state = IDLE;
    pressures_t pressures = {0};
    thermocouples_t thermocouples = {0};
    actuators_bitmap_t actuators = {0};
    loadcells_t loadcells = {0};
} system_data_t;

typedef struct {
    uint16_t target_pressure, trigger_pressure;
    uint16_t target_temperature, trigger_temperature;
    uint16_t target_weight;
} filling_params_t;

typedef struct {
    time_t ignition_begin_time;
    bool ignition_started = false;
} flight_params_t;

#endif // DATA_MODELS_H