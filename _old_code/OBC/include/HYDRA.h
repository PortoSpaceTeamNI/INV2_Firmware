#ifndef HYDRA_H
#define HYDRA_H

#include <stdint.h>
#include <stdbool.h>
#include "Comms.h"
#include "DataModels.h"

typedef enum {
    HYDRA_UF = 0,
    HYDRA_LF,
    HYDRA_FS,
    hydra_id_count,
} hydra_id_t;

typedef enum {
    H_VALVE_QUICK_DC_1 = 0,
    H_VALVE_QUICK_DC_2,
    H_VALVE_CONTROLLED_1,
    H_VALVE_CONTROLLED_2,
    H_VALVE_CONTROLLED_3,
    H_VALVE_STEEL_BALL_1,
    H_VALVE_STEEL_BALL_2,
    H_VALVE_SERVO,
    hydra_valve_count,
} hydra_valve_t;

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
} hydra_valve_states_t;

typedef struct {
    uint16_t thermo1, thermo2, thermo3;
    uint16_t pressure1, pressure2, pressure3;
    hydra_valve_states_t valve_states;
    bool cam_enable;
} hydra_data_t;

typedef struct {
    hydra_id_t id;
    hydra_data_t data;
} hydra_t;

void init_hydra(hydra_t *hydra);
int update_data_from_hydra(hydra_t *hydra, system_data_t *system_data);
int set_hydra_valve(hydra_t *hydra, hydra_valve_t valve, bool state);
int set_hydra_valve_ms(hydra_t *hydra, hydra_valve_t valve, uint16_t ms);

int send_hydra_command(hydra_t *hydra, command_t cmd, uint8_t payload[], uint8_t payload_size);
int parse_hydra_response(hydra_t hydras[], packet_t *packet, system_data_t *system_data);

int fetch_next_hydra(hydra_t hydras[], system_data_t *system_data);

#endif // HYDRA_H