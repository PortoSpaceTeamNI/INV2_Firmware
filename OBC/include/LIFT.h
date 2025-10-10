#ifndef _LIFT_H_
#define _LIFT_H_

#include <inttypes.h>
#include <time.h>

#include "HardwareCfg.h"
#include "DataModels.h"
#include "Comms.h"

typedef enum {
    LIFT_TANK = 0,
    LIFT_BOTTLE,
    LIFT_THRUST,
    lift_id_count,
} lift_id_t;

typedef enum {
    LCMD_STATUS = 0,
    LCMD_VALVE_SET,
    LCMD_VALVE_MS,
    LCMD_ACK,
} lift_cmd_t;

typedef struct {
    lift_id_t id;
    loadcells_t data;
} lift_t;

void init_lift(lift_t *lift);
int update_data_from_lift(lift_t *lift, system_data_t *system_data);
int parse_lift_response(lift_t lifts[], packet_t *packet, system_data_t *system_data);
int fetch_next_lift(lift_t lifts[], system_data_t *system_data);

#endif