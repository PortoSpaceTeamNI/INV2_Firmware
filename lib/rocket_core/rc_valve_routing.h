/**
 * @file rc_valve_routing.h
 * @brief Pure (I/O-free) logical-valve → HYDRA-node + physical-valve mapping.
 *
 * Defines the logical valve enum (valve_t), the physical HYDRA valve enum
 * (hydra_valve_t), the ValveRoute struct, and the rc_get_valve_route() lookup
 * function. Target-agnostic: depends only on <stdint.h> and rc_ids.h.
 *
 * The CORTEX board's ValveRouting.h includes this header and delegates its
 * getValveRoute() wrapper to rc_get_valve_route().
 */
#ifndef ROCKET_CORE_RC_VALVE_ROUTING_H
#define ROCKET_CORE_RC_VALVE_ROUTING_H

#include <stdint.h>
#include "rc_ids.h"

#ifdef __cplusplus
extern "C" {
#endif

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

typedef struct {
    uint8_t hydraId;
    uint8_t hydraValve;
} ValveRoute;

/**
 * Look up the HYDRA node ID and physical valve index for a logical valve.
 *
 * @param valve   logical valve identifier
 * @param route   output: filled with hydraId and hydraValve on success
 * @return 0 on success, -1 if route is NULL or valve is out of range
 */
int rc_get_valve_route(valve_t valve, ValveRoute *route);

#ifdef __cplusplus
}
#endif

#endif // ROCKET_CORE_RC_VALVE_ROUTING_H
