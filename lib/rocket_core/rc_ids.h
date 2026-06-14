/**
 * @file rc_ids.h
 * @brief Canonical board / node IDs for the INV2 RS485 + LoRa + UART bus.
 *
 * Single source of truth. Previously these #defines were copy-pasted into every
 * board's Comms.h (OBC, HYDRA, LIFT) and had begun to drift (only LIFT defined
 * GROUND_ID). Target-agnostic: safe to compile on the host for unit tests.
 */
#ifndef ROCKET_CORE_RC_IDS_H
#define ROCKET_CORE_RC_IDS_H

#define GROUND_ID      0
#define OBC_ID         1
#define HYDRA_UF_ID    2
#define HYDRA_LF_ID    3
#define HYDRA_FS_ID    4
#define NAVIGATOR_ID   5
#define LIFT_TANK_ID   6
#define LIFT_BOTTLE_ID 7
#define LIFT_THRUST_ID 8
#define BROADCAST_ID   0xFF

#endif // ROCKET_CORE_RC_IDS_H
