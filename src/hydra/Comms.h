#ifndef _COMMS_H_
#define _COMMS_H_

#include <inttypes.h>
#include <time.h>

// Shared protocol core: board IDs, the on-the-wire frame (packet_t, SYNC_BYTE,
// MAX_PAYLOAD_SIZE, HEADER_SIZE, MAX_FRAME_SIZE) and the pure byte parser /
// serializer (rc_parse_byte / rc_serialize). Single source of truth in
// lib/rocket_core -- see Documentation/monorepo-shared-lib-refactor.md.
#include "rc_ids.h"
#include "rc_packet.h"
#include "rc_parser.h"

#include "HardwareCfg.h"

#define RS485_TIMEOUT_TIME_MS 1000

/*
    Read command errors
 */
#define CMD_READ_OK 0
#define CMD_READ_TIMEOUT 1
#define CMD_READ_BAD_CRC 2
#define CMD_READ_DEFAULT_ERROR 3
#define CMD_READ_NO_CMD 4

/*
    Run command errors
 */
#define CMD_RUN_OK 0
#define CMD_RUN_ARM_ERROR 1
#define CMD_RUN_DEFAULT_ERROR 2
#define CMD_RUN_NO_ACTION 3
#define CMD_RUN_OUT_OF_BOUND 4


//----------------------------------------

#define CRC_ENABLED false

// Per-board identity. Normally supplied via -DDEFAULT_ID=HYDRA_xx_ID in the
// env's build_flags (see root platformio.ini). Falls back to UF if not set.
#ifndef DEFAULT_ID
#define DEFAULT_ID HYDRA_UF_ID
#endif

void write_packet(packet_t *cmd);
packet_t *read_packet(int *error);

#endif