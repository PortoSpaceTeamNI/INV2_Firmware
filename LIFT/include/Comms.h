#ifndef _COMMS_H_
#define _COMMS_H_

#include <inttypes.h>
#include <time.h>

#include "HardwareCfg.h"

#define SYNC_BYTE 0x55

#define MAX_PAYLOAD_SIZE 150
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

typedef struct __attribute__((__packed__))
// typedef struct
{
    uint8_t sender_id;
    uint8_t target_id;
    uint8_t cmd;
    uint8_t payload_size;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint16_t crc;

    uint8_t data_recv; // helper pointer to fill data[]
    unsigned long begin;
} packet_t;

typedef enum
{
    SYNC = 0,
    SENDER_ID,
    TARGET_ID,
    CMD,
    PAYLOAD_SIZE,
    PAYLOAD,
    CRC1, // first byte of crc
    CRC2, // second byte of crc
    END,
} cmd_parse_state_t;


#define HEADER_SIZE 5

#define CRC_ENABLED false

#define GROUND_ID 0
#define OBC_ID 1
#define HYDRA_UF_ID 2
#define HYDRA_LF_ID 3
#define HYDRA_FS_ID 4
#define NAVIGATOR_ID 5
#define LIFT_TANK_ID 6
#define LIFT_BOTTLE_ID 7
#define LIFT_THRUST_ID 8
#define BROADCAST_ID 0xFF

#define DEFAULT_ID LIFT_TANK_ID // Change this for each board

void write_packet(packet_t *cmd);
packet_t *read_packet(int *error);

#endif