/**
 * @file Comms.h
 * @author AF
 * @brief
 *      Shared file bettewn all the systems,
 * @version 0.1
 * @date 2024-02-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef _COMMS_H_
#define _COMMS_H_

#include <inttypes.h>
#include <time.h>

#define SYNC_BYTE 0x55
#define MAX_PAYLOAD_SIZE 200
#define RS485_TIMEOUT_TIME_MS 50 // try to get limit bounds

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
    time_t begin;
} packet_t;

#define HEADER_SIZE 5 // sync + sender_id + target_id + cmd + payload_size

#define MAX_PACKET_SIZE (1 + HEADER_SIZE + MAX_PAYLOAD_SIZE + 2)

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

typedef enum
{
    LORA_INTERFACE,
    RS485_INTERFACE,
    UART_INTERFACE,
    interface_t_size
} interface_t;

#define CRC_ENABLED false

// Board IDs
#define CORTEX_ID 1
#define HYDRA_UF_ID 2
#define HYDRA_LF_ID 3
#define HYDRA_FS_ID 4
#define NAVIGATOR_ID 5
#define LIFT_TANK_ID 6
#define LIFT_BOTTLE_ID 7
#define LIFT_THRUST_ID 8
#define BROADCAST_ID 0xFF

void write_packet(packet_t *cmd, interface_t interface);
packet_t *read_packet(int *error, interface_t interface);
int create_packet(packet_t *packet, uint8_t sender_id, uint8_t target_id, uint8_t cmd, uint8_t *payload, uint8_t payload_size);

#endif