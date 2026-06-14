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
#include <stddef.h>
#include <time.h>

// Shared protocol core: board IDs, the on-the-wire frame (packet_t, SYNC_BYTE,
// MAX_PAYLOAD_SIZE, HEADER_SIZE, MAX_FRAME_SIZE) and the pure byte parser /
// serializer (rc_parse_byte / rc_serialize). Single source of truth in
// lib/rocket_core -- see Documentation/monorepo-shared-lib-refactor.md.
#include "rc_ids.h"
#include "rc_packet.h"
#include "rc_parser.h"

#define RS485_TIMEOUT_TIME_MS 100 // mid-frame timeout; max frame ~14 ms at 115200, 100 ms gives safe headroom

// Largest fully-serialized frame on the wire (sync..crc), from rc_packet.h.
#define MAX_PACKET_SIZE MAX_FRAME_SIZE

typedef enum
{
    LORA_INTERFACE,
    RS485_INTERFACE,
    UART_INTERFACE,
    interface_t_size
} interface_t;

#define CRC_ENABLED false

int write_packet(packet_t *cmd, interface_t interface);
packet_t *read_packet(int *error, interface_t interface);
int create_packet(packet_t *packet, uint8_t sender_id, uint8_t target_id, uint8_t cmd, uint8_t *payload, uint8_t payload_size);
int write_to_rs485(uint8_t *buffer, size_t size);

#endif // _COMMS_H_