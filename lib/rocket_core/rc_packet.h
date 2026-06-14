/**
 * @file rc_packet.h
 * @brief Canonical on-the-wire frame for the INV2 byte protocol.
 *
 * Wire layout (unchanged from the per-board copies it replaces):
 *
 *   0x55 | sender_id | target_id | cmd | payload_size | payload[payload_size] | crc_hi | crc_lo
 *
 * Only the protocol-relevant fields live here plus one parser scratch field
 * (`data_recv`). Transport-level concerns (timeout reference timestamps, the
 * RS485 DE pin, the interface enum) deliberately do NOT live in this struct --
 * they stay in each board's transport layer. This keeps the frame definition and
 * the parser host-compilable for unit tests.
 *
 * Target-agnostic: depends only on <stdint.h>.
 */
#ifndef ROCKET_CORE_RC_PACKET_H
#define ROCKET_CORE_RC_PACKET_H

#include <stdint.h>

#define SYNC_BYTE        0x55
#define MAX_PAYLOAD_SIZE 150
#define HEADER_SIZE      5  // sync + sender + target + cmd + payload_size
#define CRC_SIZE         2

/** Maximum number of bytes a fully serialized frame can occupy. */
#define MAX_FRAME_SIZE (HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE)

typedef struct __attribute__((__packed__))
{
    uint8_t  sender_id;
    uint8_t  target_id;
    uint8_t  cmd;
    uint8_t  payload_size;
    uint8_t  payload[MAX_PAYLOAD_SIZE];
    uint16_t crc;

    uint8_t  data_recv; // parser scratch: bytes of payload received so far
} packet_t;

#endif // ROCKET_CORE_RC_PACKET_H
