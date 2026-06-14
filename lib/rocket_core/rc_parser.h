/**
 * @file rc_parser.h
 * @brief Pure (I/O-free, timing-free) frame parser + serializer.
 *
 * This is the byte-at-a-time state machine previously embedded in each board's
 * Comms.cpp `parse_input()`, lifted out verbatim in behaviour so it can be reused
 * and, crucially, unit-tested on the host. The transport wrappers
 * (read_packet/write_packet over RS485/UART/LoRa, timeouts, the DE pin) stay
 * per-board and call into these functions.
 *
 * Target-agnostic: depends only on <stdint.h> / <stddef.h>.
 */
#ifndef ROCKET_CORE_RC_PARSER_H
#define ROCKET_CORE_RC_PARSER_H

#include <stddef.h>
#include <stdint.h>
#include "rc_packet.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    RC_SYNC = 0,
    RC_SENDER_ID,
    RC_TARGET_ID,
    RC_CMD,
    RC_PAYLOAD_SIZE,
    RC_PAYLOAD,
    RC_CRC1, // high byte of crc
    RC_CRC2, // low byte of crc
    RC_END,
} rc_parse_state_t;

/**
 * Feed one received byte into the frame state machine.
 *
 * @param read_byte  the byte just received
 * @param packet     packet being assembled (cleared on each fresh SYNC)
 * @param state      current parser state
 * @return the next parser state. RC_END means a full frame is in `packet`
 *         (CRC bytes captured but NOT validated here -- validation is a separate,
 *         currently-disabled, step).
 */
rc_parse_state_t rc_parse_byte(uint8_t read_byte, packet_t *packet, rc_parse_state_t state);

/**
 * Serialize a packet into a byte buffer in wire order.
 *
 * @param packet  source packet (sender/target/cmd/payload_size/payload/crc used)
 * @param buf     destination buffer
 * @param buflen  capacity of @p buf
 * @return number of bytes written, or 0 if the buffer is too small.
 */
size_t rc_serialize(const packet_t *packet, uint8_t *buf, size_t buflen);

#ifdef __cplusplus
}
#endif

#endif // ROCKET_CORE_RC_PARSER_H
