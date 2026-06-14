/**
 * @file rc_packet_utils.h
 * @brief Portable packet construction helper.
 *
 * rc_create_packet() fills a packet_t in one call. It is the single-copy
 * implementation that board-specific transport layers (Communications.cpp on
 * CORTEX, Comms.cpp on HYDRA/LIFT) delegate to. Target-agnostic: depends
 * only on rc_packet.h and <string.h>.
 */
#ifndef ROCKET_CORE_RC_PACKET_UTILS_H
#define ROCKET_CORE_RC_PACKET_UTILS_H

#include <stdint.h>
#include "rc_packet.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Populate a packet_t from its fields.
 *
 * Copies payload_size bytes from payload into packet->payload and zeroes the
 * remainder of the payload buffer. Sets crc to 0 (CRC is currently disabled).
 *
 * @param packet       destination struct (must not be NULL)
 * @param sender_id    sender board ID
 * @param target_id    target board ID
 * @param cmd          command byte
 * @param payload      source payload bytes (may be NULL only when payload_size == 0)
 * @param payload_size number of payload bytes to copy
 * @return 0 on success, -1 on any argument error
 */
int rc_create_packet(packet_t *packet,
                     uint8_t   sender_id,
                     uint8_t   target_id,
                     uint8_t   cmd,
                     uint8_t  *payload,
                     uint8_t   payload_size);

#ifdef __cplusplus
}
#endif

#endif // ROCKET_CORE_RC_PACKET_UTILS_H
