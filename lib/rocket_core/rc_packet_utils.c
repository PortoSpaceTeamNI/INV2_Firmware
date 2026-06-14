#include "rc_packet_utils.h"

#include <string.h>

int rc_create_packet(packet_t *packet,
                     uint8_t   sender_id,
                     uint8_t   target_id,
                     uint8_t   cmd,
                     uint8_t  *payload,
                     uint8_t   payload_size)
{
    if (packet == NULL)
        return -1;

    if (payload_size > MAX_PAYLOAD_SIZE)
        return -1;

    if (payload_size > 0 && payload == NULL)
        return -1;

    packet->sender_id    = sender_id;
    packet->target_id    = target_id;
    packet->cmd          = cmd;
    packet->payload_size = payload_size;

    if (payload_size > 0)
        memcpy(packet->payload, payload, payload_size);

    if (payload_size < MAX_PAYLOAD_SIZE)
        memset(packet->payload + payload_size, 0, MAX_PAYLOAD_SIZE - payload_size);

    packet->crc = 0; // CRC computation deferred; CRC_ENABLED is false

    return 0;
}
