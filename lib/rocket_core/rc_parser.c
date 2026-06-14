#include "rc_parser.h"

#include <string.h>

rc_parse_state_t rc_parse_byte(uint8_t read_byte, packet_t *packet, rc_parse_state_t state)
{
    switch (state)
    {
    case RC_SYNC:
        if (read_byte == SYNC_BYTE)
        {
            memset(packet, 0, sizeof(*packet));
            packet->data_recv = 0;
            state = RC_SENDER_ID;
        }
        break;

    case RC_SENDER_ID:
        packet->sender_id = read_byte;
        state = RC_TARGET_ID;
        break;

    case RC_TARGET_ID:
        packet->target_id = read_byte;
        state = RC_CMD;
        break;

    case RC_CMD:
        packet->cmd = read_byte;
        state = RC_PAYLOAD_SIZE;
        break;

    case RC_PAYLOAD_SIZE:
        packet->payload_size = read_byte;
        // Clamp to capacity so a corrupt length byte can't overflow payload[].
        if (packet->payload_size > MAX_PAYLOAD_SIZE)
            packet->payload_size = MAX_PAYLOAD_SIZE;
        state = (packet->payload_size == 0) ? RC_CRC1 : RC_PAYLOAD;
        break;

    case RC_PAYLOAD:
        packet->payload[packet->data_recv++] = read_byte;
        if (packet->data_recv == packet->payload_size)
            state = RC_CRC1;
        break;

    case RC_CRC1:
        packet->crc = (uint16_t)(read_byte << 8);
        state = RC_CRC2;
        break;

    case RC_CRC2:
        packet->crc = (uint16_t)(packet->crc + read_byte);
        state = RC_END;
        break;

    default:
        state = RC_SYNC;
        break;
    }

    return state;
}

size_t rc_serialize(const packet_t *packet, uint8_t *buf, size_t buflen)
{
    size_t payload_size = packet->payload_size;
    if (payload_size > MAX_PAYLOAD_SIZE)
        payload_size = MAX_PAYLOAD_SIZE;

    size_t needed = HEADER_SIZE + payload_size + CRC_SIZE;
    if (buf == NULL || buflen < needed)
        return 0;

    size_t i = 0;
    buf[i++] = SYNC_BYTE;
    buf[i++] = packet->sender_id;
    buf[i++] = packet->target_id;
    buf[i++] = packet->cmd;
    buf[i++] = (uint8_t)payload_size;
    for (size_t p = 0; p < payload_size; p++)
        buf[i++] = packet->payload[p];
    buf[i++] = (uint8_t)((packet->crc >> 8) & 0xFF);
    buf[i++] = (uint8_t)(packet->crc & 0xFF);

    return i;
}
