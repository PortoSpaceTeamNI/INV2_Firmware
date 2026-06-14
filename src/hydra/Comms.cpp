#include <Arduino.h>

#include <inttypes.h>
#include <time.h>
#include <string.h>

#include "Peripherals/IO_Map.h"
#include "Comms.h"
#include <Crc.h>

#include "Peripherals/RS485.h"

bool check_crc(packet_t *packet);

void write_packet(packet_t *packet)
{
    uint8_t buff[MAX_FRAME_SIZE] = {0};
    size_t size = rc_serialize(packet, buff, sizeof(buff));
    if (size == 0)
        return; // payload too large / bad buffer

    rs485_send(buff, size);
}

// The byte-at-a-time frame parser now lives in lib/rocket_core (rc_parse_byte).
// rc_parse_byte is timing-free, so the frame-start timestamp (for the mid-frame
// timeout below) is captured here, when the SYNC byte is consumed.

packet_t *read_packet(int *error)
{
    static packet_t packet;
    static rc_parse_state_t state = RC_SYNC;
    static unsigned long begin = 0;

    while (Serial2.available() && state != RC_END)
    {
        uint8_t read_byte = Serial2.read();
        rc_parse_state_t prev = state;
        state = rc_parse_byte(read_byte, &packet, state);
        if (prev == RC_SYNC && state != RC_SYNC)
            begin = millis(); // a frame just started
    }

    // if timeout reset state
    if (state != RC_SYNC && (millis() - begin) > RS485_TIMEOUT_TIME_MS) // timeout
    {

        state = RC_SYNC;

        *error = CMD_READ_TIMEOUT;

        return NULL;
    }
    else if (state == RC_END)
    {
        state = RC_SYNC;
        if (packet.target_id == DEFAULT_ID ||
            packet.target_id == BROADCAST_ID)
        {
            
            if (CRC_ENABLED)
            {
                if (check_crc(&packet))
                {
                    *error = CMD_READ_OK;
                    return &packet;
                }
            }
            else
            {
                *error = CMD_READ_OK;
                return &packet;
            }
        }
        else
        {
            *error = CMD_READ_NO_CMD;
            return NULL;
        }
        return NULL;
    }
    else // default
    {
        *error = CMD_READ_NO_CMD;
        return NULL;
    }
    return NULL;
}

bool check_crc(packet_t *packet)
{
    /* TODO: Implement CRC
    uint16_t crc1, crc2;

    crc1 = packet->crc;
    crc2 = crc((unsigned char*)packet, packet->payload_size + 3); //+3 bytes cmd, id, size

    return (crc1 == crc2);
    */
    return true;
}