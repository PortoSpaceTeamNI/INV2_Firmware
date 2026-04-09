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
    int size = 0;
    uint8_t buff[HEADER_SIZE + MAX_PAYLOAD_SIZE + 2] = {0}; // 2 bytes from CRC

    buff[size++] = SYNC_BYTE;
    buff[size++] = packet->sender_id;
    buff[size++] = packet->target_id;
    buff[size++] = packet->cmd;
    buff[size++] = packet->payload_size;
    for (int i = 0; i < packet->payload_size; i++)
        buff[size++] = packet->payload[i];
    buff[size++] = ((packet->crc >> 8) & 0xff);
    buff[size++] = ((packet->crc) & 0xff);
    for (int i = 0; i < size; i++) {
        Serial.print(buff[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    Serial.println("Packet sent!");
    rs485_send(buff, size);
}

static cmd_parse_state_t parse_input(uint8_t read_byte, packet_t *packet, cmd_parse_state_t cmd_state)
{
    uint8_t state = cmd_state;
    switch (state)
    {
    case SYNC:
    {
        if (read_byte == SYNC_BYTE)
        {
            // start timeout timer
            state = SENDER_ID;
            packet->data_recv = 0;
            memset(packet, 0, sizeof(packet_t));
            packet->begin = millis();
        }
    }
    break;

    case SENDER_ID:
    {
        packet->sender_id = read_byte;
        state = TARGET_ID;
    }
    break;

    case TARGET_ID:
    {
        packet->target_id = read_byte;
        state = CMD;
    }
    break;

    case CMD:
    {
        packet->cmd = read_byte;
        state = PAYLOAD_SIZE;
    }
    break;

    case PAYLOAD_SIZE:
    {
        packet->payload_size = read_byte;
        if (packet->payload_size == 0)
            state = CRC1;
        else
            state = PAYLOAD;
    }
    break;

    case PAYLOAD:
    {
        packet->payload[packet->data_recv++] = read_byte;
        if (packet->data_recv == packet->payload_size)
            state = CRC1;
    }
    break;

    case CRC1:
    {
        packet->crc = read_byte << 8;
        state = CRC2;
    }
    break;

    case CRC2:
    {
        packet->crc += read_byte;
        state = END;
    }
    break;

    default:
        state = SYNC;
    };

    return (cmd_parse_state_t)state;
}

packet_t *read_packet(int *error)
{
    static packet_t packet;
    static cmd_parse_state_t state = SYNC;
    static clock_t end = 0;

    size_t size;
    uint8_t read_byte;

    if (Serial1.available() && state != END)
    {
        read_byte = Serial1.read();
        state = parse_input(read_byte, &packet, state);
    }

    end = millis();
    int msec = end - packet.begin;

    // if timeout reset state
    if (state != SYNC && msec > RS485_TIMEOUT_TIME_MS) // timeout
    {

        state = SYNC;

        *error = CMD_READ_TIMEOUT;

        return NULL;
    }
    else if (state == END)
    {
        state = SYNC;
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