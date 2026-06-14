#include <Arduino.h>

#include <inttypes.h>
#include <time.h>
#include <string.h>

#include "Comms.h"
#include "GlobalVars.h"
#include "HardwareCfg.h"

#include <Crc.h>
#include <LoRa.h>

bool check_crc(packet_t *packet);

void write_packet(packet_t *packet, interface_t interface)
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
    switch (interface)
    {
    case LORA_INTERFACE:
        break;
    case RS485_INTERFACE:
        digitalWrite(ENABLE_RS_PIN, HIGH); // switch to transmit mode
        Serial2.write(buff, size);
        Serial2.flush();
        digitalWrite(ENABLE_RS_PIN, LOW); // switch back to receive mode
        break;
    case UART_INTERFACE:
        Serial.write(buff, size);
        break;
    default:
        break;
    }
}

static cmd_parse_state_t parse_input(uint8_t read_byte, packet_t *packet, cmd_parse_state_t cmd_state)
{
    uint8_t state = cmd_state;
    // Serial.printf("State: %d - RX: 0x%x\n", state, read_byte);
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
            packet->begin = clock();
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

packet_t *read_packet(int *error, interface_t interface)
{
    static packet_t packet_arr[interface_t_size];
    static cmd_parse_state_t state_arr[interface_t_size] = {SYNC};
    static clock_t end_arr[interface_t_size] = {0};

    uint8_t index = (uint8_t)interface;
    packet_t *packet = &packet_arr[index];
    cmd_parse_state_t *state = &state_arr[index];
    clock_t *end = &end_arr[index];

    size_t size;
    uint8_t read_byte;

    switch (interface)
    {
    case LORA_INTERFACE:
    {
        // Work arourd for lora to not spam the spi bus
        static unsigned int begin = 0, end = 0;
        end = millis();
        if (end - begin < 5)
            break;
        begin = end;

        int packetSize = LoRa.parsePacket();
        while (packetSize != 0 && LoRa.available() && *state != END)
        {
            read_byte = LoRa.read();
            // printf("got byte %x\n", read_byte);
            *state = parse_input(read_byte, packet, *state);
        }
    }
    break;

    case RS485_INTERFACE:
    {
        while (Serial2.available() && *state != END)
        {
            read_byte = Serial2.read();
            *state = parse_input(read_byte, packet, *state);
        }
    }
    break;

    case UART_INTERFACE:
    {
        while (Serial.available() && *state != END)
        {
            read_byte = Serial.read();
            *state = parse_input(read_byte, packet, *state);
        }
    }
    break;

    default:
        break;
    };

    *end = clock();
    int msec = (*end - packet->begin) * 1000 / CLOCKS_PER_SEC;

    // if timeout reset state
    if (*state != SYNC && msec > RS485_TIMEOUT_TIME_MS) // timeout
    {
        *state = SYNC;

        *error = CMD_READ_TIMEOUT;

        return NULL;
    }
    // if bad cr reset state
    else if (*state == END &&
             (packet->target_id == DEFAULT_ID ||
              packet->target_id == BROADCAST_ID) &&
             (check_crc(packet) || !CRC_ENABLED))
    {
        *state = SYNC;

        *error = CMD_READ_OK;

        return packet;
    }
    else if (*state == END)
    {
        // uint8_t* ptr = (uint8_t*)packet;
        // printf("%x %x %x\n", packet->cmd, packet->id, packet->size);
        // for(int i = 0; i < packet->size + 3; i++)
        //{
        // printf("%x ", (uint8_t)(*(ptr + i)));
        //}
        // printf("\nCrc: %x %x %x \n", packet->crc , (packet->crc >> 8) & 0xff, (packet->crc) & 0xff);
        *state = SYNC;

        *error = CMD_READ_BAD_CRC;
        return NULL;
    }
    else // default
    {
        *error = CMD_READ_NO_CMD;
        return NULL;
    }
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