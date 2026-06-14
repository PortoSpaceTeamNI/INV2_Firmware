#include <Arduino.h>

#include "Communications.h"
#include "rc_packet_utils.h"
#include "Pinout.h"
#include "Commands.h"
#include "Configs.h"

bool check_crc(packet_t *packet)
{
    /* TODO: Implement CRC */
    return true;
}

/* The byte-at-a-time frame parser now lives in lib/rocket_core (rc_parse_byte);
 * see read_stream() below for how it's driven. */

/* RS485 */
int write_to_rs485(uint8_t *buffer, size_t size)
{
#if RS485_DEBUG_LOG
    Serial1.println("Writing to RS485:");
    for (size_t i = 0; i < size; i++)
    {
        Serial1.print("0x");
        Serial1.println(buffer[i], HEX);
    }
#endif

    // Keep DE asserted long enough for all bits to leave the transceiver.
    // 1 start + 8 data + 1 stop bit = 10 bits per byte (8N1).
    const unsigned long frameTimeUs = (unsigned long)(((size * 10UL * 1000000UL) + (RS485_BAUD_RATE - 1)) / RS485_BAUD_RATE);

    digitalWrite(ENABLE_RS_PIN, RS485_TX_ENABLE_LEVEL); // switch to transmit mode
    if (Serial2.write(buffer, size) != size)
    {
        digitalWrite(ENABLE_RS_PIN, RS485_RX_ENABLE_LEVEL); // switch back to receive mode
        return -1;
    }
    Serial2.flush();
    digitalWrite(ENABLE_RS_PIN, RS485_RX_ENABLE_LEVEL); // switch back to receive mode
    return 0;
}

/* Feed available bytes from a stream into the shared frame parser.
 * rc_parse_byte is timing-free, so the frame-start timestamp (used for the
 * mid-frame timeout) is captured here, when the SYNC byte is consumed. */
static void read_stream(Stream &port, packet_t *packet, rc_parse_state_t *state, clock_t *begin)
{
    while (port.available() && *state != RC_END)
    {
        uint8_t read_byte = port.read();
        rc_parse_state_t prev = *state;
        *state = rc_parse_byte(read_byte, packet, *state);
        if (prev == RC_SYNC && *state != RC_SYNC)
            *begin = clock(); // a frame just started
    }
}

int write_to_serial(uint8_t *buffer, size_t size)
{
    if (Serial.write(buffer, size) != size)
    {
        Serial1.println("Failed to write to serial");
        return -1;
    }
    return 0;
}

int write_packet(packet_t *packet, interface_t interface)
{
    uint8_t buff[MAX_FRAME_SIZE] = {0};
    size_t size = rc_serialize(packet, buff, sizeof(buff));
    if (size == 0)
        return -1; // payload too large / bad buffer

    switch (interface)
    {
    case LORA_INTERFACE:
        break;
    case RS485_INTERFACE:
        if (write_to_rs485(buff, size) != 0)
        {
            return -1;
        }
        break;
    case UART_INTERFACE:
        if (write_to_serial(buff, size) != 0)
        {
            return -1;
        }
        break;
    default:
        break;
    }
    return 0;
}

packet_t *read_packet(int *error, interface_t interface)
{
    static packet_t packet_arr[interface_t_size];
    static rc_parse_state_t state_arr[interface_t_size] = {RC_SYNC};
    static clock_t begin_arr[interface_t_size] = {0};

    uint8_t index = (uint8_t)interface;
    packet_t *packet = &packet_arr[index];
    rc_parse_state_t *state = &state_arr[index];
    clock_t *begin = &begin_arr[index];

    switch (interface)
    {
    case LORA_INTERFACE:
        // TODO: Implement LoRa packet reading
        break;
    case RS485_INTERFACE:
        read_stream(Serial2, packet, state, begin);
        break;
    case UART_INTERFACE:
        read_stream(Serial, packet, state, begin);
        break;
    default:
        break;
    };

    int msec = (clock() - *begin) * 1000 / CLOCKS_PER_SEC;

    // if timeout reset state
    if (*state != RC_SYNC && msec > RS485_TIMEOUT_TIME_MS) // timeout
    {
        *state = RC_SYNC;

        *error = CMD_READ_TIMEOUT;

        return NULL;
    }
    // packet good
    else if (*state == RC_END &&
             (packet->target_id == DEVICE_ID ||
              packet->target_id == BROADCAST_ID) &&
             (check_crc(packet) || !CRC_ENABLED))
    {
        *state = RC_SYNC;
        *error = CMD_READ_OK;
        return packet;
    }
    else if (*state == RC_END) // not addressed to us / bad crc
    {
        *state = RC_SYNC;
        *error = CMD_READ_BAD_CRC;
        return NULL;
    }
    else // default
    {
        *error = CMD_READ_NO_CMD;
        return NULL;
    }
}

int create_packet(packet_t *packet, uint8_t sender_id, uint8_t target_id, uint8_t cmd, uint8_t *payload, uint8_t payload_size)
{
    return rc_create_packet(packet, sender_id, target_id, cmd, payload, payload_size);
}