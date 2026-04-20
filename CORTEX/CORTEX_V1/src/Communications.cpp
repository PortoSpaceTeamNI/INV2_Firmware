#include <Arduino.h>

#include "Communications.h"
#include "Pinout.h"
#include "Commands.h"
#include "Configs.h"

bool check_crc(packet_t *packet)
{
    /* TODO: Implement CRC */
    return true;
}

static cmd_parse_state_t parse_input(uint8_t read_byte, packet_t *packet, cmd_parse_state_t cmd_state)
{
    uint8_t state = cmd_state;
    // Serial1.printf("State: %d - RX: 0x%x\n", state, read_byte);
    switch (state)
    {
    case SYNC:
        if (read_byte != SYNC_BYTE)
            break;
        // start timeout timer
        state = SENDER_ID;
        packet->data_recv = 0;
        memset(packet, 0, sizeof(packet_t));
        packet->begin = clock();
        break;
    case SENDER_ID:
        packet->sender_id = read_byte;
        state = TARGET_ID;
        break;
    case TARGET_ID:
        packet->target_id = read_byte;
        state = CMD;
        break;
    case CMD:
        packet->cmd = read_byte;
        state = PAYLOAD_SIZE;
        break;
    case PAYLOAD_SIZE:
        packet->payload_size = read_byte;
        if (packet->payload_size > MAX_PAYLOAD_SIZE)
        {
            state = SYNC;
            break;
        }
        if (packet->payload_size == 0)
            state = CRC1;
        else
            state = PAYLOAD;
        break;
    case PAYLOAD:
        if (packet->data_recv >= MAX_PAYLOAD_SIZE)
        {
            state = SYNC;
            break;
        }
        packet->payload[packet->data_recv++] = read_byte;
        if (packet->data_recv == packet->payload_size)
            state = CRC1;
        break;
    case CRC1:
        packet->crc = read_byte << 8;
        state = CRC2;
        break;
    case CRC2:
        packet->crc += read_byte;
        state = END;
        break;
    default:
        state = SYNC;
    };
    return (cmd_parse_state_t)state;
}

/* RS485 */
int write_to_rs485(uint8_t *buffer, size_t size)
{
#if RS485_DEBUG_LOG
    Serial.println("Writing to RS485:");
    for (size_t i = 0; i < size; i++)
    {
        Serial.print("0x");
        Serial.println(buffer[i], HEX);
    }
#endif

    // Keep DE asserted long enough for all bits to leave the transceiver.
    // 1 start + 8 data + 1 stop bit = 10 bits per byte (8N1).
    const unsigned long frameTimeUs = (unsigned long)(((size * 10UL * 1000000UL) + (RS485_BAUD_RATE - 1)) / RS485_BAUD_RATE);

    digitalWrite(ENABLE_RS_PIN, RS485_TX_ENABLE_LEVEL); // switch to transmit mode
    delayMicroseconds(RS485_TX_ENABLE_SETUP_US);
    if (Serial2.write(buffer, size) != size)
    {
        digitalWrite(ENABLE_RS_PIN, RS485_RX_ENABLE_LEVEL); // switch back to receive mode
        return -1;
    }
    Serial2.flush();
    delayMicroseconds(frameTimeUs + RS485_TX_ENABLE_HOLD_US);
    digitalWrite(ENABLE_RS_PIN, RS485_RX_ENABLE_LEVEL); // switch back to receive mode
    return 0;
}

void read_from_rs485(uint8_t *read_byte, packet_t *packet, cmd_parse_state_t *state)
{
    while (Serial2.available() && *state != END)
    {
#if RS485_DEBUG_LOG
        Serial.println(Serial2.peek(), HEX);
#endif
        *read_byte = Serial2.read();
        *state = parse_input(*read_byte, packet, *state);
    }
}

/* UART */
void read_from_serial(uint8_t *read_byte, packet_t *packet, cmd_parse_state_t *state)
{
    while (Serial1.available() && *state != END)
    {
        *read_byte = Serial1.read();
        *state = parse_input(*read_byte, packet, *state);
    }
}

int write_to_serial(uint8_t *buffer, size_t size)
{
    if (Serial1.write(buffer, size) != size)
    {
        Serial.println("Failed to write to serial1");
        return -1;
    }
    return 0;
}

int write_packet(packet_t *packet, interface_t interface)
{
    int size = 0;
    uint8_t buff[MAX_PACKET_SIZE] = {0}; // 2 bytes from CRC

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
        // TODO: Implement LoRa packet reading
        break;
    case RS485_INTERFACE:
        read_from_rs485(&read_byte, packet, state);
        break;
    case UART_INTERFACE:
        read_from_serial(&read_byte, packet, state);
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
    // packet good
    else if (*state == END &&
             (packet->target_id == DEVICE_ID ||
              packet->target_id == BROADCAST_ID) &&
             (check_crc(packet) || !CRC_ENABLED))
    {
        *state = SYNC;
        *error = CMD_READ_OK;
        return packet;
    }
    else if (*state == END) // bad crc
    {
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

int create_packet(packet_t *packet, uint8_t sender_id, uint8_t target_id, uint8_t cmd, uint8_t *payload, uint8_t payload_size)
{
    if (packet == NULL)
    {
        return -1;
    }

    if (payload_size > MAX_PAYLOAD_SIZE)
    {
        return -1;
    }

    if (payload_size > 0 && payload == NULL)
    {
        return -1;
    }

    packet->sender_id = sender_id;
    packet->target_id = target_id;
    packet->cmd = cmd;
    packet->payload_size = payload_size;
    if (payload_size > 0)
    {
        memcpy(packet->payload, payload, payload_size);
    }
    if (payload_size < MAX_PAYLOAD_SIZE)
    {
        memset(packet->payload + payload_size, 0, MAX_PAYLOAD_SIZE - payload_size);
    }
    packet->crc = 0; // TODO: Implement CRC
    return 0;
}