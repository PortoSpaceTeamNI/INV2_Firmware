#include "Comms.h"

#include <hardware/gpio.h>
#include <hardware/uart.h>
#include <math.h>
#include <string.h>

static uart_inst_t* sensorUart = uart1;
static nav_packet_t rxPacket = {};
static nav_parse_state_t rxState = NAV_SYNC;

static bool check_crc(const nav_packet_t *packet)
{
    (void)packet;
    // TODO 
    return true;
}

static int create_packet(nav_packet_t *packet, uint8_t senderId, uint8_t targetId, uint8_t cmd, const uint8_t *payload, uint8_t payloadSize)
{
    if (packet == nullptr)
    {
        return -1;
    }

    if (payloadSize > NAV_MAX_PAYLOAD_SIZE)
    {
        return -1;
    }

    if (payloadSize > 0 && payload == nullptr)
    {
        return -1;
    }

    packet->sender_id = senderId;
    packet->target_id = targetId;
    packet->cmd = cmd;
    packet->payload_size = payloadSize;
    if (payloadSize > 0)
    {
        memcpy(packet->payload, payload, payloadSize);
    }
    packet->crc = 0;
    return 0;
}

static int write_packet(const nav_packet_t *packet)
{
    uint8_t buffer[NAV_MAX_PACKET_SIZE] = {0};
    int size = 0;

    buffer[size++] = NAV_SYNC_BYTE;
    buffer[size++] = packet->sender_id;
    buffer[size++] = packet->target_id;
    buffer[size++] = packet->cmd;
    buffer[size++] = packet->payload_size;

    for (int i = 0; i < packet->payload_size; i++)
    {
        buffer[size++] = packet->payload[i];
    }

    buffer[size++] = ((packet->crc >> 8) & 0xFF);
    buffer[size++] = (packet->crc & 0xFF);

    const int written = uart_write_blocking(sensorUart, buffer, size);
    return (written == size) ? 0 : -1;
}

static nav_parse_state_t parse_input(uint8_t readByte, nav_packet_t *packet, nav_parse_state_t state)
{
    switch (state)
    {
    case NAV_SYNC:
        if (readByte != NAV_SYNC_BYTE)
        {
            return NAV_SYNC;
        }
        memset(packet, 0, sizeof(nav_packet_t));
        packet->begin_ms = millis();
        return NAV_SENDER_ID;
    case NAV_SENDER_ID:
        packet->sender_id = readByte;
        return NAV_TARGET_ID;
    case NAV_TARGET_ID:
        packet->target_id = readByte;
        return NAV_CMD;
    case NAV_CMD:
        packet->cmd = readByte;
        return NAV_PAYLOAD_SIZE;
    case NAV_PAYLOAD_SIZE:
        packet->payload_size = readByte;
        if (packet->payload_size > NAV_MAX_PAYLOAD_SIZE)
        {
            return NAV_SYNC;
        }
        if (packet->payload_size == 0)
        {
            return NAV_CRC1;
        }
        return NAV_PAYLOAD;
    case NAV_PAYLOAD:
        if (packet->data_recv >= packet->payload_size || packet->data_recv >= NAV_MAX_PAYLOAD_SIZE)
        {
            return NAV_SYNC;
        }
        packet->payload[packet->data_recv++] = readByte;
        if (packet->data_recv == packet->payload_size)
        {
            return NAV_CRC1;
        }
        return NAV_PAYLOAD;
    case NAV_CRC1:
        packet->crc = (uint16_t)readByte << 8;
        return NAV_CRC2;
    case NAV_CRC2:
        packet->crc |= readByte;
        return NAV_END;
    case NAV_END:
    default:
        return NAV_SYNC;
    }
}

static bool read_packet(nav_packet_t *packet)
{
    while (uart_is_readable(sensorUart) && rxState != NAV_END)
    {
        uint8_t readByte = (uint8_t)uart_getc(sensorUart);
        rxState = parse_input(readByte, &rxPacket, rxState);
    }

    const uint32_t nowMs = millis();
    if (rxState != NAV_SYNC && rxPacket.begin_ms != 0 && (nowMs - rxPacket.begin_ms) > NAV_PACKET_TIMEOUT_MS)
    {
        rxState = NAV_SYNC;
        return false;
    }

    if (rxState == NAV_END)
    {
        const bool targetMatches = (rxPacket.target_id == NAVIGATOR_ID || rxPacket.target_id == BROADCAST_ID);
        const bool validCrc = check_crc(&rxPacket);
        if (targetMatches && validCrc)
        {
            *packet = rxPacket;
            rxState = NAV_SYNC;
            return true;
        }
        rxState = NAV_SYNC;
    }

    return false;
}

static uint16_t clamp_to_u16_scaled(float value, float scale)
{
    long scaled = lroundf(value * scale);
    if (scaled < 0)
    {
        scaled = 0;
    }
    if (scaled > 65535)
    {
        scaled = 65535;
    }
    return (uint16_t)scaled;
}

static void send_status_ack(uint8_t targetId, float altitudeMeters, float verticalVelocityMetersPerSecond, float verticalAccelerationMetersPerSecond2)
{
    NavigatorData navigatorData = {};
    navigatorData.altitude = clamp_to_u16_scaled(altitudeMeters, 1.0f);
    navigatorData.velocity = clamp_to_u16_scaled(fabsf(verticalVelocityMetersPerSecond), 1.0f);
    navigatorData.acceleration = clamp_to_u16_scaled(fabsf(verticalAccelerationMetersPerSecond2), 1.0f);
    navigatorData.gps_latitude = 0;
    navigatorData.gps_longitude = 0;

    uint8_t payload[1 + sizeof(NavigatorData)] = {0};
    payload[0] = NAV_CMD_STATUS;
    memcpy(payload + 1, &navigatorData, sizeof(NavigatorData));

    nav_packet_t ackPacket;
    if (create_packet(&ackPacket, NAVIGATOR_ID, targetId, NAV_CMD_ACK, payload, (uint8_t)sizeof(payload)) == 0)
    {
        write_packet(&ackPacket);
    }
}

static void send_nack(uint8_t targetId, uint8_t cmd)
{
    uint8_t payload[1] = {cmd};
    nav_packet_t nackPacket;
    if (create_packet(&nackPacket, NAVIGATOR_ID, targetId, NAV_CMD_NACK, payload, (uint8_t)sizeof(payload)) == 0)
    {
        write_packet(&nackPacket);
    }
}

void InitializeSensorUART() {
    uart_init(sensorUart, 115200);
    gpio_set_function(READ_OBC_PIN, GPIO_FUNC_UART);
    gpio_set_function(WRITE_OBC_PIN, GPIO_FUNC_UART);
    gpio_pull_up(READ_OBC_PIN);
}

void PollAndHandleComms(float altitudeMeters, float verticalVelocityMetersPerSecond, float verticalAccelerationMetersPerSecond2)
{
    nav_packet_t incoming;
    while (read_packet(&incoming))
    {
        if (incoming.sender_id != CORTEX_ID)
        {
            continue;
        }

        switch (incoming.cmd)
        {
        case NAV_CMD_STATUS:
            send_status_ack(incoming.sender_id, altitudeMeters, verticalVelocityMetersPerSecond, verticalAccelerationMetersPerSecond2);
            break;
        default:
            send_nack(incoming.sender_id, incoming.cmd);
            break;
        }
    }
}