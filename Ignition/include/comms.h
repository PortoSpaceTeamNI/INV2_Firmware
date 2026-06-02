#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>

#define SYNC 0x55
#define MISSION_CONTROL_ID 0
#define IGNITION_ID 9
#define FIRE_COMMAND_ID 5
#define ACK_COMMAND_ID 8
#define FIRE_COMMAND_PAYLOAD_SIZE 0
#define ACK_COMMAND_PAYLOAD_SIZE 1
#define PACKET_MAX_PAYLOAD_SIZE 1

typedef struct __attribute__((packed)) {
    uint8_t sync;
    uint8_t sender;
    uint8_t target;
    uint8_t command;
    uint8_t payload_size;
    uint8_t payload[PACKET_MAX_PAYLOAD_SIZE];
    uint16_t crc;
} packet_t;

typedef enum {
    PACKET_SYNC,
    PACKET_SENDER,
    PACKET_TARGET,
    PACKET_COMMAND,
    PACKET_PAYLOAD_SIZE,
    PACKET_PAYLOAD,
    PACKET_CRC,
} packet_field_t;

#endif