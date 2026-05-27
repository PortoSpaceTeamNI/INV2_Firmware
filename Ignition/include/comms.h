#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>

#define PACKET_SYNC 0x55
#define PACKET_MISSION_CONTROL_ID 0
#define PACKET_IGNITION_ID 9
#define PACKET_FIRE_COMMAND 5
#define PACKET_FIRE_COMMAND_PAYLOAD_SIZE 0

#define IGNITION_PACKET_MAX_PAYLOAD_SIZE 1

typedef struct __attribute__((packed)) {
    uint8_t sync;
    uint8_t sender;
    uint8_t target;
    uint8_t command;
    uint8_t payload_size;
    uint8_t payload[IGNITION_PACKET_MAX_PAYLOAD_SIZE];
    uint16_t crc;
} IGNITION_packet_t;

typedef enum {
    IGNITION_PACKET_SYNC_e,
    IGNITION_PACKET_SENDER_e,
    IGNITION_PACKET_TARGET_e,
    IGNITION_PACKET_COMMAND_e,
    IGNITION_PACKET_PAYLOAD_SIZE_e,
    IGNITION_PACKET_PAYLOAD_e,
    IGNITION_PACKET_CRC_e,
} IGNITION_packet_field_t;

#endif