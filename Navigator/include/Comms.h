#ifndef COMMS_H
#define COMMS_H

#include "Sensors.h"
#include <inttypes.h>

#define NAV_SYNC_BYTE 0x55
#define NAV_MAX_PAYLOAD_SIZE 200
#define NAV_MAX_PACKET_SIZE (1 + 4 + NAV_MAX_PAYLOAD_SIZE + 2)
#define NAV_PACKET_TIMEOUT_MS 50

typedef enum
{
	NAV_CMD_NONE = 0,
	NAV_CMD_STATUS,
	NAV_CMD_ABORT,
	NAV_CMD_STOP,
	NAV_CMD_ARM,
	NAV_CMD_FIRE,
	NAV_CMD_FILL_EXEC,
	NAV_CMD_MANUAL_EXEC,
	NAV_CMD_ACK,
	NAV_CMD_NACK,
} nav_command_t;

typedef enum
{
	NAV_SYNC = 0,
	NAV_SENDER_ID,
	NAV_TARGET_ID,
	NAV_CMD,
	NAV_PAYLOAD_SIZE,
	NAV_PAYLOAD,
	NAV_CRC1,
	NAV_CRC2,
	NAV_END,
} nav_parse_state_t;

typedef struct __attribute__((__packed__))
{
	uint16_t altitude;
	uint16_t velocity;
	uint16_t acceleration;
	uint32_t gps_latitude;
	uint32_t gps_longitude;
} NavigatorData;

typedef struct __attribute__((__packed__))
{
	uint8_t sender_id;
	uint8_t target_id;
	uint8_t cmd;
	uint8_t payload_size;
	uint8_t payload[NAV_MAX_PAYLOAD_SIZE];
	uint16_t crc;

	uint8_t data_recv;
	uint32_t begin_ms;
} nav_packet_t;

// Shared board IDs to match CORTEX definitions.
#define CORTEX_ID 1
#define NAVIGATOR_ID 5
#define BROADCAST_ID 0xFF

void InitializeSensorUART();
void PollAndHandleComms(float altitudeMeters, float verticalVelocityMetersPerSecond, float verticalAccelerationMetersPerSecond2);

#endif // COMMS_H