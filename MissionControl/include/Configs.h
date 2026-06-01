#ifndef CONFIGS_H
#define CONFIGS_H

#include <Communications.h>

#define DEVICE_ID MISSION_CONTROL_ID

// RS485 Configs
#define RS_ENABLED false
#define RS485_BAUD_RATE 115200
#define RS485_RESPONSE_TIMEOUT_MS 10
#define STATUS_POLL_INTERVAL_MS 20
#define RS485_TX_ENABLE_LEVEL HIGH
#define RS485_RX_ENABLE_LEVEL LOW

// RS485 line validation mode
// When enabled, normal RS485 queue traffic is bypassed and a fixed test frame is sent periodically.
#define RS485_TEST_PATTERN_ENABLED false
#define RS485_TEST_PATTERN_INTERVAL_MS 200
#define RS485_TEST_PATTERN_BYTE0 0x55
#define RS485_TEST_PATTERN_BYTE1 0xAA
#define RS485_TEST_CONTINUOUS_TX true

// RS485 debug logging
#define RS485_DEBUG_LOG false

// Radio Configs
#define RADIO_RESPONSE_TIMEOUT_MS 500

// LoRa Parameters (SHORT range profile)
#define RADIO_FREQUENCY   2450.0
#define RADIO_BANDWIDTH   1625.0
#define RADIO_SPREAD_FACTOR 7
#define RADIO_CODING_RATE 5
#define RADIO_SYNC_WORD   0x12
#define RADIO_TX_POWER    0           // 0 dBm into chip → PA raises to ~27 dBm output
#define RADIO_PREAMBLE_LEN 12

#endif // CONFIGS_H