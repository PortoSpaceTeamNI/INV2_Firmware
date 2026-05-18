#ifndef CONFIGS_H
#define CONFIGS_H

#include <Communications.h>

#define DEVICE_ID CORTEX_ID

// RS485 UART configuration
#define RS485_BAUD_RATE 115200

// RS485 timeout configuration (ms)
#define RS485_RESPONSE_TIMEOUT_MS 8

// RS485 polling interval (ms)
#define STATUS_POLL_INTERVAL_MS 20

// RS485 transceiver direction pin levels
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

#endif // CONFIGS_H