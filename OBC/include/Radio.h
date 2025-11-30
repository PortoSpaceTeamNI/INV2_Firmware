#ifndef RADIO_H_
#define RADIO_H_

#include "HardwareCfg.h"
#include <Arduino.h>

// Radio configuration constants
// Move all configurable values here to avoid hardcoding inside implementation

// Common LoRa radio parameters
#define RADIO_STANDBY_MODE          0x00  // STDBY_RC
#define RADIO_PACKET_TYPE           0x01  // LoRa
// 2.4GHz frequency word (Frf = (Fxosc/2^18) * rfFrequency)
#define RADIO_RFFREQ_23_16          0xB8
#define RADIO_RFFREQ_15_8           0x9D
#define RADIO_RFFREQ_7_0            0x89

// LoRa modulation and packet settings
#define RADIO_SPREADING_FACTOR      0x70  // SF7
#define RADIO_BANDWIDTH             0x0A
#define RADIO_CODING_RATE           0x01
#define RADIO_PREAMBLE_LENGTH       0x0C
#define RADIO_HEADER_TYPE           0x00  // explicit header
#define RADIO_CRC                   0x20  // CRC ON
#define RADIO_CHIRP_INVERT          0x40

// RX settings
#define RADIO_RX_IRQ_15_8           0x40
#define RADIO_RX_IRQ_7_0            0x7E
#define RADIO_RX_PERIOD_BASE        0x02  // 1 ms
#define RADIO_RX_COUNT_15_8         0xFF
#define RADIO_RX_COUNT_7_0          0xFF

// TX settings
#define RADIO_TX_POWER              0x1F
#define RADIO_TX_RAMP_TIME          0xE0
#define RADIO_TX_IRQ_15_8           0x40
#define RADIO_TX_IRQ_7_0            0x01
#define RADIO_TX_PERIOD_BASE        0x02  // 1 ms
#define RADIO_TX_COUNT_15_8         0x01
#define RADIO_TX_COUNT_7_0          0xF4

// Demo behavior
#define RADIO_LED_BLINK_MS          50
#define RADIO_DEFAULT_TX_MESSAGE    "hi"

// Initialize SPI and SX1280 transceiver
void radio_setup();

// One-shot demo task: attempts a short RX window then transmits default message.
// Safe to call periodically from loop() if you want repeated behavior.
void radio_task();

#endif // RADIO_H_