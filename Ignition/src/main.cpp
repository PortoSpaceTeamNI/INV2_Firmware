#include <Arduino.h>
#include <RadioLib.h>

#include "comms.h"

#define SERIAL Serial

#define PIN_IGNITE_LED 9
#define PIN_IGNITE_TRIGGER 11

// RADIO Config
#define RADIO_PIN_NSS 17
#define RADIO_PIN_DIO1 20
#define RADIO_PIN_RESET 22
#define RADIO_PIN_BUSY 21

#define RADIO_FREQ_MHZ 2400

SX1280 radio;
volatile bool received_data;

IGNITION_packet_t pkt;
IGNITION_packet_field_t current_field;

uint16_t radio_init(SX1280& radio) {
  uint16_t err = RADIOLIB_ERR_NONE;

  radio = new Module(RADIO_PIN_NSS, RADIO_PIN_DIO1, RADIO_PIN_RESET, RADIO_PIN_BUSY);

  ConfigLoRa_t radio_cfg = { .frequency = RADIO_FREQ_MHZ };
  err = radio.begin(radio_cfg);
  if (err != RADIOLIB_ERR_NONE) {
    SERIAL.printf("Failed radio initialization: Error code %d\n", err);
    return err;
  }

  radio.setPacketReceivedAction([](){ received_data = true; });

  err = radio.startReceive();
  if (err != RADIOLIB_ERR_NONE) {
    SERIAL.printf("Failed radio receiving: Error code %d\n", err);
    return err;
  }

  return RADIOLIB_ERR_NONE;
}

bool parse_packet(byte* data, size_t len) {
  bool is_packet_complete = false;

  for (size_t i = 0; i < len; ++i) {
    switch (current_field) {
      case IGNITION_PACKET_SYNC_e: {
        if (data[i] != PACKET_SYNC) {
          SERIAL.printf("Non Sync byte ignored: 0x%X", data[i]);
          break;
        }

        pkt.sync = data[i];
        current_field = IGNITION_PACKET_SENDER_e;
        break;
      }
      case IGNITION_PACKET_SENDER_e: {
        if (data[i] != PACKET_MISSION_CONTROL_ID) {
          SERIAL.printf("Non Mission Control sender ignored: 0x%X", data[i]);
          current_field = IGNITION_PACKET_SYNC_e;
          break;
        }

        pkt.sender = data[i];
        current_field = IGNITION_PACKET_TARGET_e;
        break;
      }
      case IGNITION_PACKET_TARGET_e: {
        if (data[i] != PACKET_IGNITION_ID) {
          SERIAL.printf("Non Ignition target ignored: 0x%X", data[i]);
          current_field = IGNITION_PACKET_SYNC_e;
          break;
        }

        pkt.target = data[i];
        current_field = IGNITION_PACKET_COMMAND_e;
        break;
      }
      case IGNITION_PACKET_COMMAND_e: {
        if (data[i] != PACKET_FIRE_COMMAND) {
          SERIAL.printf("Non Fire command ignored: 0x%X", data[i]);
          current_field = IGNITION_PACKET_SYNC_e;
          break;
        }

        pkt.command = data[i];
        current_field = IGNITION_PACKET_PAYLOAD_SIZE_e;
        break;
      }
      case IGNITION_PACKET_PAYLOAD_SIZE_e: {
        if (data[i] != PACKET_FIRE_COMMAND_PAYLOAD_SIZE) {
          SERIAL.printf("Non 0 payload size ignored: 0x%X", data[i]);
          current_field = IGNITION_PACKET_SYNC_e;
          break;
        }

        pkt.payload_size = data[i];
        current_field = IGNITION_PACKET_CRC_e;
        break;
      }
      case IGNITION_PACKET_CRC_e: {
        // TODO: CRC check (the two bytes need to be checked)

        current_field = IGNITION_PACKET_SYNC_e;
        is_packet_complete = true;
        break;
      }
    }
  }

  return is_packet_complete;
}

void setup() {
  pinMode(PIN_IGNITE_LED, INPUT);

  uint16_t err = radio_init(radio);
  if (err != RADIOLIB_ERR_NONE) {
    while (true) { delay(100); }
  }

  received_data = false;
  pkt = {0};
  current_field = IGNITION_PACKET_SYNC_e;
}

void loop() {
  PinStatus flag_ignite = digitalRead(PIN_IGNITE_LED);

  if (received_data) {
    received_data = false;

    byte data[sizeof(IGNITION_packet_t)];

    size_t len = radio.getPacketLength();
    uint16_t err = radio.readData(data, len);
    if (err != RADIOLIB_ERR_NONE) {
      Serial.printf("Failed radio data reading: Error code %d\n", err);
    
    } else {
      bool is_ignite_packet_complete = parse_packet(data, len);
      if (flag_ignite && is_ignite_packet_complete) {
        digitalWrite(PIN_IGNITE_TRIGGER, HIGH);
      }
    }

    radio.startReceive();
  }
}