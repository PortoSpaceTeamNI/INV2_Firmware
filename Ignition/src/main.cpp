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

packet_t pkt;
packet_field_t current_field;
size_t current_field_size;

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

void goToField(packet_field_t field) {
  current_field = field;
  current_field_size = 0;
}

bool parse_packet(byte* data, size_t len) {
  bool is_packet_complete = false;

  for (size_t i = 0; i < len; ++i) {
    ++current_field_size;

    switch (current_field) {
      case PACKET_SYNC: {
        if (data[i] != PACKET_SYNC) {
          SERIAL.printf("Non Sync byte ignored: 0x%X", data[i]);
          break;
        }

        pkt.sync = data[i];
        goToField(PACKET_SENDER);
        break;
      }
      case PACKET_SENDER: {
        if (data[i] != MISSION_CONTROL_ID) {
          SERIAL.printf("Non Mission Control sender ignored: 0x%X", data[i]);
          goToField(PACKET_SYNC);
          break;
        }

        pkt.sender = data[i];
        goToField(PACKET_TARGET);
        break;
      }
      case PACKET_TARGET: {
        if (data[i] != IGNITION_ID) {
          SERIAL.printf("Non Ignition target ignored: 0x%X", data[i]);
          goToField(PACKET_SYNC);
          break;
        }

        pkt.target = data[i];
        goToField(PACKET_COMMAND);
        break;
      }
      case PACKET_COMMAND: {
        if (data[i] != FIRE_COMMAND_ID) {
          SERIAL.printf("Non Fire command ignored: 0x%X", data[i]);
          goToField(PACKET_SYNC);
          break;
        }

        pkt.command = data[i];
        goToField(PACKET_PAYLOAD_SIZE);
        break;
      }
      case PACKET_PAYLOAD_SIZE: {
        if (data[i] != FIRE_COMMAND_PAYLOAD_SIZE) {
          SERIAL.printf("Non 0 payload size ignored: 0x%X", data[i]);
          goToField(PACKET_SYNC);
          break;
        }

        pkt.payload_size = data[i];
        goToField(PACKET_CRC);
        break;
      }
      case PACKET_CRC: {
        pkt.crc |= (current_field_size == 1 ? data[i] : (uint16_t)data[i] << 8);

        if (current_field_size == sizeof(uint16_t)) {
          goToField(PACKET_SYNC);
          is_packet_complete = true;
        }
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
  current_field = PACKET_SYNC;
  current_field_size = 0;
}

void loop() {
  PinStatus flag_ignite = digitalRead(PIN_IGNITE_LED);

  if (received_data) {
    received_data = false;

    byte data[sizeof(packet_t)];

    size_t len = radio.getPacketLength();
    uint16_t err = radio.readData(data, len);
    if (err != RADIOLIB_ERR_NONE) {
      Serial.printf("Failed radio data reading: Error code %d\n", err);
    
    } else {
      bool is_ignite_packet_complete = parse_packet(data, len);
      if (is_ignite_packet_complete) {
        byte ack[] = {
          SYNC,                      // Sync
          IGNITION_ID,               // Sender
          MISSION_CONTROL_ID,        // Target
          ACK_COMMAND_ID,            // Command
          ACK_COMMAND_PAYLOAD_SIZE,  // Payload Size
          FIRE_COMMAND_ID,           // Payload
          0, 0                              // TODO: CRC
        };

        err = radio.transmit(ack, sizeof(ack));
        if (err != RADIOLIB_ERR_NONE) {
          SERIAL.printf("Failed radio data sending: Error code %d\n", err);
        }

        if (flag_ignite) {
          digitalWrite(PIN_IGNITE_TRIGGER, HIGH);
        }
      }
    }

    radio.startReceive();
  }
}