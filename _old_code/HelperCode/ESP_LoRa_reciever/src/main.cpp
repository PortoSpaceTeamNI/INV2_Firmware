#include <arduino.h>
#include <LoRa.h>

#include "HardwareCfg.h"

#include "Wire.h"

void setup() {
  Serial.begin(115200);

  while (!Serial);
  
  Wire.begin();

  pinMode(LORA_SS_PIN, OUTPUT);
  
  LoRa.setPins(LORA_SS_PIN, LORA_RESET_PIN, LORA_DIO0_PIN);
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  
  // read packet
  int packetSize = LoRa.parsePacket();
  if( packetSize > 0) Serial.printf("got message %d\n", packetSize);
  while (packetSize != 0 && LoRa.available()) {
    Serial.write((char)LoRa.read());
  }

}