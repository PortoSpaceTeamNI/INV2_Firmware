#include <SPI.h>
#include <LoRa.h>

#include "HardwareCfg.h"
#include "Wire.h"

int counter = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial);

  Wire.begin();
  Serial.println("LoRa Sender");

  pinMode(LORA_SS_PIN, OUTPUT);
  LoRa.setPins(LORA_SS_PIN, LORA_RESET_PIN, LORA_DIO0_PIN);
  //LoRa.setSignalBandwidth(500E3);
  //LoRa.setCodingRate4(5);
  //LoRa.setSpreadingFactor(7);
  //LoRa.setGain(1);

  while (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    delay(100);
  }
}

void loop() {
  uint8_t buff[100] = {0};
  int size = 0;
  while(Serial.available())
  {
    buff[size++] = Serial.read();
  }

  //if(size > 0) Serial.printf("got message %d\n", size);
  //for(int i = 0; i < size; i++)
    //Serial.printf("0x%x\n", buff[i]);
  //if(size > 0)Serial.println("");

  if(size > 0)
  {
    //Serial.println("Got bytes");
    LoRa.beginPacket();
    int sz = LoRa.write(buff, size);
    LoRa.endPacket(true);
    //Serial.printf("sent %d byts\n", sz);
  }

  // read packet
  int packetSize = LoRa.parsePacket();
  //if(packetSize > 0) Serial.println("got response");
  while (packetSize != 0 && LoRa.available()) {
    Serial.write((char)LoRa.read());
  }

}