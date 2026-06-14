#include <SPI.h>
#include <LoRa.h>

int counter = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Ino Sender");

  LoRa.setPins(10, 9, 8);
  //LoRa.setSignalBandwidth(500E3);
  //LoRa.setCodingRate4(5);
  //LoRa.setSpreadingFactor(7);
  //LoRa.setGain(1);

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  uint8_t buff[100] = {0};
  int size = 0;
  while(Serial.available())
  {
    buff[size++] = Serial.read();
    Serial.print("got serial ");
    Serial.println(buff[size - 1]);
  }

  if(size > 0)
  {
    //Serial.println("Send LoRa");
    LoRa.beginPacket();
    LoRa.write(buff, size);
    LoRa.endPacket(true);
  }

  // read packet
  int packetSize = LoRa.parsePacket();
  while (packetSize != 0 && LoRa.available()) {
    Serial.write((char)LoRa.read());
  }

}