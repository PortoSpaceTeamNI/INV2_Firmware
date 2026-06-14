#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
}

void loop() {
  while(Serial.available()) Serial2.write(Serial.read());
  while(Serial2.available()) Serial.write(Serial2.read());
}
