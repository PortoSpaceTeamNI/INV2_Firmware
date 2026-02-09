#include <Arduino.h>
#include "Sensors.h"
#include "Display.h"
#include "Sensors/buzzer.h"

extern SensorDataResult sensorData;

void setup() {
  Serial.begin(115200); // Start serial communication
  Serial.println("Navigator Initiating...");
  setup_buzzer();
  Serial.println("Starting Setup...");

  Wire.setSDA(I2C_SDA_PIN0);
  Wire.setSCL(I2C_SCL_PIN0);
  Wire.begin();
  Serial.println("I2C0 initialized");

  Wire1.setSDA(I2C_SDA_PIN1);
  Wire1.setSCL(I2C_SCL_PIN1);
  Wire1.begin();
  Serial.println("I2C1 initialized");

  if (InitializeSensors() == 0) {
    Serial.println("Sensors initialized.");
  } else Serial.println("One or more sensors failed.");

  if (ConfigureSensors() == 0) {
    Serial.println("Sensors Configured.");
  } else Serial.println("One or more sensors failed to configure.");

}

void loop() {
  if (ReadSensors() != 0) Serial.println("Failed to read data.");
  else DisplayData(&sensorData);

  delay(1000);
}
