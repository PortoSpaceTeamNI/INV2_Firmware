#include <Arduino.h>
#include <ArduinoEigen.h>
#include "Comms.h"
#include "Sensors.h"
#include "Display.h"
#include "Sensors/buzzer.h"
#include "runkalman.h"
#include "func.h"
#include "quaternion.h"

#define POLL_INTERVAL_MS 10

extern SensorDataResult sensorData;
unsigned long last_poll_time = 0;

void setup()
{
  Serial.begin(115200); // Start serial communication
  while (!Serial)
  {
    ; // Wait for serial port to connect. Needed for native USB
  }
  Serial.println("Navigator Initiating...");
  setup_buzzer();
  Serial.println("Starting Setup...");

  Wire.setSDA(SDA0_PIN);
  Wire.setSCL(SCL0_PIN);
  Wire.begin();
  Serial.println("I2C0 initialized");

  Wire1.setSDA(SDA1_PIN);
  Wire1.setSCL(SCL1_PIN);
  Wire1.begin();
  Serial.println("I2C1 initialized");

  SPI1.setSCK(SPI1_SCK_PIN);
  SPI1.setTX(SPI1_MOSI_PIN);
  SPI1.setRX(SPI1_MISO_PIN);
  SPI1.begin();
  Serial.println("SPI1 initialized");

  if (InitializeSensors() == 0)
  {
    Serial.println("Sensors initialized.");
  }
  else
    Serial.println("One or more sensors failed.");

  // while (1);

  if (ConfigureSensors() == 0)
  {
    Serial.println("Sensors Configured.");
  }
  else
    Serial.println("One or more sensors failed to configure.");

  play_buzzer_success();

  // while(1);
}

void loop()
{
  if ((millis() - last_poll_time) >= POLL_INTERVAL_MS)
  {
    if (ReadSensors() != 0)
      Serial.println("Failed to read data.");
    else
      DisplayData(&sensorData);
    last_poll_time = millis();
  }
}
