#include <Arduino.h>
#include "Sensors.h"

extern SensorDataResult sensorData;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10); // wait for serial port to connect. For debug only.
  Serial.println("Hello, I'm PocketSat!");

  Wire.setSDA(I2C0_SDA_PIN);
  Wire.setSCL(I2C0_SCL_PIN);
  Wire.begin();
  Serial.println("I2C0 initialized");

  Wire1.setSDA(I2C1_SDA_PIN);
  Wire1.setSCL(I2C1_SCL_PIN);
  Wire1.begin();
  Serial.println("I2C1 initialized");

  SPI1.setMISO(SPI1_MISO_PIN);
  SPI1.setMOSI(SPI1_MOSI_PIN);
  SPI1.setSCK(SPI1_SCK_PIN);
  SPI1.begin();

  if (InitializeSensors() == 0)
  {
    Serial.println("All sensors initialized successfully.");
  }
  else
  {
    Serial.println("One or more sensors failed to initialize.");
    while (1)
      ; // Stop execution if sensors failed to initialize
  }

  if (ConfigureSensors() == 0)
  {
    Serial.println("All sensors configured successfully.");
  }
  else
  {
    Serial.println("One or more sensors failed to configure.");
  }
}

void loop()
{
  if (ReadSensors() != 0)
  {
    Serial.println("Failed to read sensors data!");
  }
  else
  {
    Serial.printf("HDC: %.2f ºC, %.2f %%\n", sensorData.hdcData.Temperature, sensorData.hdcData.Humidity);
    Serial.printf("BMP: %.2f ºC, %.2f hPa, %.2f m\n", sensorData.bmpData.Temperature, sensorData.bmpData.Pressure, sensorData.bmpData.Altitude);
    Serial.printf("LPS: %.2f ºC, %.2f hPa, %.2f m\n", sensorData.lpsData.Temperature, sensorData.lpsData.Pressure, sensorData.lpsData.Altitude);
    Serial.printf("BME: %.2f ºC, %.2f hPa, %.2f %%\n", sensorData.bmeData.Temperature, sensorData.bmeData.Pressure, sensorData.bmeData.Humidity);
  }
}
