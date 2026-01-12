#include <Arduino.h>
#include <Crc.h>
#include <Wire.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

#include "Comms.h"
#include "DataModels.h"
#include "HardwareCfg.h"
#include "Peripherals/IO_Map.h"
#include "bmp5_defs.h"
#include "bmp5.h"
#include "Peripherals/bmp581.h"
#include "Peripherals/lps.h"
#include "Peripherals/hdc.h"
#include "Peripherals/bme.h"
#include <SPI.h>

//int cs_pin = HUM_CS_PIN;
/*
void setup() {
    Serial.begin(9600);
    while (!Serial) {}
    delay(500);

    SPI.begin();
    delay(100);

    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    SPI.transfer(0xD0 | 0x80);
    uint8_t id = SPI.transfer(0x00);
    SPI.endTransaction();

    Serial.print("Chip ID: 0x");
    Serial.println(id, HEX);
}

void loop() {}*/



//static uint8_t bme_addr = BME280_I2C_ADDR_PRIM;

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);  // Wait for USB Serial

  // Initialize I2C buses once
  Wire.setSDA(I2C_SDA_PIN0);
  Wire.setSCL(I2C_SCL_PIN0);
  Wire.begin();
  Wire1.setSDA(I2C_SDA_PIN1);
  Wire1.setSCL(I2C_SCL_PIN1);
  Wire1.begin();

  delay(200);
  Serial.println("Starting Setup...");

  // Use the bmp_setup() function from bmp.cpp
  if (bmp_setup() == 0) {
    Serial.println("BMP581 initialized!");
  } else {
    Serial.println("BMP581 init failed!");
  }

  // Initialize LPS22DF
  if (lps_setup() == 0)
    Serial.println("LPS22DF initialized");
  else
    Serial.println("LPS22DF init failed");

  // Initialize HDC302x
  if (hdc_setup() == 0)
    Serial.println("HDC302x initialized");
  else
    Serial.println("HDC302x init failed");

  Serial.println("After");

  // Initialize BME280
  if (bme_setup() == 0)
    Serial.println("BME280 initialized");
  else
    Serial.println("BME280 init failed");
}

void loop() {

  data_t data;

  // Read BMP581
  if (read_baro2(&data) == 0) {
      Serial.print("BMP581 -> Pressure: ");
      Serial.print(data.pressure_bmp, 2);
      Serial.print(" hPa, Temperature: ");
      Serial.println(data.temperature_bmp, 2);
  } else {
      Serial.println("BMP581 data not ready or error.");
  }

  // Read LPS22DF
  if (read_baro3(&data) == 0) {
      Serial.print("LPS22DF -> Pressure: ");
      Serial.print(data.pressure_lps, 2);
      Serial.print(" hPa, Temperature: ");
      Serial.println(data.temperature_lps, 2);
  } else {
      Serial.println("LPS22DF data not ready or error.");
  }

  // Read HDC3020
  if (read_hdc(&data) == 0) {
      Serial.print("HDC3020 -> Humidity: ");
      Serial.print(data.humidity_hdc, 2);
      Serial.print(" %, Temperature: ");
      Serial.println(data.temperature_hdc, 2);
  } else {
      Serial.println("HDC3020 data not ready or error.");
  }

  // Read BME280
  if (read_bme(&data) == 0) {
    Serial.print("BME280 -> T: ");
    Serial.print(data.temperature_bme, 2);
    Serial.print(" °C, P: ");
    Serial.print(data.pressure_bme, 2);
    Serial.print(" hPa, H: ");
    Serial.print(data.humidity_bme, 2);
    Serial.println(" %");
}
  else {
    Serial.println("BME280 data not ready or error.");
  }

  delay(1000);  // 1 second delay between readings
}


/*
void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize Wire1 on I2C1 pins
  Wire1.setSDA(I2C_SDA_PIN1);
  Wire1.setSCL(I2C_SCL_PIN1);
  Wire1.begin();

  Serial.println("\nI2C1 Scanner");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("Done\n");

  delay(3000);
}*/

