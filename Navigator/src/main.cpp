#include <Arduino.h>
#include <Crc.h>
#include <Wire.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "Comms.h"
#include "DataModels.h"
#include "HardwareCfg.h"
#include "Peripherals/IO_Map.h"
#include "bmp5_defs.h"
#include "bmp5.c"
#include "bmp5.h"
#include "Peripherals/bmp581.h"

// BMP581 I2C Address
#define BMP581_I2C_ADDR 0x76  // 0x76 LOW ; 0x77 HIGH

void setup() {
  Serial.begin(115200); // Start serial communication

  // Initialize I2C
  Wire1.setSDA(I2C_SDA_PIN0);  // Set SDA pin
  Wire1.setSCL(I2C_SCL_PIN0);  // Set SCL pin
  Wire1.begin();  // Start the I2C communication

  // Initialize BMP581
  int8_t status = bmp5_init(&bmp581_dev);
  if (status != 0) {
    Serial.println("Failed to initialize BMP581 sensor.");
    return;
  }
  Serial.println("BMP581 initialized successfully.");

  // Set BMP581 to normal power mode
  status = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &bmp581_dev);
  if (status != 0) {
    Serial.println("Failed to set BMP581 power mode.");
    return;
  }

  // Set up other pins as needed (if necessary for your application)
}

    void loop() {
    // Read sensor data
    bmp5_sensor_data sensor_data;
    bmp5_osr_odr_press_config osr_odr_press_cfg;
    osr_odr_press_cfg.press_en = BMP5_ENABLE;  // Pressure enable
    osr_odr_press_cfg.osr_p = 4;               // Oversampling rate for pressure
    osr_odr_press_cfg.odr = 4;                 // Output Data Rate


  if (digitalRead(BAR2_RDY) == LOW) {  // Check if data is ready
    int8_t status = bmp5_get_sensor_data(&sensor_data, &osr_odr_press_cfg, &bmp581_dev);
    if (status == 0) {
      // Print the pressure and temperature data
      Serial.print("Pressure: ");
      Serial.print(sensor_data.pressure / 100);  // Pressure in hPa (divide by 100 to convert from Pa)
      Serial.print(" hPa, Temperature: ");
      Serial.println(sensor_data.temperature);  // Temperature in °C
    } else {
      Serial.println("Error reading data from BMP581.");
    }
  } else {
    Serial.println("Sensor data not ready yet...");
  }

  delay(1000);  // Delay to allow next reading (1 second)
}
