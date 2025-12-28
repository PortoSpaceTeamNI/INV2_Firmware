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
#include "Peripherals/Buzzer.h"
#include "bmp5_defs.h"
#include "bmp5.h"
#include "Peripherals/bmp581.h"

void setup() {
  Serial.begin(9600); // Start serial communication

  setup_buzzer();
  Serial.println("Starting Setup...");

  if (bmp_setup() == 0) {
    Serial.println("Setup complete!");
    play_buzzer_success();
  } else {
    Serial.println("Setup failed!");
    play_buzzer_error();
  }
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
