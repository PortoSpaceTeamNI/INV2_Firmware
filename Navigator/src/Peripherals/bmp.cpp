// src >> Peripherals >> bmp.cpp

#include "Peripherals/bmp581.h"
#include "Peripherals/IO_Map.h"
#include "DataModels.h"
#include <Wire.h>

struct bmp5_dev bmp581_dev;

int bmp_setup(void) {

    pinMode(BAR2_RDY, INPUT); 
    Wire1.setSDA(I2C_SDA_PIN0);  // Set SDA pin
    Wire1.setSCL(I2C_SCL_PIN0);  // Set SCL pin
    Wire1.begin();  // Start the I2C communication
    
    int8_t status = bmp5_init(&bmp581_dev);
    if (status != 0) {
        Serial.println("Failed to initialize BMP581 sensor.");
        return -1;  
    }

    status = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &bmp581_dev);  // Set sensor to normal mode
    if (status != 0) {
        Serial.println("Failed to set BMP581 power mode.");
        return -1;
    }

    Serial.println("BMP581 initialized successfully.");
    return 0; 
}

int read_baro2(data_t *data) {
    bmp5_sensor_data sensor_data;
    bmp5_osr_odr_press_config osr_odr_press_cfg;
    osr_odr_press_cfg.odr = 4;  // Output Data Rate
    osr_odr_press_cfg.osr_p = 4;  // Oversampling Rate for Pressure
    osr_odr_press_cfg.press_en = BMP5_ENABLE;  // Enable pressure measurement

 
    if (digitalRead(BAR2_RDY) == LOW) {
        int8_t status = bmp5_get_sensor_data(&sensor_data, &osr_odr_press_cfg, &bmp581_dev);
        if (status != 0) {
            Serial.println("Error reading data from BMP581.");
            return -1;
        }

        data->pressure1 = (int16_t)(sensor_data.pressure / 100);  // Convert pressure from Pa to hPa
        data->temperature1 = (int16_t)(sensor_data.temperature);  // Temperature in °C, multiply by 100 for centi-degrees

        Serial.print("Pressure: ");
        Serial.print(sensor_data.pressure);
        Serial.print(" Pa, Temperature: ");
        Serial.println(sensor_data.temperature);

        return 0;  // Successful read
    }
    return -1;  // No data ready
}


// interrupts??