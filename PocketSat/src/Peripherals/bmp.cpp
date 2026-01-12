#include "Peripherals/bmp581.h"
#include "Peripherals/IO_Map.h"
#include "DataModels.h"
#include <Wire.h>

struct bmp5_dev bmp581_dev;

BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) {
        return -1;
    }
    
    Wire.requestFrom(dev_addr, (uint8_t)len);
    for (uint32_t i = 0; i < len; i++) {
        reg_data[i] = Wire.read();
    }
    
    return 0;
}

// I2C write callback for BMP5 library
BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.write(reg_data, len);
    
    return (Wire.endTransmission() == 0) ? 0 : -1;
}

// Delay callback for BMP5 library
void bmp5_delay_us(uint32_t period, void *intf_ptr) {
    delayMicroseconds(period);
}

int bmp_setup(void) {
    pinMode(BAR2_RDY, INPUT);
    
    Wire.setSDA(I2C_SDA_PIN0);
    Wire.setSCL(I2C_SCL_PIN0);
    Wire.begin();
    delay(100);
    
    // Configure BMP581 device structure BEFORE calling bmp5_init
    bmp581_dev.chip_id = BMP5_CHIP_ID_PRIM;
    bmp581_dev.intf = BMP5_I2C_INTF;
    bmp581_dev.read = bmp5_i2c_read;
    bmp581_dev.write = bmp5_i2c_write;
    bmp581_dev.delay_us = bmp5_delay_us;
    
    static uint8_t dev_addr = BMP581_I2C_ADDR;
    bmp581_dev.intf_ptr = &dev_addr;
    
    Serial.println("Calling bmp5_init...");
    int8_t status = bmp5_init(&bmp581_dev);
    Serial.print("Init status: ");
    Serial.println(status);
    
    if (status != 0 && status != -5) {
        Serial.println("Failed to initiate BMP5");
        return -1;
    }

    if (status == -5) {
        Serial.println("Warning: Soft reset failed, but continuing...");
    }
    
    status = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &bmp581_dev);
    if (status != 0) {
        Serial.println("Failed to set power mode");
        return -1;
    }
    
    Serial.println("BMP581 initialized!");
    return 0;
}

int read_baro2(data_t *data) {
    bmp5_sensor_data sensor_data;
    bmp5_osr_odr_press_config osr_odr_press_cfg;
    osr_odr_press_cfg.odr = 4;  // Output Data Rate 50 Hz
    osr_odr_press_cfg.osr_p = 4;  // Oversampling Rate for Pressure
    osr_odr_press_cfg.press_en = BMP5_ENABLE;  // Enable pressure measurement

 
    if (digitalRead(BAR2_RDY) == LOW) {
        int8_t status = bmp5_get_sensor_data(&sensor_data, &osr_odr_press_cfg, &bmp581_dev);
        if (status != 0) {
            Serial.println("Error reading data from BMP581.");
            return -1;
        }

        data->pressure_bmp = (sensor_data.pressure / 100);  // Convert pressure from Pa to hPa
        data->temperature_bmp = sensor_data.temperature;  // Temperature in °C, multiply by 100 for centi-degrees

        Serial.print("BMP581 -> Pressure: ");
        Serial.print(data->pressure_bmp);
        Serial.print(" hPa, Temperature: ");
        Serial.println(data->temperature_bmp);

        return 0;  // Successful read
    }
    return -1;  // No data ready
}