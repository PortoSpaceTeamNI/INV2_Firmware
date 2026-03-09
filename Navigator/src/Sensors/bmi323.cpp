#include "Sensors/bmi323.h"
#include "Sensors.h"
#include "Pinout.h"
#include <Arduino.h>
#include <SPI.h>

extern "C" {
    #include "../../lib/BMI/bmi323.h"
    #include "../../lib/BMI/bmi3.h"
}

struct bmi3_dev bmi323_dev;
bool bmi323_initialized = false;
SPISettings bmi323_spi_settings(10000000, MSBFIRST, SPI_MODE0);

BMI3_INTF_RET_TYPE bmi323_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t cs_pin = *(uint8_t*)intf_ptr;
    
    SPI1.beginTransaction(bmi323_spi_settings);
    digitalWrite(cs_pin, LOW);
    delayMicroseconds(1); // Small delay for CS settling
    
    SPI1.transfer(reg_addr | 0x80); // Set MSB for read
    
    for (uint32_t i = 0; i < length; i++) {
        reg_data[i] = SPI1.transfer(0x00);
    }
    
    digitalWrite(cs_pin, HIGH);
    SPI1.endTransaction();
    
    return 0;
}

BMI3_INTF_RET_TYPE bmi323_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t cs_pin = *(uint8_t*)intf_ptr;
    
    SPI1.beginTransaction(bmi323_spi_settings);
    digitalWrite(cs_pin, LOW);
    delayMicroseconds(1); // Small delay for CS settling
    
    SPI1.transfer(reg_addr & 0x7F); // Clear MSB for write
    
    for (uint32_t i = 0; i < length; i++) {
        SPI1.transfer(reg_data[i]);
    }
    
    digitalWrite(cs_pin, HIGH);
    SPI1.endTransaction();
    
    return 0;
}

void bmi323_delay_us(uint32_t period, void *intf_ptr)
{
    delayMicroseconds(period);
}

int InitializeBMI323()
{
    static uint8_t cs_pin = IMU1_CS_PIN;
    
    pinMode(IMU1_CS_PIN, OUTPUT);
    digitalWrite(IMU1_CS_PIN, HIGH);
    
    // Wait for sensor power-up
    delay(10);
    
    // Initialize device structure
    bmi323_dev.chip_id = 0;
    bmi323_dev.intf = BMI3_SPI_INTF;
    bmi323_dev.read = bmi323_spi_read;
    bmi323_dev.write = bmi323_spi_write;
    bmi323_dev.delay_us = bmi323_delay_us;
    bmi323_dev.intf_ptr = &cs_pin;
    bmi323_dev.read_write_len = 32; // Maximum read/write length
    bmi323_dev.dummy_byte = 1; // SPI requires dummy byte
    
    Serial.println("BMI323: Starting init...");
    Serial.printf("BMI323: CS=%d, SPI1(MISO=%d,MOSI=%d,SCK=%d)\n", 
                  IMU1_CS_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN);
    
    // Initialize sensor
    int8_t rslt = bmi323_init(&bmi323_dev);
    if (rslt != 0) {
        Serial.printf("BMI323 init failed: %d (chip_id=0x%02X)\n", rslt, bmi323_dev.chip_id);
        return -1;
    }
    
    Serial.printf("BMI323: Chip ID = 0x%02X (expected 0x43)\n", bmi323_dev.chip_id);
    
    bmi323_initialized = true;
    return 0;
}

int ConfigureBMI323()
{
    if (!bmi323_initialized) {
        return -1;
    }
    
    int8_t rslt;
    
    // Configure accelerometer: 6.4kHz ODR, ±16g range
    struct bmi3_sens_config sens_cfg[2];
    
    // Accelerometer config
    sens_cfg[0].type = BMI323_ACCEL;
    sens_cfg[0].cfg.acc.odr = BMI3_ACC_ODR_6400HZ;
    sens_cfg[0].cfg.acc.range = BMI3_ACC_RANGE_16G;
    sens_cfg[0].cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;
    sens_cfg[0].cfg.acc.avg_num = BMI3_ACC_AVG1;
    sens_cfg[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;
    
    // Gyroscope config: 6.4kHz ODR, ±2000dps range
    sens_cfg[1].type = BMI323_GYRO;
    sens_cfg[1].cfg.gyr.odr = BMI3_GYR_ODR_6400HZ;
    sens_cfg[1].cfg.gyr.range = BMI3_GYR_RANGE_2000DPS;
    sens_cfg[1].cfg.gyr.bwp = BMI3_GYR_BW_ODR_QUARTER;
    sens_cfg[1].cfg.gyr.avg_num = BMI3_GYR_AVG1;
    sens_cfg[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;
    
    rslt = bmi323_set_sensor_config(sens_cfg, 2, &bmi323_dev);
    if (rslt != 0) {
        Serial.printf("BMI323 config failed: %d\n", rslt);
        return -1;
    }
    
    delay(50); 
    
    return 0;
}

bool IsBMI323Ready()
{
    return bmi323_initialized;
}

int ReadBMI323(BMI323DataResult &result)
{
    if (!bmi323_initialized) {
        return -1;
    }
    
    struct bmi3_sensor_data sensor_data[3];
    
    // Request accelerometer data
    sensor_data[0].type = BMI323_ACCEL;
    
    // Request gyroscope data
    sensor_data[1].type = BMI323_GYRO;
    
    // Request temperature data
    sensor_data[2].type = BMI3_TEMP;
    
    int8_t rslt = bmi323_get_sensor_data(sensor_data, 3, &bmi323_dev);
    if (rslt != 0) {
        Serial.printf("BMI323 read failed: %d\n", rslt);
        return -1;
    }
    
    // Convert accelerometer data (LSB to m/s^2)
    // For ±16g range: 1 LSB = 16g / 32768 = 0.00048828125 g
    // 1 g = 9.80665 m/s^2
    float acc_lsb_to_ms2 = (16.0f / 32768.0f) * 9.80665f;
    result.AccelX = sensor_data[0].sens_data.acc.x * acc_lsb_to_ms2;
    result.AccelY = sensor_data[0].sens_data.acc.y * acc_lsb_to_ms2;
    result.AccelZ = sensor_data[0].sens_data.acc.z * acc_lsb_to_ms2;
    
    // Convert gyroscope data (LSB to rad/s)
    // For ±2000dps range: 1 LSB = 2000 / 32768 = 0.06103515625 dps
    // Convert to rad/s: degrees * (PI / 180)
    float gyr_lsb_to_rads = (2000.0f / 32768.0f) * (PI / 180.0f);
    result.GyroX = sensor_data[1].sens_data.gyr.x * gyr_lsb_to_rads;
    result.GyroY = sensor_data[1].sens_data.gyr.y * gyr_lsb_to_rads;
    result.GyroZ = sensor_data[1].sens_data.gyr.z * gyr_lsb_to_rads;
    
    // Convert temperature data
    // Temperature in °C = (temp_data / 512) + 23
    result.Temperature = (sensor_data[2].sens_data.temp.temp_data / 512.0f) + 23.0f;
    
    return 0;
}
