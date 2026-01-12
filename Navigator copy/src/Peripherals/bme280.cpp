/*#include "Peripherals/bme.h"
#include "Peripherals/IO_Map.h"
#include "DataModels.h"
#include <Wire.h>
#include <SPI.h>

struct bme280_dev bme280_dev;
uint8_t cs_pin = HUM_CS_PIN; 

int8_t bme280_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){

    uint8_t cs = *(uint8_t *)intf_ptr;

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(cs, LOW);

    SPI.transfer(reg_addr | 0x80); // READ command  

    for (uint32_t i = 0; i < len; i++) {
        reg_data[i] = SPI.transfer(0x00);
    }

    digitalWrite(cs, HIGH);
    SPI.endTransaction();
    return BME280_OK;
}

int8_t bme280_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){

    uint8_t cs = *(uint8_t *)intf_ptr;

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(cs, LOW);

    // MSB = 0 → WRITE
    SPI.transfer(reg_addr & 0x7F);
    for (uint32_t i = 0; i < len; i++) {
        SPI.transfer(reg_data[i]);
    }

    digitalWrite(cs, HIGH);
    SPI.endTransaction();
    return BME280_INTF_RET_SUCCESS;
}

void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    delayMicroseconds(period);
}


int bme_setup(void)
{   
    Serial.println("Starting BME280 setup...");

    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

    SPI.setRX(SPI_MISO_PIN);
    SPI.setTX(SPI_MOSI_PIN);
    SPI.setSCK(SPI_SCK_PIN);
    SPI.begin(); // just init SPI with pins already set

    Serial.println("SPI.begin done");

    uint8_t chip_id;
    int8_t id_rslt = bme280_spi_read(0xD0, &chip_id, 1, &cs_pin);
    Serial.print("Chip ID read result: "); Serial.println(id_rslt);
    Serial.print("Chip ID: 0x"); Serial.println(chip_id, HEX);


    bme280_dev.intf = BME280_SPI_INTF;
    bme280_dev.intf_ptr = &cs_pin;
    bme280_dev.read = bme280_spi_read;
    bme280_dev.write = bme280_spi_write;
    bme280_dev.delay_us = bme280_delay_us;

    Serial.println("Calling bme280_init...");
    int8_t status = bme280_init(&bme280_dev);
    Serial.print("Init status: ");
    Serial.println(status);

    if (status != BME280_OK) {
        Serial.println("Failed to initiate BME280");
        return -1;
    }

    Serial.println("BME280 initialized, setting config...");

    struct bme280_settings settings = {0};
    

    settings.osr_t = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.filter = BME280_FILTER_COEFF_OFF;
    settings.standby_time = BME280_STANDBY_TIME_1000_MS;

    bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &bme280_dev);
    bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme280_dev);

    Serial.println("BME280 initialized!");
    return 0;
}

int read_bme(data_t *data)
{
    struct bme280_data comp_data;

    int8_t rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme280_dev);
    if (rslt != BME280_OK) {
        Serial.println("BME280 read failed");
        return -1;
    }

#ifdef BME280_DOUBLE_ENABLE
    data->temperature_bme = comp_data.temperature;   // °C
    data->pressure_bme    = comp_data.pressure / 100.0; // Pa → hPa
    data->humidity_bme    = comp_data.humidity;      // %
#else
    data->temperature_bme = comp_data.temperature / 100.0;
    data->pressure_bme    = comp_data.pressure / 10000.0;
    data->humidity_bme    = comp_data.humidity / 1024.0;
#endif

    return 0;
} */


#include <Arduino.h>
#include <SPI.h>

#include "bme280_spi.hpp"
#include "Peripherals/IO_Map.h"
#include "DataModels.h"



/*
int main() {
    stdio_init_all();
    sleep_ms(3000); // required for some OSses to make Pico visible

    // using default SPI pins
    // frequency = 0.5 MHz
    // FORCED_MODE to reduce energy consumption
    BME280 myBME280(0, 
            PICO_DEFAULT_SPI_RX_PIN, 
            PICO_DEFAULT_SPI_TX_PIN, 
            PICO_DEFAULT_SPI_SCK_PIN, 
            PICO_DEFAULT_SPI_CSN_PIN, 
            500 * 1000,
            BME280::MODE::MODE_FORCED);
    BME280::Measurement_t values;

    // empty read as a warm-up
    myBME280.measure();
    sleep_ms(100);

    while (true) {
        // get measurement from BME280
        values = myBME280.measure();
        // create formatted strings with measurement results
        printf("temperature: %6.1f C\n",   values.temperature);
        printf("humidity   : %6.1f %c\n",  values.humidity, '%');
        printf("pressure   : %6.1f hPa\n", values.pressure);
        printf("altitude   : %6.1f m\n",   values.altitude);
        printf("\n\n");
        sleep_ms(2000);
    }   
}*/

BME280 bme(
    0,              // SPI bus (spi0)
    SPI_MISO_PIN,           // RX
    SPI_MOSI_PIN,           // TX
    SPI_SCK_PIN,            // SCK
    HUM_CS_PIN,             // CS
    500000,         // 500 kHz
    BME280::MODE::MODE_FORCED
);

int bme_setup() {
    auto id = bme.get_chipID();
    if (id != 0x60) {
        Serial.print("BME280 wrong ID: 0x");
        Serial.println(id, HEX);
        return -1;
    }
    return 0;
}

int read_bme(data_t* data) {
    auto m = bme.measure();

    data->temperature_bme = m.temperature;
    data->pressure_bme    = m.pressure;
    data->humidity_bme    = m.humidity;

    return 0;
}

