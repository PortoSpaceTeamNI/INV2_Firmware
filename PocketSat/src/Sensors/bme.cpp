#include "Sensors/bme.h"
#include "Sensors.h"
#include "Pinout.h"
#include <Arduino.h>
#include <BME280SpiSw.h>

// Default: forced mode, standby time 1000ms, oversampling x1 for all, filter off
BME280SpiSw::Settings settings(BME280_CS_PIN, BME280_MOSI_PIN, BME280_MISO_PIN, BME280_SCK_PIN);
BME280SpiSw bme(settings);

int InitializeBME280()
{
    SPI1.setMISO(BME280_MISO_PIN);
    SPI1.setMOSI(BME280_MOSI_PIN);
    SPI1.setSCK(BME280_SCK_PIN);
    SPI1.begin();

    if (!bme.begin())
    {
        return -1; // Return -1 on failure
    }

    switch (bme.chipModel())
    {
    case BME280::ChipModel_BME280:
        break;
    case BME280::ChipModel_BMP280:
        Serial.println("BMP280 detected instead of BME280. Check wiring and connections.");
        return -1; // Return -1 if the wrong chip is detected
    default:
        Serial.println("Unknown chip model detected. Check wiring and connections.");
        return -1; // Return -1 if an unknown chip is detected
    }
    return 0; // Return 0 on success
}

int ConfigureBME280()
{

    return 0;
}

bool IsBME280Ready()
{
    return true;
}

int ReadBME280(BME280DataResult &result)
{
    bme.read(result.Pressure, result.Temperature, result.Humidity);
    return 0;
}