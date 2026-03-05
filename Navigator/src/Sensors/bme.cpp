#include "Sensors/bme.h"
#include "Sensors.h"
#include "Pinout.h"
#include <Arduino.h>
#include <BME280SpiSw.h>

BME280SpiSw::Settings* settings = nullptr;
BME280SpiSw* bme = nullptr;

int InitializeBME280()
{
    settings = new BME280SpiSw::Settings(BME280_CS_PIN, BME280_MOSI_PIN, BME280_MISO_PIN, BME280_SCK_PIN);
    bme = new BME280SpiSw(*settings);

    if (!bme->begin())
    {
        return -1;
    }

    return 0;
}

int ConfigureBME280() { return 0; }

bool IsBME280Ready() { return true; }

int ReadBME280(BME280DataResult &result)
{
    bme->read(result.Pressure, result.Temperature, result.Humidity);
    return 0;
}