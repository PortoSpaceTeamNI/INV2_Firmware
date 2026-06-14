#include "Peripherals/Thermo.h"
#include "Peripherals/IO_Map.h"

Adafruit_MAX31856 maxthermo1 = Adafruit_MAX31856(THERMO1_CS_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN); // cs, mosi, miso, clk
Adafruit_MAX31856 maxthermo2 = Adafruit_MAX31856(THERMO2_CS_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN);
Adafruit_MAX31856 maxthermo3 = Adafruit_MAX31856(THERMO3_CS_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN);

int thermo_setup(void)
{
    pinMode(THERMO1_DRDY_PIN, INPUT);
    pinMode(THERMO2_DRDY_PIN, INPUT);
    pinMode(THERMO3_DRDY_PIN, INPUT);

    if (!maxthermo1.begin())
    {
        Serial.println("Could not initialize thermocouple 1.");
        return -1;
    }

    if (!maxthermo2.begin())
    {
        Serial.println("Could not initialize thermocouple 2.");
        return -1;
    }

    if (!maxthermo3.begin())
    {
        Serial.println("Could not initialize thermocouple 3.");
        return -1;
    }

    maxthermo1.setConversionMode(MAX31856_CONTINUOUS);
    maxthermo2.setConversionMode(MAX31856_CONTINUOUS);
    maxthermo3.setConversionMode(MAX31856_CONTINUOUS);

    return 0; // Successful initialization
}

int read_thermocouples(data_t *data)
{
    if (digitalRead(THERMO1_DRDY_PIN) == LOW)
    {
        float temp1 = maxthermo1.readThermocoupleTemperature();
        data->thermo1 = (int16_t)(temp1 * 100);
    }

    if (digitalRead(THERMO2_DRDY_PIN) == LOW)
    {
        float temp2 = maxthermo2.readThermocoupleTemperature();
        data->thermo2 = (int16_t)(temp2 * 100);
    }

    if (digitalRead(THERMO3_DRDY_PIN) == LOW)
    {
        float temp3 = maxthermo3.readThermocoupleTemperature();
        data->thermo3 = (int16_t)(temp3 * 100);
    }
    return 0; // Successful read
}
