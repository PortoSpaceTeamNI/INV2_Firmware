#include "Sensors.h"
#include "Sensors/bmp.h"
#include <Adafruit_BMP5xx.h>

Adafruit_BMP5xx bmp;

int InitializeBMP581()
{
    pinMode(BAR2_RDY_PIN, INPUT);

    int ret = bmp.begin(BMP581_ADDR, &Wire);
    return ret ? 0 : -1; // Return 0 on success, -1 on failure
}

int ConfigureBMP581()
{
    int ret = 0;
    if (!bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X)) {
        Serial.println("BMP581: setTemperatureOversampling failed.");
        ret = -1;
    }
    if (!bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X)) {
        Serial.println("BMP581: setPressureOversampling failed.");
        ret = -1;
    }
    if (!bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3)) {
        Serial.println("BMP581: setIIRFilterCoeff failed.");
        ret = -1;
<<<<<<< HEAD
    // Set to higher output data rate for faster updates
    if (bmp.setOutputDataRate(BMP5XX_ODR_240_HZ) != 0)
=======
    }
    if (!bmp.setOutputDataRate(BMP5XX_ODR_50_HZ)) {
        Serial.println("BMP581: setOutputDataRate failed.");
>>>>>>> 82799b79a740b71aeff4e4c836784823be307a9d
        ret = -1;
    }
    if (!bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL)) {
        Serial.println("BMP581: setPowerMode failed.");
        ret = -1;
    }
    if (!bmp.enablePressure(true)) {
        Serial.println("BMP581: enablePressure failed.");
        ret = -1;
<<<<<<< HEAD
    // Configure interrupt for data ready (latched, active low, push-pull)
    if (bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED, BMP5XX_INTERRUPT_ACTIVE_LOW, BMP5XX_INTERRUPT_PUSH_PULL, BMP5XX_INTERRUPT_DATA_READY, true) != 0)
=======
    }
    if (!bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED, BMP5XX_INTERRUPT_ACTIVE_HIGH, BMP5XX_INTERRUPT_PUSH_PULL, BMP5XX_INTERRUPT_DATA_READY, true)) {
        Serial.println("BMP581: configureInterrupt failed.");
>>>>>>> 82799b79a740b71aeff4e4c836784823be307a9d
        ret = -1;
    }
    return ret;
}

bool IsBMP581Ready()
{
    return bmp.dataReady();
}

int ReadBMP581(BMP581DataResult& result)
{
    if (bmp.performReading())
    {
        result.Temperature = bmp.temperature;
        result.Pressure = bmp.pressure;
        result.Altitude = bmp.readAltitude();
        return 0;
    }
    else
    {
        Serial.println("Failed to perform reading from BMP581!");
        return -1;
    }
}