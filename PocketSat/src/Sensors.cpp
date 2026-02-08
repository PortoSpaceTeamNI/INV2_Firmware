#include "Sensors.h"

SensorDataResult sensorData;

int InitializeSensors()
{
    int ret = 0;
    if (InitializeHDC3020() != 0)
    {
        Serial.println("Failed to initialize HDC3020 sensor!");
        ret = -1;
    }
    else
    {
        Serial.println("HDC3020 sensor initialized successfully.");
    }

    if (InitializeBMP581() != 0)
    {
        Serial.println("Could not find BMP581!");
        ret = -1;
    }
    else
    {
        Serial.println("BMP581 sensor initialized successfully.");
    }

    if (InitializeLPS22DF() != 0)
    {
        Serial.println("Failed to initialize LPS22DF sensor!");
        ret = -1;
    }
    else
    {
        Serial.println("LPS22DF sensor initialized successfully.");
    }

    if (InitializeBME280() != 0)
    {
        Serial.println("Failed to initialize BME280 sensor!");
        ret = -1;
    }
    else
    {
        Serial.println("BME280 sensor initialized successfully.");
    }

    return ret;
}

int ConfigureSensors()
{
    int ret = 0;
    if (ConfigureBMP581() != 0)    {
        Serial.println("Failed to configure BMP581 sensor!");
        ret = -1;
    }
    if (ConfigureLPS22DF() != 0)    {
        Serial.println("Failed to configure LPS22DF sensor!");
        ret = -1;
    }
    if (ConfigureBME280() != 0)    {
        Serial.println("Failed to configure BME280 sensor!");
        ret = -1;
    }
    return ret;
}

int ReadSensors()
{
    int ret = 0;
    if(IsHDC3020Ready())
    { 
        if (ReadHDC3020(sensorData.hdcData) != 0)
        {
            Serial.println("Failed to read HDC3020 sensor data!");
            ret = -1;
        }
    }
    
    if(IsBMP581Ready())
    {
        if (ReadBMP581(sensorData.bmpData) != 0)
        {
            Serial.println("Failed to read BMP581 sensor data!");
            ret = -1;
        }
    }

    if (IsLPS22DFReady())
    {
        if (ReadLPS22DF(sensorData.lpsData) != 0)
        {
            Serial.println("Failed to read LPS22DF sensor data!");
            ret = -1;
        }
    }

    if (IsBME280Ready())
    {
        if (ReadBME280(sensorData.bmeData) != 0)
        {
            Serial.println("Failed to read BME280 sensor data!");
            ret = -1;
        }
    }
    return ret;
}