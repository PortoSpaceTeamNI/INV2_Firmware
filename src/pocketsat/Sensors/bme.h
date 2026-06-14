#ifndef BME_H
#define BME_H

struct BME280DataResult
{
    /// @brief Temperature in Celcius
    float Temperature = 0.0f;
    /// @brief Pressure in hPa
    float Pressure = 0.0f;
    /// @brief Humidity in percentage
    float Humidity = 0.0f;
};

int InitializeBME280();
int ConfigureBME280();
int ReadBME280(BME280DataResult& result);
bool IsBME280Ready();

#endif // BME_H