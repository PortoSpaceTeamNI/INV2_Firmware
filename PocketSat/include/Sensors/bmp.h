#ifndef BMP_H
#define BMP_H

struct BMP581DataResult
{
    /// @brief Temperature in Celcius
    float Temperature = 0.0f;
    /// @brief Pressure in hPa
    float Pressure = 0.0f;
    /// @brief Altitude in meters
    float Altitude = 0.0f;
};

int InitializeBMP581();
int ConfigureBMP581();
int ReadBMP581(BMP581DataResult& result);
bool IsBMP581Ready();

#endif // BMP_H