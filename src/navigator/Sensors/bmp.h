#ifndef BMP_H
#define BMP_H

// Declare bmp581_dev as an external variable
extern struct bmp5_dev bmp581_dev;

#define BMP581_ADDR 0x46

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

#endif // BMP581_H