#ifndef LIS2MDL_H
#define LIS2MDL_H

#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>

struct LIS2MDLDataResult
{
    /// @brief Magnetic field in microteslas (uT)
    float MagX = 0.0f;
    float MagY = 0.0f;
    float MagZ = 0.0f;
};

int InitializeLIS2MDL();
int ConfigureLIS2MDL();
int ReadLIS2MDL(LIS2MDLDataResult& result);
bool IsLIS2MDLReady();

#endif // LIS2MDL_H