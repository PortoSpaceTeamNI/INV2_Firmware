#ifndef LPS_H
#define LPS_H

#include <LPS.h>

#define LPS22DF_ADDR 0x5C

struct LPS22DFDataResult
{
    /// @brief Pressure in hPa
    float Pressure = 0.0f;
    /// @brief Temperature in Celcius
    float Temperature = 0.0f;
    /// @brief Altitude in meters
    float Altitude = 0.0f;
};

int InitializeLPS22DF();
int ConfigureLPS22DF();
int ReadLPS22DF(LPS22DFDataResult& result);
bool IsLPS22DFReady();

#endif // LPS_H