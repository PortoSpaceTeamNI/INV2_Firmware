#include "Sensors/lps.h"
#include <LPS.h>
#include "Pinout.h"
#include "Sensors.h"
#include <Arduino.h>
#include <Wire.h>
LPS lps;

int InitializeLPS22DF()
{
    pinMode(LPS22DF_RDY_PIN, INPUT_PULLUP);

    bool ret = lps.init(LPS::device_22DF, LPS::sa0_low, &Wire);
    return ret ? 0 : -1; // Return 0 on success, -1 on failure
}

int ConfigureLPS22DF()
{
    lps.enableDefault();
    return 0;
}

bool IsLPS22DFReady()
{

    return digitalRead(LPS22DF_RDY_PIN) == LOW; // RDY pin goes LOW when data is ready, per datasheet
}

int ReadLPS22DF(LPS22DFDataResult &result)
{
    result.Pressure = lps.readPressureMillibars(); // mBar = hPa
    result.Temperature = lps.readTemperatureC();
    result.Altitude = lps.pressureToAltitudeMeters(result.Pressure);
    return 0;
}