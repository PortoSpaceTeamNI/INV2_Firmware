#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>

#include "Pinout.h"
#include "Sensors/bmp.h"

typedef struct
{
    BMP581DataResult bmpData;
    // LSM6DODataResult lsmData;
    // LPS22DFDataResult lpsData;
} SensorDataResult;

int InitializeSensors();
int ConfigureSensors();
int ReadSensors();

#endif // SENSORS_H