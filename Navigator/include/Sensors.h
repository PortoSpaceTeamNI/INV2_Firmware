#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>

#include "Pinout.h"
#include "Sensors/bmp.h"
#include "Sensors/lsm.h"
#include "Sensors/lps.h"

typedef struct
{
    BMP581DataResult bmpData;
    LSM6DSODataResult lsmData;
    LPS22DFDataResult lpsData;
} SensorDataResult;

int InitializeSensors();
int ConfigureSensors();
int ReadSensors();

#endif // SENSORS_H