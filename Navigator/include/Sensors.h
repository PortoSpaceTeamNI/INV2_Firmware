#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>

#include "Pinout.h"
#include "Sensors/bmp.h"
#include "Sensors/lsm.h"
#include "Sensors/lps.h"
#include "Sensors/lis.h"

typedef struct
{
    BMP581DataResult bmpData;
    LSM6DSODataResult lsmData;
    LPS22DFDataResult lpsData;
    LIS2MDLDataResult lisData;
} SensorDataResult;

int InitializeSensors();
int ConfigureSensors();
int ReadSensors();

#endif // SENSORS_H