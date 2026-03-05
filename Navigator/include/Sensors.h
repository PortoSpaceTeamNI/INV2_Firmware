#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>

#include "Pinout.h"

// Sensor Unit 1
#include "Sensors/bme.h"

// Sensor Unit 2
#include "Sensors/bmp.h"
#include "Sensors/lsm.h"
#include "Sensors/lps.h"


typedef struct
{
    BMP581DataResult bmpData;
    LSM6DSODataResult lsmData;
    LPS22DFDataResult lpsData;
    BME280DataResult bmeData;
} SensorDataResult;

int InitializeSensors();
int ConfigureSensors();
int ReadSensors();

#endif // SENSORS_H