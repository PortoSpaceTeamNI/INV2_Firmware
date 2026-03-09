#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <HDC302x.h>
#include <Adafruit_BMP5xx.h>

#include "Pinout.h"
#include "Sensors/hdc.h"
#include "Sensors/bmp.h"
#include "Sensors/lps.h"
#include "Sensors/bme.h"

#define BMP581_ADDR 0x46
#define HDC302X_ADDR 0x44
#define LPS22DF_ADDR 0x5C

typedef struct
{
    BMP581DataResult bmpData;
    HDC302xDataResult hdcData;
    LPS22DFDataResult lpsData;
    BME280DataResult bmeData;
} SensorDataResult;

int InitializeSensors();
int ConfigureSensors();
int ReadSensors();

#endif // SENSORS_H