#ifndef BMI323_SENSOR_H
#define BMI323_SENSOR_H

#include <Arduino.h>

typedef struct {
    float AccelX;
    float AccelY;
    float AccelZ;
    float GyroX;
    float GyroY;
    float GyroZ;
    float Temperature;
} BMI323DataResult;

int InitializeBMI323();
int ConfigureBMI323();
bool IsBMI323Ready();
int ReadBMI323(BMI323DataResult &result);

#endif // BMI323_SENSOR_H
