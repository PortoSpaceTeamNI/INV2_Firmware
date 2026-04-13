#ifndef COMMS_H
#define COMMS_H

#include "Sensors.h"

void InitializeSensorUART();
void SendSensorDataOverUART(const SensorDataResult& data);

#endif // COMMS_H