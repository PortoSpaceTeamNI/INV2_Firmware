#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "Peripherals/IO_Map.h"
#include "DataModels.h"
#include "Peripherals/bmp581.h"

void read_sensors(data_t *data);

#endif // SENSORS_H