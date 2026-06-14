#ifndef THERMO_H
#define THERMO_H

#include <Adafruit_MAX31856.h>
#include "DataModels.h"

int thermo_setup(void);
int read_thermocouples(data_t *data);

#endif // THERMO_H
