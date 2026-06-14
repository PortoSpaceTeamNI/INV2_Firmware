#ifndef PRESSURES_H
#define PRESSURES_H

#include <AD5593R.h>
#include "DataModels.h"

void read_adc_channels(data_t *data);
int pressures_setup(void);

#endif // PRESSURES_H