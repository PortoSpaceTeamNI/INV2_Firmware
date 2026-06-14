#ifndef LOADCELLS_H
#define LOADCELLS_H

#include <HX711.h>
#include "DataModels.h"

int loadcells_setup(void);
int read_loadcells(data_t *data);
void calibrate_loadcells();
#endif // LOADCELLS_H
