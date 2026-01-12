

#include <Wire.h>
#include <Adafruit_HDC302x.h>
#include "DataModels.h"

#ifndef HDC_H
#define HDC_H

#define HDC_I2C_ADDR 0x44

extern Adafruit_HDC302x hdc_dev;

int hdc_setup(void);
int read_hdc(data_t *data);

#endif // HDC_H