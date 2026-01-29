//o que é o packet?

// pôr na lib. os 3 ficheiros .c e .h;

/* na pasta include >> Peripherals; hardwareCfg.h; DataModels.h    --> pôr o quê?; Sensors.h  --> adicionar o bmp.h; 
						            50 ms
						            ADC_POLL_INTERVAL
						            THERMO_POLL_INTERVAL

IO_MAP.h   --> SDA_PIN 20
               SCL_PIN 21
               BAR2_DRDY 23

Dentro do Peripherals >> pasta bmp581.h*/


/* Sensors.cpp

read_sensors --> não preciso da taxa de 400 Hz, certo? o anterior tem BMP_POLL_INTERVAL? */

#ifndef BMP581_H
#define BMP581_H

// Declare bmp581_dev as an external variable
extern struct bmp5_dev bmp581_dev;

#include <Wire.h>
#include <bmp5.h>  // Bosch BMP581 sensor library
#include "DataModels.h"


int bmp_setup(void);
int read_baro2(data_t *data);

#endif // BMP581_H