//o que é o packet?

// pôr na lib. os 3 ficheiros .c e .h;

/* na pasta include >> Peripherals; hardwareCfg.h; DataModels.h    --> pôr o quê?; Sensors.h  --> adicionar o bmp.h; 
						            50 ms
						            ADC_POLL_INTERVAL
						            THERMO_POLL_INTERVAL*/


#ifndef LPS_H
#define LPS_H

#include <stdint.h>
#include "DataModels.h"

// Include C driver with C++ compatibility
extern "C" {
#include "lps22df_reg.h"
}

// LPS22DF I2C address (you can choose L or H depending on SA0 pin)
#define LPS_I2C_ADDR 0x5C

// Expose a context for the sensor, similar to bmp5_dev
extern stmdev_ctx_t lps22df_ctx;

// Function prototypes to mimic BMP interface
int lps_setup(void);                      // Initialize LPS22DF sensor
int read_baro3(data_t *data);             // Read pressure and temperature

#endif // LPS_H
