#ifndef VALVES_H
#define VALVES_H

#include <inttypes.h>
#include "DataModels.h"
#include "Peripherals/IO_Map.h"
#include <Arduino.h>

#define SERVO_OPEN_PULSE_WIDTH 500
#define SERVO_CLOSE_PULSE_WIDTH 2500
int valves_setup(data_t *data);

void valve_set(data_t *data, uint8_t valve, uint8_t state);
void close_all_valves(data_t *data);
#endif // VALVES_H