#ifndef RS485_H
#define RS485_H

#include <Arduino.h>
#include "Peripherals/IO_Map.h"

void rs485_init();
void rs485_send(uint8_t *data, size_t length);

#endif // RS485_H