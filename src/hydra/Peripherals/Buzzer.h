#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>
#include "Peripherals/IO_Map.h"

void setup_buzzer();

void play_buzzer_error();
void play_buzzer_success();

#endif
// BUZZER_H