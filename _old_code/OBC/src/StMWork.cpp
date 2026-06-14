#include <Arduino.h>

#include <time.h>

#include "HardwareCfg.h"
#include "StMWork.h"
#include "StMComms.h"
#include "GlobalVars.h"
#include "Comms.h"

#include <Crc.h>

void timer_tick(uint16_t *timer) { (*timer)++; }
