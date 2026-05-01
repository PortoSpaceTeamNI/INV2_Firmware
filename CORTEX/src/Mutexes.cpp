#include "Mutexes.h"
#include <FreeRTOS.h>
#include <semphr.h>

SemaphoreHandle_t rocketDataMutex;
SemaphoreHandle_t stateMachineConfigsMutex;

int createMutexes() {
  rocketDataMutex = xSemaphoreCreateMutex();
  stateMachineConfigsMutex = xSemaphoreCreateMutex();
  
  if (!rocketDataMutex || !stateMachineConfigsMutex) {
    return -1;
  }
  return 0;
}