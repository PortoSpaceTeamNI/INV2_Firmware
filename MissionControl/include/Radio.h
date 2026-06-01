#ifndef RADIO_H
#define RADIO_H

#include <FreeRTOS.h>
#include <semphr.h>
#include "RadioLib.h"

// RadioLib SX1281 radio object (defined in Radio.cpp)
extern SX1281 radio;

// Binary semaphore for ISR signaling
extern SemaphoreHandle_t radioISRSemaphore;

// Mutex for protecting waitingForResponse flag
extern SemaphoreHandle_t waitingForResponseMutex;

int setupRadio();

void vRadioTask(void *pvParameters);

#endif // RADIO_H