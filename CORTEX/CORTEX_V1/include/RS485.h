#ifndef RS485_H
#define RS485_H

#include <FreeRTOS.h>
#include <semphr.h>

// Mutex for protecting waitingForResponse flag
extern SemaphoreHandle_t waitingForResponseMutex;

void vRS485Task(void *pvParameters);

#endif // RS485_H