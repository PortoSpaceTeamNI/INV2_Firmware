#include "Queues.h"
#include "Communications.h"
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

QueueHandle_t CommandQueue;
QueueHandle_t AcknowledgementQueue;

int createQueues() {
  CommandQueue = xQueueCreate(10, sizeof(packet_t));
  AcknowledgementQueue = xQueueCreate(10, sizeof(packet_t));

  if (!CommandQueue || !AcknowledgementQueue) {
    return -1;
  }
  return 0;
}

