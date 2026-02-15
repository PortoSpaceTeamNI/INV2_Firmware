#include "Queues.h"
#include "StateMachine.h"
#include "Communications.h"
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

QueueHandle_t EventQueue;
QueueHandle_t ManualCommandQueue;
QueueHandle_t StatusQueue;
QueueHandle_t AcknowledgementQueue;

int createQueues() {
  EventQueue = xQueueCreate(10, sizeof(RocketEvent));
  ManualCommandQueue = xQueueCreate(10, sizeof(packet_t));
  StatusQueue = xQueueCreate(10, sizeof(packet_t));
  AcknowledgementQueue = xQueueCreate(10, sizeof(packet_t));
  
  if (!EventQueue || !ManualCommandQueue || !StatusQueue || !AcknowledgementQueue) {
    return -1;
  }
  return 0;
}

