#include "RS485.h"
#include "Communications.h"
#include "Commands.h"
#include "Configs.h"
#include "Queues.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

bool waitingForResponse = false;
uint32_t waitingStartTime = 0; // Timestamp when we started waiting

// External queue declarations
extern QueueHandle_t ManualCommandQueue;
extern QueueHandle_t StatusQueue;
extern QueueHandle_t AcknowledgementQueue;

void vRS485Task(void *pvParameters)
{
  packet_t rxPacket;
  packet_t txPacket;
  int error = 0;
  bool isWaiting;

  //Serial.println("[TASK] RS485 task started");
  while (true)
  {
    //Serial.println("[TASK] Running: RS485");
    if (isWaiting)
    {
      if (millis() - waitingStartTime > RS485_RESPONSE_TIMEOUT_MS)
      {
        // Timeout occurred - clear waiting flag and allow next transmission
        waitingForResponse = false;
        isWaiting = false;

        // TODO: Send NACK to AcknowledgementQueue to indicate timeout error
      }
      // Read if data is available on RS485
      packet_t *receivedPacket = read_packet(&error, RS485_INTERFACE);

      // If data received, send to AckQ and set waitingForResponse to false
      if (receivedPacket != NULL && error == CMD_READ_OK)
      {
        rxPacket = *receivedPacket;
        if (xQueueSend(AcknowledgementQueue, &rxPacket, 0) == pdPASS)
        {
          // Update waitingForResponse flag
          waitingForResponse = false;
        }
      }
    }
    else
    {
      // Read from StatusQ and ManCmdQ and write to RS485

      // Try to read from ManualCommandQueue first (higher priority)
      if (xQueueReceive(ManualCommandQueue, &txPacket, 0) == pdPASS)
      {
        write_packet(&txPacket, RS485_INTERFACE);
        // Update waitingForResponse
        waitingForResponse = true;
        waitingStartTime = millis();
      }
      // If nothing in ManualCommandQueue, try StatusQueue
      else if (xQueueReceive(StatusQueue, &txPacket, 0) == pdPASS)
      {
        write_packet(&txPacket, RS485_INTERFACE);
        // Update waitingForResponse flag
        waitingForResponse = true;
        waitingStartTime = millis();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}