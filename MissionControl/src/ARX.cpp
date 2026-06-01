#include "Communications.h"
#include "Commands.h"
#include "Queues.h"
#include "Configs.h"
#include <Arduino.h>

extern QueueHandle_t AcknowledgementQueue;
extern QueueHandle_t CommandQueue;

void vARXTask(void *pvParameters)
{
  //Serial1.println("[TASK] ARX task started");
  while (true)
  {
    bool hadPacket = false;
    //Serial1.println("[TASK] Running: ARX");
    int error;
    packet_t *receivedPacket = read_packet(&error, UART_INTERFACE);
    if (receivedPacket != NULL && error == CMD_READ_OK)
    {
        hadPacket = true;
        if (xQueueSend(CommandQueue, receivedPacket, 0) != pdPASS)
        {
            Serial1.println("Failed to send received packet to CommandQueue");
        }
    } else if (receivedPacket != NULL) {
        Serial1.print("Error reading packet from ARX: ");
        Serial1.println(error);
    }
    if (!hadPacket) {
        // If no packet received, check if we have a packet to send
        packet_t txPacket;
        if (xQueueReceive(AcknowledgementQueue, &txPacket, 0) == pdPASS)
        {
            if (!write_packet(&txPacket, UART_INTERFACE)) {
                Serial1.println("Failed to write packet to ARX");
            }
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}