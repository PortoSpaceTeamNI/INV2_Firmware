#include "UART.h"
#include "Communications.h"
#include "Commands.h"
#include "Configs.h"
#include "Queues.h"
#include "Pinout.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

extern QueueHandle_t AcknowledgementQueue;

void vUARTTask(void *pvParameters)
{
  static packet_t rxPacket;
  int error = 0;

  while (true)
  {
    packet_t *receivedPacket = read_packet(&error, UART_INTERFACE);

    if (receivedPacket != nullptr && error == CMD_READ_OK)
    {
      rxPacket = *receivedPacket;
      xQueueSend(AcknowledgementQueue, &rxPacket, 0);
    }

    // Optional: only print real UART parser issues.
    if (error == CMD_READ_TIMEOUT || error == CMD_READ_BAD_CRC)
    {
      Serial.printf("UART read error=%d\n", error);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}