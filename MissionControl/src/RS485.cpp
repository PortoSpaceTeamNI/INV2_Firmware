#include "RS485.h"
#include "Communications.h"
#include "Commands.h"
#include "Configs.h"
#include "Queues.h"
#include "Pinout.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

volatile bool rs485WaitingForResponse = false;
uint32_t rs485WaitingStartTime = 0; // Timestamp when we started waiting

// External queue declarations
extern QueueHandle_t PacketQueue;
extern QueueHandle_t AcknowledgementQueue;

void vRS485Task(void *pvParameters)
{
  static packet_t rxPacket;
  static packet_t txPacket;
  static uint32_t lastTestPatternTime = 0;
  static bool testTxInitialized = false;
  int error = 0;

  //Serial1.println("[TASK] RS485 task started");
  while (true)
  {
    //Serial1.println("[TASK] Running: RS485");
#if RS485_TEST_PATTERN_ENABLED
    if (!testTxInitialized)
    {
#if RS485_TEST_CONTINUOUS_TX
  digitalWrite(ENABLE_RS_PIN, RS485_TX_ENABLE_LEVEL);
  delayMicroseconds(RS485_TX_ENABLE_SETUP_US);
#endif
  testTxInitialized = true;
    }

    if ((millis() - lastTestPatternTime) >= RS485_TEST_PATTERN_INTERVAL_MS)
    {
      uint8_t testBytes[2] = {RS485_TEST_PATTERN_BYTE0, RS485_TEST_PATTERN_BYTE1};
#if RS485_TEST_CONTINUOUS_TX
  Serial2.write(testBytes, sizeof(testBytes));
  Serial2.flush();
#else
      write_to_rs485(testBytes, sizeof(testBytes));
#endif

      lastTestPatternTime = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    continue;
#endif

    if (rs485WaitingForResponse)
    {
      if (millis() - rs485WaitingStartTime > RS485_RESPONSE_TIMEOUT_MS)
      {
        // Timeout occurred - clear waiting flag and allow next transmission
        rs485WaitingForResponse = false;
        // Send nack packet to AckQ to indicate timeout
        packet_t nackPacket;
        create_packet(&nackPacket, DEVICE_ID, BROADCAST_ID, CMD_NACK, NULL, 0);
        xQueueSend(AcknowledgementQueue, &nackPacket, 0);
        // Serial1.println("[RS485] Response timeout, sent NACK");
      }
      // Read if data is available on RS485
      packet_t *receivedPacket = read_packet(&error, RS485_INTERFACE);

      // If data received, send to AckQ and set waitingForResponse to false
      if (receivedPacket != NULL && error == CMD_READ_OK)
      {
        //tone(BUZZER_PIN, 1500, 50); // Beep to indicate received response
        rxPacket = *receivedPacket;
        if (xQueueSend(AcknowledgementQueue, &rxPacket, 0) == pdPASS)
        {
          // Update rs485WaitingForResponse flag
          rs485WaitingForResponse = false;
        }
      }
    }
    else
    {
      // Read from PacketQ and write to RS485
      if (xQueueReceive(PacketQueue, &txPacket, 0) == pdPASS)
      {
        write_packet(&txPacket, RS485_INTERFACE);
        // Update rs485WaitingForResponse
        rs485WaitingForResponse = true;
        rs485WaitingStartTime = millis();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}