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
#include "RadioLib.h"

// External queue declarations
extern QueueHandle_t CommandQueue;
extern QueueHandle_t AcknowledgementQueue;

// ════ RadioLib SX1281 Objects ════
SPIClassRP2040 radioSPI(spi0, SPIO0_MISO_PIN, RADIO_CS_PIN, SPIO0_SCK_PIN, SPIO0_MOSI_PIN);
SX1281 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RESET_PIN, RADIO_BUSY_PIN, radioSPI);

// ════ ISR and Synchronization ════
volatile bool radioWaitingForResponse = false;
uint32_t radioWaitingStartTime = 0; // Timestamp when we started waiting

// Binary semaphore for signaling RX/TX complete from ISR
SemaphoreHandle_t radioISRSemaphore = NULL;

// ISR handler for radio packet transmission/reception complete
void onRadioISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (radioISRSemaphore != NULL) {
    xSemaphoreGiveFromISR(radioISRSemaphore, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ════ RF Switch Control ════
static inline void rfIdle() {
  digitalWrite(RADIO_TX_ENABLE_PIN, LOW);
  digitalWrite(RADIO_RX_ENABLE_PIN, LOW);
}

static inline void rfTx() {
  digitalWrite(RADIO_RX_ENABLE_PIN, LOW);
  digitalWrite(RADIO_TX_ENABLE_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH); // LED on during transmission
}

static inline void rfRx() {
  digitalWrite(RADIO_TX_ENABLE_PIN, LOW);   
  digitalWrite(RADIO_RX_ENABLE_PIN, HIGH);
  //digitalWrite(LED_PIN, LOW); // LED off during reception
}

int setupRadio() {
  // Create binary semaphore for ISR signaling (must be called before task creates it)
  if (radioISRSemaphore == NULL) {
    radioISRSemaphore = xSemaphoreCreateBinary();
    if (radioISRSemaphore == NULL) {
      Serial2.println("[RADIO] Failed to create ISR semaphore");
      return -1;
    }
  }

  // Configure RF switch control pins
  pinMode(RADIO_TX_ENABLE_PIN, OUTPUT);
  pinMode(RADIO_RX_ENABLE_PIN, OUTPUT);
  rfIdle();

  // Initialize SPI bus
  radioSPI.begin();

  // Initialize LoRa radio
  Serial2.print("[RADIO] Initializing SX1281 … ");
  int err = radio.begin(RADIO_FREQUENCY,
                        RADIO_BANDWIDTH,
                        RADIO_SPREAD_FACTOR,
                        RADIO_CODING_RATE,
                        RADIO_SYNC_WORD,
                        RADIO_TX_POWER,
                        RADIO_PREAMBLE_LEN);

  if (err != RADIOLIB_ERR_NONE) {
    Serial2.print("FAILED  code=");
    Serial2.println(err);
    return -1;
  }
  Serial2.println("OK");
  Serial2.printf("  %.3f MHz  BW=%.3f kHz  SF=%d  CR=4/%d  ACK timeout=%lu ms\n",
                RADIO_FREQUENCY, RADIO_BANDWIDTH, RADIO_SPREAD_FACTOR,
                RADIO_CODING_RATE, RADIO_RESPONSE_TIMEOUT_MS);

  // Attach ISR handlers (single DIO1 for all IRQs on SX1281)
  radio.setPacketSentAction(onRadioISR);
  radio.setPacketReceivedAction(onRadioISR);

  return 0;  // Success
}

void vRadioTask(void *pvParameters)
{
  static packet_t rxPacket;
  static packet_t txPacket;
  int error = 0;

  //Serial2.println("[TASK] Radio task started");

  while (true)
  {
    // ════ TX State: Waiting for ACK to a transmitted command ════
    if (radioWaitingForResponse)
    {
      // Check for timeout
      if (millis() - radioWaitingStartTime > RADIO_RESPONSE_TIMEOUT_MS)
      {
        // Timeout occurred - send NACK to acknowledge queue
        radioWaitingForResponse = false;
        packet_t nackPacket;
        create_packet(&nackPacket, DEVICE_ID, BROADCAST_ID, CMD_NACK, NULL, 0);
        xQueueSend(AcknowledgementQueue, &nackPacket, 0);
        Serial2.println("[Radio] Response timeout, sent NACK");
      }
      else
      {
        // Try to read response from radio
        packet_t *receivedPacket = read_packet(&error, RADIO_INTERFACE);

        if (receivedPacket != NULL && error == CMD_READ_OK)
        {
          // Valid ACK received - forward to acknowledgement queue
          rxPacket = *receivedPacket;
          if (xQueueSend(AcknowledgementQueue, &rxPacket, 0) == pdPASS)
          {
            radioWaitingForResponse = false;
          }
        }
      }
    }
    // ════ RX State: Listening for commands or ready to transmit ════
    else
    {
      // Try to read pending command from queue
      if (xQueueReceive(CommandQueue, &txPacket, 0) == pdPASS)
      {
        // Transmit command over radio
        write_packet(&txPacket, RADIO_INTERFACE);
        // Mark that we're waiting for a response
        radioWaitingForResponse = true;
        radioWaitingStartTime = millis();
      }
    }

    // Yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}