#include "DataPolling.h"
#include "Communications.h"
#include "Commands.h"
#include "Configs.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

extern QueueHandle_t AcknowledgementQueue;
extern QueueHandle_t StatusQueue;
extern SemaphoreHandle_t rocketDataMutex;
extern SemaphoreHandle_t waitingForResponseMutex;
extern bool waitingForResponse;

RocketData rocketData;

int requestStatus(uint8_t targetID)
{
  packet_t packet;
  uint8_t payload[1] = {0}; // No payload for status request
  create_packet(&packet, CORTEX_ID, targetID, CMD_STATUS, payload, 0);

  // Use timeout when sending to queue to avoid blocking indefinitely
  if (xQueueSend(StatusQueue, &packet, pdMS_TO_TICKS(100)) == pdPASS)
  {
    return 0;
  }
  return -1; // Failed to send
}

int pollNextSlave()
{
  static uint8_t nextSlaveId = HYDRA_UF_ID; // Start polling from the first slave

  requestStatus(nextSlaveId);

  // Round-robin to the next slave for the next poll
  nextSlaveId++;
  if (nextSlaveId > LIFT_THRUST_ID)
  {
    nextSlaveId = HYDRA_UF_ID; // Wrap around to the first slave
  }
  return 0;
}

void processStatusAck(packet_t *ack)
{
  // Process status acknowledgment based on sender ID
  // TODO: Maybe will change if bools are sent as bitfields

  // Take mutex before accessing rocketData
  if (xSemaphoreTake(rocketDataMutex, portMAX_DELAY) == pdTRUE)
  {
    switch (ack->sender_id)
    {
    case HYDRA_UF_ID:
      if (ack->payload_size >= sizeof(HydraUFData))
      {
        memcpy(&rocketData.hydraUFData, ack->payload, sizeof(HydraUFData));
      }
      break;
    case HYDRA_LF_ID:
      if (ack->payload_size >= sizeof(HydraLFData))
      {
        memcpy(&rocketData.hydraLFData, ack->payload, sizeof(HydraLFData));
      }
      break;
    case HYDRA_FS_ID:
      if (ack->payload_size >= sizeof(HydraFSData))
      {
        memcpy(&rocketData.hydraFSData, ack->payload, sizeof(HydraFSData));
      }
      break;
    case NAVIGATOR_ID:
      if (ack->payload_size >= sizeof(NavigatorData))
      {
        memcpy(&rocketData.navigatorData, ack->payload, sizeof(NavigatorData));
      }
      break;
    case LIFT_TANK_ID:
      if (ack->payload_size >= sizeof(LiftTankData))
      {
        memcpy(&rocketData.liftTankData, ack->payload, sizeof(LiftTankData));
      }
      break;
    case LIFT_BOTTLE_ID:
      if (ack->payload_size >= sizeof(LiftBottleData))
      {
        memcpy(&rocketData.liftBottleData, ack->payload, sizeof(LiftBottleData));
      }
      break;
    case LIFT_THRUST_ID:
      if (ack->payload_size >= sizeof(LiftThrustData))
      {
        memcpy(&rocketData.liftThrustData, ack->payload, sizeof(LiftThrustData));
      }
      break;
    default:
      break;
    }
    // Release mutex after accessing rocketData
    xSemaphoreGive(rocketDataMutex);
  }
}

void vDataPollingTask(void *pvParameters)
{
  int32_t lastPollTime = 0;
  //Serial.println("[TASK] DataPolling task started");
  while (true)
  {
    //Serial.println("[TASK] Running: DataPolling");
    // Poll rs bus - check if the top of acknowledgments queue is a status ack
    packet_t ack;
    if (xQueuePeek(AcknowledgementQueue, &ack, 0) == pdTRUE)
    {
      // Check if it's a status acknowledgment
      if (ack.cmd == CMD_ACK && ack.payload[0] == CMD_STATUS)
      {
        // Remove from queue
        xQueueReceive(AcknowledgementQueue, &ack, 0);

        // Process it and update system data
        processStatusAck(&ack);
      }
    }

    if ((millis() - lastPollTime) >= STATUS_POLL_INTERVAL_MS)
    {
      // If not waiting for response, poll next slave
      pollNextSlave();
      lastPollTime = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
