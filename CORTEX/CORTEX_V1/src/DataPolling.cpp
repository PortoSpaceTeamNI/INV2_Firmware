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
extern volatile bool waitingForResponse;

RocketData rocketData;

static uint8_t expectedStatusSenderId = 0;

static uint8_t expectedStatusPayloadSizeForSender(uint8_t senderId)
{
  switch (senderId)
  {
  case HYDRA_UF_ID:
    return sizeof(HydraUFData);
  case HYDRA_LF_ID:
    return sizeof(HydraLFData);
  case HYDRA_FS_ID:
    return sizeof(HydraFSData);
  case NAVIGATOR_ID:
    return sizeof(NavigatorData);
  case LIFT_TANK_ID:
    return sizeof(LiftTankData);
  case LIFT_BOTTLE_ID:
    return sizeof(LiftBottleData);
  case LIFT_THRUST_ID:
    return sizeof(LiftThrustData);
  default:
    return 0;
  }
}

static bool isExpectedStatusAck(const packet_t *ack)
{
  if (ack == nullptr || ack->cmd != CMD_ACK || ack->payload_size < 1)
  {
    return false;
  }

  if (ack->payload[0] != CMD_STATUS)
  {
    return false;
  }

  if (ack->sender_id != expectedStatusSenderId)
  {
    return false;
  }

  const uint8_t expectedPayloadSize = expectedStatusPayloadSizeForSender(ack->sender_id);
  if (expectedPayloadSize == 0)
  {
    return false;
  }

  return ack->payload_size == (uint8_t)(expectedPayloadSize + 1);
}

int requestStatus(uint8_t targetID)
{
  packet_t packet;
  uint8_t payload[1] = {0}; // No payload for status request
  create_packet(&packet, CORTEX_ID, targetID, CMD_STATUS, payload, 0);

  // Use timeout when sending to queue to avoid blocking indefinitely
  if (xQueueSend(StatusQueue, &packet, pdMS_TO_TICKS(10)) == pdPASS)
  {
    expectedStatusSenderId = targetID;
    return 0;
  }
  return -1; // Failed to send
}

int pollNextSlave()
{
  static uint8_t nextSlaveId = HYDRA_UF_ID; // Start polling from the first slave

  if (requestStatus(nextSlaveId) != 0)
  {
    Serial1.print("Failed to request status from slave ID: ");
    Serial1.println(nextSlaveId);
  } else {
    //Serial1.print("Requested status from slave ID: ");
    //Serial1.println(nextSlaveId);
  }

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
  if (!isExpectedStatusAck(ack))
  {
    return;
  }

  // First payload byte is the acknowledged command (CMD_STATUS)
  const uint8_t *statusPayload = ack->payload + 1;

  // Process status acknowledgment based on sender ID
  // TODO: Maybe will change if bools are sent as bitfields

  // Take mutex before accessing rocketData
  if (xSemaphoreTake(rocketDataMutex, portMAX_DELAY) == pdTRUE)
  {
    switch (ack->sender_id)
    {
    case HYDRA_UF_ID:
      memcpy(&rocketData.hydraUFData, statusPayload, sizeof(HydraUFData));
      break;
    case HYDRA_LF_ID:
      memcpy(&rocketData.hydraLFData, statusPayload, sizeof(HydraLFData));
      break;
    case HYDRA_FS_ID:
      memcpy(&rocketData.hydraFSData, statusPayload, sizeof(HydraFSData));
      break;
    case NAVIGATOR_ID:
      memcpy(&rocketData.navigatorData, statusPayload, sizeof(NavigatorData));
      break;
    case LIFT_TANK_ID:
      memcpy(&rocketData.liftTankData, statusPayload, sizeof(LiftTankData));
      break;
    case LIFT_BOTTLE_ID:
      memcpy(&rocketData.liftBottleData, statusPayload, sizeof(LiftBottleData));
      break;
    case LIFT_THRUST_ID:
      memcpy(&rocketData.liftThrustData, statusPayload, sizeof(LiftThrustData));
      break;
    default:
      break;
    }
    expectedStatusSenderId = 0;
    // Release mutex after accessing rocketData
    xSemaphoreGive(rocketDataMutex);
  }
}

void vDataPollingTask(void *pvParameters)
{
  uint32_t lastPollTime = 0;
  //Serial1.println("[TASK] DataPolling task started");
  while (true)
  {
    //Serial1.println("[TASK] Running: DataPolling");
    // Poll rs bus acknowledgments
    static packet_t ack;
    if (xQueueReceive(AcknowledgementQueue, &ack, 0) == pdTRUE)
    {
      processStatusAck(&ack);
    }

    if ((millis() - lastPollTime) >= STATUS_POLL_INTERVAL_MS)
    {
      // If not waiting for response, poll next slave
      if (!waitingForResponse)
      {
        pollNextSlave();
        lastPollTime = millis();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
