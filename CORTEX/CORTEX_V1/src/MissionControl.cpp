#include "MissionControl.h"
#include "Communications.h"
#include "Commands.h"
#include "Queues.h"
#include "StateMachine.h"
#include "DataPolling.h"
#include "Configs.h"
#include <Arduino.h>

extern QueueHandle_t EventQueue;
extern QueueHandle_t ManualCommandQueue;
extern SemaphoreHandle_t rocketDataMutex;
extern RocketData rocketData;

int processManualExecCommand(packet_t *packet)
{
  manual_command_t manual_command = (manual_command_t)packet->payload[0];
  switch (manual_command)
  {
  case CMD_MANUAL_SD_LOG_START:
    // TODO: Start SD logging
    break;
  case CMD_MANUAL_SD_LOG_STOP:
    // TODO: Stop SD logging
    break;
  case CMD_MANUAL_SD_STATUS:
    // TODO: Return the logging data (file count, file ids, space left, etc).
    break;
  case CMD_MANUAL_VALVE_STATE:
    // Forward to RS485 task to send to valve controller
    xQueueSend(ManualCommandQueue, packet, 0);
    break;
  default:
    return -1;
  }
  return 0;
}

int processFillCommand(packet_t *packet)
{
  fill_command_t fill_command = (fill_command_t)packet->payload[0];
  switch (fill_command)
  {
  case CMD_FILL_N2:
    // Send fill N2 event to EventQueue
    break;
    break;
  case CMD_PRE_PRESSURIZE:
    // Send pre-pressurize event to EventQueue
    break;
  case CMD_FILL_OX:
    // Send fill OX event to EventQueue
    break;
  case CMD_POST_PRESSURIZE:
    // Send post-pressurize event to EventQueue
    break;
  default:
    return -1; // Unknown fill command
  }
  return 0;
}

int processMissionControlCommand(packet_t *packet, packet_t *ack)
{
  switch (packet->cmd)
  {
  case CMD_STATUS:
    // Send back system data in the acknowledgment packet
    if (xSemaphoreTake(rocketDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      // Fill ack payload with rocketData
      memcpy(ack->payload + 1, &rocketData, sizeof(rocketData)); // +1 because ack->payload[0] is the CMD acknowledged
      ack->payload_size += sizeof(rocketData);
      xSemaphoreGive(rocketDataMutex);

      // Change command to ACK and send back
      ack->cmd = CMD_ACK;
      return 0;
    }
    else {
      return -1; // Failed to take mutex
    }
    break;
  case CMD_STOP:
    {
      RocketEvent event = EV_STOP_CMD;
      if (xQueueSend(EventQueue, &event, 0) != pdPASS)
      {
        return -1; // Failed to send stop event
      }
    }
    break;
  case CMD_ABORT:
    {
      RocketEvent event = EV_ABORT_CMD;
      if (xQueueSend(EventQueue, &event, 0) != pdPASS)
      {
        return -1; // Failed to send abort event
      }
    }
    break;
  case CMD_FILL_EXEC:
    if (packet->payload_size >= 1)
    {
      return processFillCommand(packet);
    }
    break;
  case CMD_ARM:
    {
      RocketEvent event = EV_ARM_CMD;
      if (xQueueSend(EventQueue, &event, 0) != pdPASS) {
        return -1; // Failed to send arm event
      }
    }
    break;
  case CMD_FIRE:
    {
      RocketEvent event = EV_FIRE_CMD;
      if (xQueueSend(EventQueue, &event, 0) != pdPASS) {
        return -1; // Failed to send fire event
      }
    }
    break;
  case CMD_MANUAL_EXEC:
    if (packet->payload_size >= 1)
    {
      return processManualExecCommand(packet);
    }
    break;
  default:
    return -1; // Unknown command
  }
  return 0;
}

void vMissionControlTask(void *pvParameters)
{
  //Serial.println("[TASK] MissionControl task started");
  while (true)
  {
    //Serial.println("[TASK] Running: MissionControl");
    // Read from mission control (e.g. LoRa, RS485)
    int error;
    packet_t *receivedPacket = read_packet(&error, LORA_INTERFACE);
    if (receivedPacket != NULL && error == CMD_READ_OK)
    {
      packet_t ack;
      uint8_t ack_payload[1] = {0};
      ack.payload[0] = receivedPacket->cmd;
      ack.payload_size = 1;
      create_packet(&ack, CORTEX_ID, receivedPacket->sender_id, CMD_NACK, NULL, 0); // NACK by default
      processMissionControlCommand(receivedPacket, &ack);
    }

    // Send command to proper queue (EventQueue for state machine events, ManualCommandQueue for manual commands)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}