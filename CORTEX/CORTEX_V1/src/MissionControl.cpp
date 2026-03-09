#include "MissionControl.h"
#include "Communications.h"
#include "Commands.h"
#include "Queues.h"
#include "StateMachine.h"
#include "DataPolling.h"
#include "ValveRouting.h"
#include "Configs.h"
#include <Arduino.h>

extern QueueHandle_t EventQueue;
extern QueueHandle_t ManualCommandQueue;
extern SemaphoreHandle_t rocketDataMutex;
extern RocketData rocketData;

int processManualExecCommand(packet_t *packet)
{
  if (packet == nullptr || packet->payload_size < 1)
  {
    return -1;
  }

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
    if (packet->payload_size < 3)
    {
      return -1;
    }
    {
      ValveRoute route;
      if (getValveRoute((valve_t)packet->payload[1], &route) != 0)
      {
        return -1;
      }

      packet->sender_id = CORTEX_ID;
      packet->target_id = route.hydraId;
      packet->payload[1] = route.hydraValve;

      if (xQueueSend(ManualCommandQueue, packet, 0) != pdPASS)
      {
        return -1;
      }
    }
    break;
  default:
    return -1;
  }
  return 0;
}

int processFillCommand(packet_t *packet)
{
  if (packet == nullptr || packet->payload_size < 1)
  {
    return -1;
  }

  fill_command_t fill_command = (fill_command_t)packet->payload[0];
  switch (fill_command)
  {
  case CMD_FILL_N2: {
    // Send fill N2 event to EventQueue
    RocketEvent fillN2Event = EV_FILL_N2_CMD;
    xQueueSend(EventQueue, &fillN2Event, 0);
    break;
  }
  case CMD_PRE_PRESSURIZE: {
    // Send pre-pressurize event to EventQueue
    RocketEvent prePressurizeEvent = EV_PRE_PRESSURIZE_CMD;
    xQueueSend(EventQueue, &prePressurizeEvent, 0);
    break;
  }
  case CMD_FILL_OX: {
    // Send fill OX event to EventQueue
    RocketEvent fillOXEvent = EV_FILL_OX_CMD;
    xQueueSend(EventQueue, &fillOXEvent, 0);
    break;
  }
  case CMD_POST_PRESSURIZE: {
    // Send post-pressurize event to EventQueue
    RocketEvent postPressurizeEvent = EV_POST_PRESSURIZE_CMD;
    xQueueSend(EventQueue, &postPressurizeEvent, 0);
    break;
  }
  default:
    return -1; // Unknown fill command
  }
  return 0;
}

// Serialize rocketData into ack payload matching the Java MC parsing layout.
// Total payload: 36 bytes, all multi-byte fields big-endian (ByteBuffer default).
// Returns 0 on success, -1 if the mutex could not be taken.
#define WRITE_U16_BE(buf, i, val) do { \
  (buf)[(i)]     = ((uint16_t)(val) >> 8) & 0xFF; \
  (buf)[(i) + 1] = ((uint16_t)(val)     ) & 0xFF; \
} while(0)

static int buildStatusAckPayload(packet_t *ack)
{
  if (xSemaphoreTake(rocketDataMutex, pdMS_TO_TICKS(10)) != pdTRUE)
  {
    return -1;
  }

  uint8_t *p = ack->payload;
  uint8_t idx = 0;

  // [0]     ack cmd
  p[idx++] = CMD_STATUS;

  // [1]     rocket state (1 byte)
  p[idx++] = (uint8_t)rocketData.cortexData.rocket_state;

  // [2-3]   valve flags (big-endian uint16)
  uint16_t flags = 0;
  flags |= (uint16_t)(rocketData.hydraUFData.valve_pressurizing & 0x01) << 0;
  flags |= (uint16_t)(rocketData.hydraUFData.valve_vent         & 0x01) << 1;
  flags |= (uint16_t)(rocketData.hydraLFData.valve_abort        & 0x01) << 2;
  flags |= (uint16_t)(rocketData.hydraLFData.valve_main         & 0x01) << 3;
  flags |= (uint16_t)(rocketData.hydraFSData.valve_ox_fill      & 0x01) << 4; // n2o fill
  flags |= (uint16_t)(rocketData.hydraFSData.valve_ox_purge     & 0x01) << 5; // n2o purge
  flags |= (uint16_t)(rocketData.hydraFSData.valve_n2_fill      & 0x01) << 6; // n2 fill
  flags |= (uint16_t)(rocketData.hydraFSData.valve_n2_purge     & 0x01) << 7; // n2 purge
  // bits 8-10: ematches — not yet implemented, leave 0
  flags |= (uint16_t)(rocketData.hydraFSData.valve_n2o_quick_dc & 0x01) << 11;
  flags |= (uint16_t)(rocketData.hydraFSData.valve_n2_quick_dc  & 0x01) << 12;
  WRITE_U16_BE(p, idx, flags); idx += 2;

  // [4-5]   hydra_lf tank_bottom_pressure
  WRITE_U16_BE(p, idx, rocketData.hydraLFData.tank_bottom_pressure); idx += 2;
  // [6-7]   hydra_lf chamber_pressure
  WRITE_U16_BE(p, idx, rocketData.hydraLFData.chamber_pressure); idx += 2;
  // [8-9]   hydra_fs n2o (ox) pressure
  WRITE_U16_BE(p, idx, rocketData.hydraFSData.ox_pressure); idx += 2;
  // [10-11] hydra_fs n2 pressure
  WRITE_U16_BE(p, idx, rocketData.hydraFSData.n2_pressure); idx += 2;
  // [12-13] quick_dc pressure — not yet implemented
  WRITE_U16_BE(p, idx, 0); idx += 2;

  // [14-15] hydra_uf probe_temperature_1
  WRITE_U16_BE(p, idx, rocketData.hydraUFData.probe_temperature_1); idx += 2;
  // [16-17] hydra_uf probe_temperature_2
  WRITE_U16_BE(p, idx, rocketData.hydraUFData.probe_temperature_2); idx += 2;
  // [18-19] hydra_uf probe_temperature_3
  WRITE_U16_BE(p, idx, rocketData.hydraUFData.probe_temperature_3); idx += 2;
  // [20-21] hydra_lf probe_temperature_1
  WRITE_U16_BE(p, idx, rocketData.hydraLFData.probe_temperature_1); idx += 2;
  // [22-23] hydra_lf probe_temperature_2
  WRITE_U16_BE(p, idx, rocketData.hydraLFData.probe_temperature_2); idx += 2;
  // [24-25] hydra_lf chamber_temperature
  WRITE_U16_BE(p, idx, rocketData.hydraLFData.chamber_temperature); idx += 2;
  // [26-27] hydra_fs n2o (ox) temperature
  WRITE_U16_BE(p, idx, rocketData.hydraFSData.ox_temperature); idx += 2;

  // [28-29] lift fill-station n2o load cell (bottle weight)
  WRITE_U16_BE(p, idx, (uint16_t)rocketData.liftBottleData.bottle_weight); idx += 2;
  // [30-31] lift thrust load cell 1
  WRITE_U16_BE(p, idx, (uint16_t)rocketData.liftThrustData.thrust_1); idx += 2;
  // [32-33] lift thrust load cell 2
  WRITE_U16_BE(p, idx, (uint16_t)rocketData.liftThrustData.thrust_2); idx += 2;
  // [34-35] lift thrust load cell 3
  WRITE_U16_BE(p, idx, (uint16_t)rocketData.liftThrustData.thrust_3); idx += 2;

  ack->payload_size = idx; // 36 bytes
  xSemaphoreGive(rocketDataMutex);
  return 0;
}

#undef WRITE_U16_BE

int processMissionControlCommand(packet_t *packet, packet_t *ack)
{
  switch (packet->cmd)
  {
  case CMD_STATUS:
    if (buildStatusAckPayload(ack) != 0)
    {
      return -1; // Failed to take mutex
    }
    ack->cmd = CMD_ACK;
    if (write_packet(ack, UART_INTERFACE) != 0)
    {
      Serial1.println("Failed to send status acknowledgment");
      return -1;
    }
    return 0;
  case CMD_STOP:
    {
      RocketEvent event = EV_STOP_CMD;
      if (xQueueSend(EventQueue, &event, 0) != pdPASS)
      {
        ack->cmd = CMD_NACK;
        write_packet(ack, UART_INTERFACE); 
        return -1; // Failed to send stop event
      }
      ack->cmd = CMD_ACK;
      write_packet(ack, UART_INTERFACE);
    }
    break;
  case CMD_ABORT:
    {
      RocketEvent event = EV_ABORT_CMD;
      if (xQueueSend(EventQueue, &event, 0) != pdPASS)
      {
        ack->cmd = CMD_NACK;
        write_packet(ack, UART_INTERFACE); 
        return -1; // Failed to send abort event
      }
      ack->cmd = CMD_ACK;
      write_packet(ack, UART_INTERFACE);
    }
    break;
  case CMD_FILL_EXEC:
    if (packet->payload_size >= 1)
    {
      int result = processFillCommand(packet);
      if (result != 0)
      {
        ack->cmd = CMD_NACK;
        write_packet(ack, UART_INTERFACE);
        return -1;
      }
      ack->cmd = CMD_ACK;
      write_packet(ack, UART_INTERFACE);
    }
    break;
  case CMD_ARM:
    {
      RocketEvent event = EV_ARM_CMD;
      if (xQueueSend(EventQueue, &event, 0) != pdPASS) {
        ack->cmd = CMD_NACK;
        write_packet(ack, UART_INTERFACE);
        return -1; // Failed to send arm event
      }
      ack->cmd = CMD_ACK;
      write_packet(ack, UART_INTERFACE);
    }
    break;
  case CMD_FIRE:
    {
      RocketEvent event = EV_FIRE_CMD;
      if (xQueueSend(EventQueue, &event, 0) != pdPASS) {
        ack->cmd = CMD_NACK;
        write_packet(ack, UART_INTERFACE);
        return -1; // Failed to send fire event
      }
      ack->cmd = CMD_ACK;
      write_packet(ack, UART_INTERFACE);
    }
    break;
  case CMD_MANUAL_EXEC:
    if (packet->payload_size >= 1)
    {
      int result = processManualExecCommand(packet);
      if (result != 0)
      {
        ack->cmd = CMD_NACK;
        write_packet(ack, UART_INTERFACE);
        return -1;
      }
      ack->cmd = CMD_ACK;
      ack->payload[0] = CMD_MANUAL_EXEC;
      ack->payload_size = 1;
      write_packet(ack, UART_INTERFACE);
    }
    break;
  default:
    return -1; // Unknown command
  }
  return 0;
}

void vMissionControlTask(void *pvParameters)
{
  //Serial1.println("[TASK] MissionControl task started");
  while (true)
  {
    bool hadPacket = false;
    //Serial1.println("[TASK] Running: MissionControl");
    // Read from mission control (e.g. LoRa, RS485)
    int error;
    packet_t *receivedPacket = read_packet(&error, UART_INTERFACE);
    if (receivedPacket != NULL && error == CMD_READ_OK)
    {
      hadPacket = true;
      static packet_t ack;
      uint8_t ack_payload[MAX_PAYLOAD_SIZE] = {0};
      ack_payload[0] = receivedPacket->cmd;
      ack.payload_size = 1;
      if (create_packet(&ack, CORTEX_ID, receivedPacket->sender_id, CMD_NACK, ack_payload, sizeof(ack_payload)) != 0)
      {
        vTaskDelay(pdMS_TO_TICKS(5));
        continue;
      }
      processMissionControlCommand(receivedPacket, &ack);
    }
    else if (receivedPacket != NULL) {
      Serial1.print("Error reading packet from Mission Control: ");
      Serial1.println(error);
    }
    if (!hadPacket) {
      vTaskDelay(pdMS_TO_TICKS(5));
    } else {
      taskYIELD();
    }
  }
}