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

typedef union {
  uint8_t raw;
  struct {
    uint8_t v_quick_dc_1 : 1;
    uint8_t v_quick_dc_2 : 1;
    uint8_t v_controlled_1 : 1;
    uint8_t v_controlled_2 : 1;
    uint8_t v_controlled_3 : 1;
    uint8_t v_steel_ball_1 : 1;
    uint8_t v_steel_ball_2 : 1;
    uint8_t v_servo : 1;
  };
} HydraValveStatesRaw;

enum HydraValveStateBit : uint8_t {
  HYDRA_BIT_QUICK_DC_1 = 0,
  HYDRA_BIT_QUICK_DC_2,
  HYDRA_BIT_CONTROLLED_1,
  HYDRA_BIT_CONTROLLED_2,
  HYDRA_BIT_CONTROLLED_3,
  HYDRA_BIT_STEEL_BALL_1,
  HYDRA_BIT_STEEL_BALL_2,
  HYDRA_BIT_SERVO,
};

struct HydraStatusRaw {
  int16_t thermo1;
  int16_t thermo2;
  int16_t thermo3;
  int16_t pressure1;
  int16_t pressure2;
  int16_t pressure3;
  HydraValveStatesRaw valve_states;
  bool cam_enable;
};

static uint8_t hydraValveStateBit(const HydraValveStatesRaw &states, uint8_t bitIndex)
{
  return (states.raw >> bitIndex) & 0x01;
}

static void mapHydraRawToRocketData(uint8_t hydraId, const HydraStatusRaw &raw, RocketData *data)
{
  if (data == nullptr)
  {
    return;
  }

  switch (hydraId)
  {
  case HYDRA_UF_ID:
    data->hydraUFData.valve_vent = hydraValveStateBit(raw.valve_states, HYDRA_BIT_CONTROLLED_1);
    data->hydraUFData.tank_top_pressure = (uint16_t)raw.pressure1;
    data->hydraUFData.probe_temperature_1 = (uint16_t)raw.thermo1;
    data->hydraUFData.probe_temperature_2 = (uint16_t)raw.thermo2;
    data->hydraUFData.probe_temperature_3 = (uint16_t)raw.thermo3;
    break;
  case HYDRA_LF_ID:
    data->hydraLFData.valve_abort = hydraValveStateBit(raw.valve_states, HYDRA_BIT_CONTROLLED_1);
    data->hydraLFData.valve_main = hydraValveStateBit(raw.valve_states, HYDRA_BIT_CONTROLLED_2);
    data->hydraUFData.valve_pressurizing = hydraValveStateBit(raw.valve_states, HYDRA_BIT_CONTROLLED_3);
    data->hydraLFData.tank_bottom_pressure = (uint16_t)raw.pressure2;
    data->hydraLFData.chamber_pressure = (uint16_t)raw.pressure3;
    data->hydraLFData.probe_temperature_1 = (uint16_t)raw.thermo2;
    data->hydraLFData.probe_temperature_2 = (uint16_t)raw.thermo1;
    data->hydraLFData.chamber_temperature = (uint16_t)raw.thermo3;
    break;
  case HYDRA_FS_ID:
    data->hydraFSData.valve_ox_fill = hydraValveStateBit(raw.valve_states, HYDRA_BIT_SERVO);
    data->hydraFSData.valve_ox_purge = hydraValveStateBit(raw.valve_states, HYDRA_BIT_CONTROLLED_3);
    data->hydraFSData.valve_n2_fill = hydraValveStateBit(raw.valve_states, HYDRA_BIT_STEEL_BALL_2);
    data->hydraFSData.valve_n2_purge = hydraValveStateBit(raw.valve_states, HYDRA_BIT_STEEL_BALL_1);
    data->hydraFSData.valve_n2o_quick_dc = hydraValveStateBit(raw.valve_states, HYDRA_BIT_QUICK_DC_1);
    data->hydraFSData.valve_n2_quick_dc = hydraValveStateBit(raw.valve_states, HYDRA_BIT_QUICK_DC_2);
    data->hydraFSData.ox_pressure = (uint16_t)raw.pressure1;
    data->hydraFSData.n2_pressure = (uint16_t)raw.pressure3;
    data->hydraFSData.n2_temperature = (uint16_t)raw.thermo1;
    data->hydraFSData.ox_temperature = (uint16_t)raw.thermo2;
    break;
  default:
    break;
  }
}

static bool isRawHydraPayloadSize(uint8_t hydraId, uint8_t payloadSize)
{
  if (hydraId == HYDRA_UF_ID || hydraId == HYDRA_LF_ID || hydraId == HYDRA_FS_ID)
  {
    return payloadSize == sizeof(HydraStatusRaw);
  }
  return false;
}

static uint8_t expectedStatusPayloadSizeForSender(uint8_t senderId)
{
  switch (senderId)
  {
  case HYDRA_UF_ID:
  case HYDRA_LF_ID:
  case HYDRA_FS_ID:
    return sizeof(HydraStatusRaw);
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

  const uint8_t actualPayloadSize = (uint8_t)(ack->payload_size - 1);
  if (ack->sender_id == HYDRA_UF_ID || ack->sender_id == HYDRA_LF_ID || ack->sender_id == HYDRA_FS_ID)
  {
    return isRawHydraPayloadSize(ack->sender_id, actualPayloadSize);
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

  for (size_t i = 0; i < ack->payload_size - 1; i++)
  {
    Serial1.write("0x");
    Serial1.print(statusPayload[i], HEX);
    Serial1.write(" ");
  }
  Serial1.println();

  // Process status acknowledgment based on sender ID
  // TODO: Maybe will change if bools are sent as bitfields

  // Take mutex before accessing rocketData
  if (xSemaphoreTake(rocketDataMutex, portMAX_DELAY) == pdTRUE)
  {
    switch (ack->sender_id)
    {
    case HYDRA_UF_ID:
      {
        HydraStatusRaw hydraRaw;
        memcpy(&hydraRaw, statusPayload, sizeof(HydraStatusRaw));
        mapHydraRawToRocketData(HYDRA_UF_ID, hydraRaw, &rocketData);
      }
      break;
    case HYDRA_LF_ID:
      {
        HydraStatusRaw hydraRaw;
        memcpy(&hydraRaw, statusPayload, sizeof(HydraStatusRaw));
        mapHydraRawToRocketData(HYDRA_LF_ID, hydraRaw, &rocketData);
      }
      break;
    case HYDRA_FS_ID:
      {
        HydraStatusRaw hydraRaw;
        memcpy(&hydraRaw, statusPayload, sizeof(HydraStatusRaw));
        mapHydraRawToRocketData(HYDRA_FS_ID, hydraRaw, &rocketData);
      }
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
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
