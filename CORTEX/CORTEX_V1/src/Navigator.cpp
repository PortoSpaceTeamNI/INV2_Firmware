#include "Navigator.h"
#include "DataPolling.h"
#include "Communications.h"
#include "Commands.h"
#include "Configs.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <semphr.h>

extern SemaphoreHandle_t rocketDataMutex;
extern RocketData rocketData;

static bool isNavigatorStatusAck(const packet_t *ack)
{
    if (ack == nullptr || ack->cmd != CMD_ACK || ack->payload_size < 1) return false;

    if (ack->sender_id != NAVIGATOR_ID) return false;

    if (ack->target_id != CORTEX_ID && ack->target_id != BROADCAST_ID) return false;

    if (ack->payload[0] != CMD_STATUS) return false;

    return ack->payload_size == (uint8_t)(1 + sizeof(NavigatorData));
}

static int requestNavigatorStatus()
{
  packet_t packet;
  if (create_packet(&packet, CORTEX_ID, NAVIGATOR_ID, CMD_STATUS, NULL, 0) != 0) return -1;

  return write_packet(&packet, UART_INTERFACE);
}

static void processNavigatorAck(const packet_t *ack)
{
  const uint8_t *statusPayload = ack->payload + 1;
  if (xSemaphoreTake(rocketDataMutex, portMAX_DELAY) == pdTRUE)
  {
    memcpy(&rocketData.navigatorData, statusPayload, sizeof(NavigatorData));
    xSemaphoreGive(rocketDataMutex);
  }
}

void vNavigatorTask(void *pvParameters)
{
    uint32_t lastPollTime = 0;
    uint32_t waitStart = 0;
    bool waitingNavigator = false;

    while (true)
    {
    int error = CMD_READ_NO_CMD;
    packet_t *rx = read_packet(&error, UART_INTERFACE);
    if (rx != nullptr && error == CMD_READ_OK && isNavigatorStatusAck(rx))
    {
    processNavigatorAck(rx);
    waitingNavigator = false;
    }

    if (waitingNavigator && (millis() - waitStart) > RS485_RESPONSE_TIMEOUT_MS)
    {
    waitingNavigator = false;
    }

    if (!waitingNavigator && (millis() - lastPollTime) >= STATUS_POLL_INTERVAL_MS)
    {
    if (requestNavigatorStatus() == 0)
    {
        waitingNavigator = true;
        waitStart = millis();
        lastPollTime = millis();
    }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    }
}