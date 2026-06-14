#include "StateMachine.h"
#include "DataPolling.h"
#include "ValveRouting.h"
#include "Communications.h"
#include "Commands.h"
#include "Configs.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <semphr.h>

extern QueueHandle_t EventQueue;
extern QueueHandle_t ManualCommandQueue;

extern RocketData rocketData;
extern SemaphoreHandle_t rocketDataMutex;
extern SemaphoreHandle_t stateMachineConfigsMutex;

StateMachineConfigs stateMachineConfigs; // TODO: Load from flash or set via command

static void sendValveCmd(valve_t valve, uint8_t state)
{
    ValveRoute route;
    if (getValveRoute(valve, &route) != 0) return;
    packet_t pkt;
    uint8_t pl[3] = { CMD_MANUAL_VALVE_STATE, route.hydraValve, state };
    create_packet(&pkt, CORTEX_ID, route.hydraId, CMD_MANUAL_EXEC, pl, 3);
    xQueueSend(ManualCommandQueue, &pkt, 0);
}

static void closeAllValves()
{
    for (int v = 0; v < (int)valve_count; v++) {
        sendValveCmd((valve_t)v, VALVE_CLOSED);
    }
}

RocketState updateState(RocketState currentState, RocketEvent event)
{
  return rc_update_state(currentState, event);
}

int performStateRun(RocketState currentState, StateMachineConfigs *configs, RocketData *data)
{
  uint32_t elapsed_ms = millis() - data->cortexData.time_state_start_ms;
  switch (currentState)
  {
  case IDLE:
    // Idle, as the name implies, does nothing
    break;
  case ABORT:
    // On run does nothing
    break;
  case FILL_N2:
    // For now does nothing
    break;
  case PRE_PRESSURIZE: // Close pressurizing valve once target pressure is reached
    if (data->hydraUFData.tank_top_pressure >= configs->pre_pressurizing_pressure)
    {
      if (data->hydraUFData.valve_pressurizing == VALVE_OPEN)
      {
        sendValveCmd(VALVE_PRESSURIZING, VALVE_CLOSED);
      }
    }
    break;
  case FILL_OX: // Vent control; close fill valve if temperature drops too low
    if (data->hydraLFData.probe_temperature_2 < configs->ox_fill_temperature)
    {
      if (data->hydraFSData.valve_ox_fill == VALVE_OPEN)
      {
        sendValveCmd(VALVE_N2O_FILL, VALVE_CLOSED);
      }
    }
    else
    {
      if (data->hydraUFData.tank_top_pressure >= configs->ox_fill_pressure)
      {
        if (data->hydraUFData.valve_vent == VALVE_CLOSED)
          sendValveCmd(VALVE_VENT, VALVE_OPEN);
      }
      else
      {
        if (data->hydraUFData.valve_vent == VALVE_OPEN)
          sendValveCmd(VALVE_VENT, VALVE_CLOSED);
      }
    }
    break;
  case POST_PRESSURIZE: // Bang-bang control on pressurizing valve to maintain pressure
    if (data->hydraUFData.tank_top_pressure >= configs->post_pressurizing_pressure)
    {
      if (data->hydraUFData.valve_pressurizing == VALVE_OPEN)
        sendValveCmd(VALVE_PRESSURIZING, VALVE_CLOSED);
    }
    else
    {
      if (data->hydraUFData.valve_pressurizing == VALVE_CLOSED)
        sendValveCmd(VALVE_PRESSURIZING, VALVE_OPEN);
    }
    break;
  case ARMED:
    if (elapsed_ms >= configs->arming_timeout_ms)
    {
      // Go to IDLE state
      RocketEvent stopEvent = EV_STOP_CMD;
      xQueueSend(EventQueue, &stopEvent, portMAX_DELAY);
    }
    break;
  case IGNITION:
    if (elapsed_ms >= configs->ignition_timeout_ms)
    {
      // Go to IDLE state
      RocketEvent stopEvent = EV_STOP_CMD;
      xQueueSend(EventQueue, &stopEvent, portMAX_DELAY);
    }
    break;
  case LAUNCH:
    if (data->navigatorData.acceleration >= configs->launch_acceleration 
      || data->navigatorData.velocity >= configs->launch_velocity)
    {
      // Go to BOOST state
      RocketEvent takeoffEvent = EV_TAKEOFF;
      xQueueSend(EventQueue, &takeoffEvent, portMAX_DELAY);
    }
    break;
  case BOOST:
    if (data->navigatorData.acceleration < configs->boost_acceleration)
    {
      // Go to COAST state
      RocketEvent boostEndEvent = EV_BOOST_END;
      xQueueSend(EventQueue, &boostEndEvent, portMAX_DELAY);
    }
    break;
  case COAST:
    if (data->navigatorData.velocity < configs->apogee_velocity)
    {
      // Go to DROGUE_DESCENT state
      RocketEvent apogeeEvent = EV_APOGEE;
      xQueueSend(EventQueue, &apogeeEvent, portMAX_DELAY);
    }
    break;
  case DROGUE_DESCENT:
    if (data->navigatorData.altitude < configs->main_deployment_altitude)
    {
      // Go to MAIN_DESCENT state
      RocketEvent mainAltitudeEvent = EV_MAIN_ALTITUDE;
      xQueueSend(EventQueue, &mainAltitudeEvent, portMAX_DELAY);
    }
    break;
  case MAIN_DESCENT:
    if (data->navigatorData.velocity < configs->touchdown_velocity 
      && data->navigatorData.acceleration < configs->touchdown_acceleration 
      && data->navigatorData.altitude < configs->touchdown_altitude)
    {
      // Go to TOUCHDOWN state
      RocketEvent touchdownEvent = EV_TOUCHDOWN;
      xQueueSend(EventQueue, &touchdownEvent, portMAX_DELAY);
    }
    break;
  case TOUCHDOWN:
    break;
  default:
    return -1; // Invalid state
  }
  return 0; // State executed successfully
}

int performStateEntry(RocketState currentState, StateMachineConfigs *configs, RocketData *data)
{
  data->cortexData.time_state_start_ms = millis(); // Record state entry timestamp
  switch (currentState)
  {
  case IDLE:
    closeAllValves();
    break;
  case ABORT:
    closeAllValves();
    sendValveCmd(VALVE_ABORT, VALVE_OPEN);
    break;
  case FILL_N2:
    closeAllValves();
    sendValveCmd(VALVE_N2_FILL, VALVE_OPEN);
    break;
  case PRE_PRESSURIZE:
    if (data->hydraUFData.tank_top_pressure < configs->pre_pressurizing_pressure) {
      sendValveCmd(VALVE_PRESSURIZING, VALVE_OPEN);
    }
    break;
  case FILL_OX:
    if (data->hydraLFData.probe_temperature_2 >= configs->ox_fill_temperature) {
      sendValveCmd(VALVE_N2O_FILL, VALVE_OPEN);
    }
    break;
  case POST_PRESSURIZE:
    if (data->hydraUFData.tank_top_pressure < configs->post_pressurizing_pressure) {
      sendValveCmd(VALVE_PRESSURIZING, VALVE_OPEN);
    }
    break;
  case ARMED:
    break;
  case IGNITION:
    break;
  case LAUNCH:
    sendValveCmd(VALVE_MAIN, VALVE_OPEN);
    break;
  case BOOST:
    break;
  case COAST:
    sendValveCmd(VALVE_MAIN, VALVE_CLOSED);
    break;
  case DROGUE_DESCENT:
    // TODO: trigger drogue ematch
    break;
  case MAIN_DESCENT:
    // TODO: trigger main ematch
    break;
  case TOUCHDOWN:
    break;
  default:
    return -1; // Invalid state
  }
  return 0; // State executed successfully
}

int performStateExit(RocketState state, StateMachineConfigs *configs, RocketData *data)
{
  // Exit actions can be added here in the future
  switch (state)
  {
  default:
    break;
  }
  return 0;
}

void vStateMachineTask(void *pvParameters)
{
  static RocketState currentState = IDLE;
  RocketEvent receivedEvent;
  const TickType_t runPeriodTicks = pdMS_TO_TICKS(10);
  
  //Serial1.println("[TASK] StateMachine task started");
  while (true)
  {
    //Serial1.println("[TASK] Running: StateMachine");
    // Receive events from EventQueue
    if (xQueueReceive(EventQueue, &receivedEvent, runPeriodTicks) == pdTRUE)
    {
      // Process state machine transitions
      RocketState newState = updateState(currentState, receivedEvent);
      
      // If state has changed, perform exit actions for old state and entry actions for new state
      if (newState != currentState)
      {
        // Take both mutexes to protect access to stateMachineConfigs and rocketData
        if (xSemaphoreTake(stateMachineConfigsMutex, portMAX_DELAY) == pdTRUE) {
          if (xSemaphoreTake(rocketDataMutex, portMAX_DELAY) == pdTRUE) {
            performStateExit(currentState, &stateMachineConfigs, &rocketData);
            performStateEntry(newState, &stateMachineConfigs, &rocketData);
            rocketData.cortexData.rocket_state = newState;
            currentState = newState;
            xSemaphoreGive(rocketDataMutex);
          }
          xSemaphoreGive(stateMachineConfigsMutex);
        }
      } 
      
      // Perform run actions for current state
      // Take both mutexes to protect access to stateMachineConfigs and rocketData
      if (xSemaphoreTake(stateMachineConfigsMutex, portMAX_DELAY) == pdTRUE) {
        if (xSemaphoreTake(rocketDataMutex, portMAX_DELAY) == pdTRUE) {
          performStateRun(currentState, &stateMachineConfigs, &rocketData);
          xSemaphoreGive(rocketDataMutex);
        }
        xSemaphoreGive(stateMachineConfigsMutex);
      }
    } else {
      if (xSemaphoreTake(stateMachineConfigsMutex, portMAX_DELAY) == pdTRUE) {
        if (xSemaphoreTake(rocketDataMutex, portMAX_DELAY) == pdTRUE) {
          performStateRun(currentState, &stateMachineConfigs, &rocketData);
          xSemaphoreGive(rocketDataMutex);
        }
        xSemaphoreGive(stateMachineConfigsMutex);
      }
    }
  }
}
