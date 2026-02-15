#include "StateMachine.h"
#include "DataPolling.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <semphr.h>

extern QueueHandle_t EventQueue;
extern QueueHandle_t ManualCommandQueue;

extern RocketData rocketData;
extern SemaphoreHandle_t rocketDataMutex;
extern SemaphoreHandle_t stateMachineConfigsMutex;

StateMachineConfigs stateMachineConfigs; // TODO: Load from flash or set via command

RocketState updateState(RocketState currentState, RocketEvent event)
{
  if (event == EV_ABORT_CMD)
    return ABORT;
  else if (event == EV_STOP_CMD)
    return IDLE;
  switch (currentState)
  {
  case IDLE:
    if (event == EV_FILL_N2_CMD)
      return FILL_N2;
    else if (event == EV_PRE_PRESSURIZE_CMD)
      return PRE_PRESSURIZE;
    else if (event == EV_FILL_OX_CMD)
      return FILL_OX;
    else if (event == EV_POST_PRESSURIZE_CMD)
      return POST_PRESSURIZE;
    else if (event == EV_ARM_CMD)
      return ARMED;
    break;
  case FILL_N2:
    if (event == EV_PRE_PRESSURIZE_CMD)
      return PRE_PRESSURIZE;
    break;
  case PRE_PRESSURIZE:
    if (event == EV_FILL_OX_CMD)
      return FILL_OX;
    break;
  case FILL_OX:
    if (event == EV_POST_PRESSURIZE_CMD)
      return POST_PRESSURIZE;
    break;
  case POST_PRESSURIZE:
    if (event == EV_ARM_CMD)
      return ARMED;
    break;
  case ARMED:
    if (event == EV_FIRE_CMD)
      return IGNITION;
    break;
  case IGNITION:
    if (event == EV_LAUNCH)
      return LAUNCH;
    break;
  case LAUNCH:
    if (event == EV_TAKEOFF)
      return BOOST;
    break;
  case BOOST:
    if (event == EV_BOOST_END)
      return COAST;
    break;
  case COAST:
    if (event == EV_APOGEE)
      return DROGUE_DESCENT;
    break;
  case DROGUE_DESCENT:
    if (event == EV_MAIN_ALTITUDE)
      return MAIN_DESCENT;
    break;
  case MAIN_DESCENT:
    if (event == EV_TOUCHDOWN)
      return TOUCHDOWN;
    break;
  default:
    break;
  }
  // If no valid transition, return current state
  return currentState;
}

int performStateRun(RocketState currentState, StateMachineConfigs *configs, RocketData *data)
{
  data->cortexData.time_since_state_start_ms = millis() - data->cortexData.time_since_state_start_ms; // Update time since state start
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
  case PRE_PRESSURIZE: // Does not open valve, just closes it when threshold is reached
    if (data->hydraUFData.tank_top_pressure >= configs->pre_pressurizing_pressure)
    {
      if (data->hydraUFData.valve_pressurizing == VALVE_OPEN)
      {
        // Close all valves
      }
    }
    break;
  case FILL_OX: // Does not open fill valve, just controls venting and stops filling if temperature is too low
    if (data->hydraLFData.probe_temperature_2 < configs->ox_fill_temperature)
    {
      // Close all valves
    }
    else
    {
      if (data->hydraUFData.tank_top_pressure >= configs->ox_fill_pressure)
      {
        // Open vent valve (without closing fill valve)
      }
      else
      {
        // Close vent valve (without closing fill valve)
      }
    }
    break;
  case POST_PRESSURIZE: // Opens and closes pressurizing valve to maintain pressure
    if (data->hydraUFData.tank_top_pressure >= configs->post_pressurizing_pressure)
    {
      // Close pressurizing valve
    } else {
      // Open pressurizing valve
    }
    break;
  case ARMED:
    if (data->cortexData.time_since_state_start_ms >= configs->arming_timeout_ms)
    {
      // Go to IDLE state
      RocketEvent stopEvent = EV_STOP_CMD;
      xQueueSend(EventQueue, &stopEvent, portMAX_DELAY);
    }
    break;
  case IGNITION:
    if (data->cortexData.time_since_state_start_ms >= configs->ignition_timeout_ms)
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
  data->cortexData.time_since_state_start_ms = 0; // Reset state timer on entry
  switch (currentState)
  {
  case IDLE:
    // Close all valves
    break;
  case ABORT:
    // Open abort valve and close all other valves
    break;
  case FILL_N2:
    // Open N2 fill valve
    break;
  case PRE_PRESSURIZE:
    if(data->hydraUFData.tank_top_pressure < configs->pre_pressurizing_pressure) {
      // Open pressurizing valve
    }
    break;
  case FILL_OX:
    if(data->hydraLFData.probe_temperature_2 >= configs->ox_fill_temperature) {
      // Open OX fill valve
    }
    break;
  case POST_PRESSURIZE:
    if(data->hydraUFData.tank_top_pressure < configs->post_pressurizing_pressure) {
      // Open pressurizing valve
    }
    break;
  case ARMED:
    // On entry does nothing
    break;
  case IGNITION:
    // On entry does nothing
    break;
  case LAUNCH:
    // Open main valve
    break;
  case BOOST:
    // On entry does nothing
    break;
  case COAST:
    // Close main valve
    break;
  case DROGUE_DESCENT:
    // Deploy drogue chute
    break;
  case MAIN_DESCENT:
    // Deploy main chute
    break;
  case TOUCHDOWN:
    // On entry does nothing
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
  RocketState currentState = IDLE;
  RocketEvent receivedEvent;
  
  //Serial.println("[TASK] StateMachine task started");
  while (true)
  {
    //Serial.println("[TASK] Running: StateMachine");
    // Receive events from EventQueue
    if (xQueueReceive(EventQueue, &receivedEvent, pdMS_TO_TICKS(100)) == pdTRUE)
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
      vTaskDelay(pdMS_TO_TICKS(10)); // No event received, delay to allow other tasks to run
    }
  }
}
