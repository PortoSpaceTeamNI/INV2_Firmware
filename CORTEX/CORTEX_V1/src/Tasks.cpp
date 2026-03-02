#include <FreeRTOS.h>
#include <task.h>

#include "Tasks.h"
#include "MissionControl.h"
#include "StateMachine.h"
#include "DataPolling.h"
#include "RS485.h"

int createTasks() {
  if (xTaskCreate(vMissionControlTask, "MissionControl", 1024, NULL, 3, NULL) != pdPASS) {
    return -1;
  }
  if (xTaskCreate(vStateMachineTask, "StateMachine", 1024, NULL, 2, NULL) != pdPASS) {
    return -1;
  }
  if (xTaskCreate(vDataPollingTask, "DataPolling", 1024, NULL, 1, NULL) != pdPASS) {
    return -1;
  }
  if (xTaskCreate(vRS485Task, "RS485", 1024, NULL, 4, NULL) != pdPASS) {
    return -1;
  }
  return 0;
}