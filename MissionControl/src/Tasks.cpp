#include <FreeRTOS.h>
#include <task.h>

#include "Tasks.h"
#include "Radio.h"
#include "RS485.h"
#include "ARX.h"
#include "Buttons.h"

int createTasks() {
  if (xTaskCreate(vRadioTask, "Radio", 1024, NULL, 3, NULL) != pdPASS) {
    return -1;
  }

  #if RS_ENABLED
  if (xTaskCreate(vRS485Task, "RS485", 1024, NULL, 4, NULL) != pdPASS) {
    return -1;
  }
  #endif

  if (xTaskCreate(vARXTask, "ARX", 1024, NULL, 2, NULL) != pdPASS) {
    return -1;
  } 

  if (xTaskCreate(vButtonsTask, "Buttons", 512, NULL, 5, NULL) != pdPASS) {
    return -1;
  }

  return 0;
}