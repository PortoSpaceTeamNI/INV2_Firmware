#include <Arduino.h>
#include <FreeRTOS.h>

#include "Pinout.h"
#include "Errors.h"
#include "Configs.h"

#include "Queues.h"
#include "Tasks.h"
#include "Mutexes.h"
#include "Radio.h"

#include <stdio.h>

void setup() {
  Serial2.setTX(DEBUG_TX_PIN);
  Serial2.setRX(DEBUG_RX_PIN);
  Serial2.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);

  Serial.begin(115200);
  
  #if RS_ENABLED
  Serial1.setTX(RS_DI_PIN);
  Serial1.setRX(RS_RO_PIN);
  Serial1.begin(RS485_BAUD_RATE);
  pinMode(RS_TX_ENABLE_PIN, OUTPUT);
  digitalWrite(RS_TX_ENABLE_PIN, RS485_RX_ENABLE_LEVEL); // Start in receive mode
  #endif 

  if (setupRadio() != 0) {
    Serial2.println("Failed to initialize radio.");
    while(true) {
      tone(BUZZER_PIN, 2000, 100); // Beep to indicate radio init failure
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  if (createQueues() != 0) {
    Serial2.println("Failed to create one or more queues.");
  } else {
    Serial2.println("Queues created successfully.");
  }

  if (createMutexes() != 0) {
    Serial2.println("Failed to create one or more mutexes.");
  } else {
    Serial2.println("Mutexes created successfully.");
  }

  if (createTasks() != 0) {
    Serial2.println("Failed to create one or more tasks.");
  } else {
    Serial2.println("Tasks created successfully.");
  }

  tone(BUZZER_PIN, 1000, 100); // Beep to indicate setup is complete
}

void loop() {
  // Empty. Tasks are running in FreeRTOS.
}
