#include <Arduino.h>
#include <FreeRTOS.h>

#include "Pinout.h"
#include "Errors.h"
#include "Configs.h"

#include "Queues.h"
#include "Tasks.h"
#include "Mutexes.h"

#include <stdio.h>

void setup() {
  Serial.begin(115200);
  
  Serial1.setTX(NAV_TX_PIN);
  Serial1.setRX(NAV_RX_PIN);
  Serial1.begin(115200);
  
  Serial2.setTX(WRITE_RS_PIN);
  Serial2.setRX(READ_RS_PIN);
  Serial2.begin(RS485_BAUD_RATE);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ENABLE_RS_PIN, OUTPUT);
  digitalWrite(ENABLE_RS_PIN, RS485_RX_ENABLE_LEVEL); // Start in receive mode

  Serial1.println("System Initializing...");

  delay(1000); // Short delay to ensure Serial is ready

  Serial1.println("Serial communication established.");

  if (createQueues() != 0) {
    Serial1.println("Failed to create one or more queues.");
  } else {
    Serial1.println("Queues created successfully.");
  }

  if (createMutexes() != 0) {
    Serial1.println("Failed to create one or more mutexes.");
  } else {
    Serial1.println("Mutexes created successfully.");
  }

  if (createTasks() != 0) {
    Serial1.println("Failed to create one or more tasks.");
  } else {
    Serial1.println("Tasks created successfully.");
  }

}

void loop() {
  // Empty. Tasks are running in FreeRTOS.
}

// Second core setup
void setup1() {
  pinMode(BUZZER_PIN, OUTPUT);
}

// Second core loop (blinks the built-in LED)
void loop1() {
  //tone(BUZZER_PIN, 1000, 100); // Beep to indicate second core is running
  delay(5000);
}
