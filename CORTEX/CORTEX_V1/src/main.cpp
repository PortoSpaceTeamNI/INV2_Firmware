#include <Arduino.h>
#include <FreeRTOS.h>

#include "Queues.h"
#include "Tasks.h"
#include "Mutexes.h"

void halt() {
  Serial.println("Stopping program");
  // TODO: Buzzer high
  while(1) {
    // Infinite loop to halt the program
    Serial.println("System halted. Please reset.");
    delay(5000);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("System Initializing...");

  // debug only
  while(!Serial) {
    // wait for serial port to connect. Needed for native USB
  }

  if (createQueues() != 0) {
    Serial.println("Failed to create one or more queues.");
    halt();
  } else {
    Serial.println("Queues created successfully.");
  }

  if (createMutexes() != 0) {
    Serial.println("Failed to create one or more mutexes.");
    halt();
  } else {
    Serial.println("Mutexes created successfully.");
  }

  if (createTasks() != 0) {
    Serial.println("Failed to create one or more tasks.");
    halt();
  } else {
    Serial.println("Tasks created successfully.");
  }

}

void loop() {
  // Empty. Tasks are running in FreeRTOS.
}

// Second core setup
void setup1() {
  pinMode(LED_BUILTIN, OUTPUT);
}

// Second core loop (blinks the built-in LED)
void loop1() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}