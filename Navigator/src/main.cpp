#include <Arduino.h>
#include <ArduinoEigen.h>
#include "Comms.h"
#include "Sensors.h"
#include "Display.h"
#include "Sensors/buzzer.h"
#include "runkalman.h"
#include "func.h"
#include "quaternion.h"

extern SensorDataResult sensorData;

void setup() {
  Serial.begin(115200); // Start serial communication

  delay(1000);
  while (!Serial && millis() < 3000) { delay(10); } // wait for monitor

  Serial.println("Navigator Initiating...");
  setup_buzzer();
  Serial.println("Starting Setup...");

  InitializeSensorUART();
  Serial.println("Sensor UART initialized.");

  Wire.setSDA(I2C_SDA_PIN0);
  Wire.setSCL(I2C_SCL_PIN0);
  Wire.begin();
  Serial.println("I2C0 initialized");

  Wire1.setSDA(I2C_SDA_PIN1);
  Wire1.setSCL(I2C_SCL_PIN1);
  Wire1.begin();
  Serial.println("I2C1 initialized");

  if (InitializeSensors() == 0) {
    Serial.println("Sensors initialized.");
  } else Serial.println("One or more sensors failed.");

  if (ConfigureSensors() == 0) {
    Serial.println("Sensors Configured.");
  } else Serial.println("One or more sensors failed to configure.");

  Serial.println("Setup complete.");
  //play_buzzer_success();
}

unsigned long t_prev = 0;
void loop() {

  unsigned long t_now = micros(); 
  float Ts = (t_now - t_prev) * 1e-6f; 
  t_prev = t_now;

  if (ReadSensors() != 0) {
    Serial.println("Failed to read data.");
    return;
  } //else DisplayData(&sensorData);
  
  // x = [delta_theta(3), bimu(3), bg(3), v(3), p(3), bias_baro]

  runKalmanFilter(&sensorData, &x, Ts);
  PollAndHandleComms(x(14), x(11), sensorData.lsmData.AccelZ);
  Serial.print(" accx = "); Serial.print(sensorData.lsmData.AccelZ);
  Serial.print(" accy = "); Serial.print(sensorData.lsmData.AccelY);
  Serial.print(" accz = "); Serial.print(sensorData.lsmData.AccelX);
  //Serial.print(" vx = "); Serial.print(x(9));
  //Serial.print(" vy = "); Serial.print(x(10));
  //Serial.print(" vz = "); Serial.print(x(11));
  //Serial.print(" x = "); Serial.print(x(12));
  //Serial.print(" y = "); Serial.print(x(13));
  Serial.print(" z = "); Serial.println(x(14));
  //Serial.print(" Ts = "); Serial.println(Ts);  
}
