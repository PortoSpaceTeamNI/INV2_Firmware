#include <Arduino.h>
#include <ArduinoEigen.h>
#include "Comms.h"
#include "Sensors.h"
#include "Display.h"
#include "Sensors/buzzer.h"
#include "runkalman.h"
#include "func.h"
#include "quaternion.h"

#define POLL_INTERVAL_MS 10

extern SensorDataResult sensorData;
unsigned long last_poll_time = 0;

void setup()
{
  Serial.begin(115200); // Start serial communication
  while (!Serial)
  {
    ; // Wait for serial port to connect. Needed for native USB
  }
  Serial.println("Navigator Initiating...");
  setup_buzzer();
  Serial.println("Starting Setup...");

  Wire.setSDA(SDA0_PIN);
  Wire.setSCL(SCL0_PIN);
  Wire.begin();
  Serial.println("I2C0 initialized");

  Wire1.setSDA(SDA1_PIN);
  Wire1.setSCL(SCL1_PIN);
  Wire1.begin();
  Serial.println("I2C1 initialized");

  SPI1.setSCK(SPI1_SCK_PIN);
  SPI1.setTX(SPI1_MOSI_PIN);
  SPI1.setRX(SPI1_MISO_PIN);
  SPI1.begin();
  Serial.println("SPI1 initialized");

  if (InitializeSensors() == 0)
  {
    Serial.println("Sensors initialized.");
  }
  else
    Serial.println("One or more sensors failed.");

  // while (1);

  if (ConfigureSensors() == 0)
  {
    Serial.println("Sensors Configured.");
  }
  else
    Serial.println("One or more sensors failed to configure.");

  play_buzzer_success();

  // while(1);
}

<<<<<<< HEAD
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
=======
void loop()
{
  if ((millis() - last_poll_time) >= POLL_INTERVAL_MS)
  {
    if (ReadSensors() != 0)
      Serial.println("Failed to read data.");
    else
      DisplayData(&sensorData);
    last_poll_time = millis();
  }
>>>>>>> e63f19d299945c536ddde911286e4bd0c8f0860a
}
