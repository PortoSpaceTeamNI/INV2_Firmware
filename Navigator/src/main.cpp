#include <Arduino.h>
#include <ArduinoEigen.h>
#include "Communications.h"
#include "Sensors.h"
#include "Display.h"
#include "Sensors/buzzer.h"
//#include "runkalman.h"
//#include "func.h"
//#include "quaternion.h" 
#include "Pinout.h"
#include <MadgwickAHRS.h>
#include "KalmanFilter.h"

Madgwick filter;
KalmanFilter kalman;

// ====== INTERRUPT-DRIVEN SENSOR READING ======
volatile bool bmp_data_ready = false;
volatile bool lsm_data_ready = false;
volatile bool lps_data_ready = false;
volatile unsigned long bmp_last_read = 0;
volatile unsigned long lsm_last_read = 0;
volatile unsigned long lps_last_read = 0;
const unsigned long SENSOR_TIMEOUT_MS = 50; // 50ms fallback polling timeout

// Kalman frequency tracking
static unsigned long kalman_iterations = 0;
static unsigned long last_freq_print_ms = 0;
const unsigned long FREQ_PRINT_INTERVAL_MS = 1000; // Print frequency every 1 second

// Altitude/velocity output tracking
static unsigned long last_state_print_ms = 0;
const unsigned long STATE_PRINT_INTERVAL_MS = 100; // Print state every 5ms (200 Hz)

void ISR_BMP581_Ready() {
  bmp_data_ready = true;
}

void ISR_LSM6DSO_Ready() {
  lsm_data_ready = true;
}

void ISR_LPS22DF_Ready() {
  lps_data_ready = true;
}

float ax, ay, az;
float gx, gy, gz;

float lin_ax, lin_ay, lin_az;

// Change later: interval is 10ms
#define POLL_INTERVAL_MS 10

extern SensorDataResult sensorData;
unsigned long last_poll_time = 0;

float processSensorData(SensorDataResult* sensorData, Eigen::MatrixXf* x, float Ts_us) {
  filter.updateIMU(sensorData->lsmData.GyroX, sensorData->lsmData.GyroY, sensorData->lsmData.GyroZ,
                    sensorData->lsmData.AccelX, sensorData->lsmData.AccelY, sensorData->lsmData.AccelZ);
  float q0 = filter.q0;
  float q1 = filter.q1;
  float q2 = filter.q2;
  float q3 = filter.q3;

  float gx_s = 2 * (q1*q3 - q0*q2);
  float gy_s = 2 * (q0*q1 + q2*q3);
  float gz_s = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  gx_s *= 9.81;
  gy_s *= 9.81;
  gz_s *= 9.81;

  lin_ax = sensorData->lsmData.AccelX - gx_s;
  lin_ay = sensorData->lsmData.AccelY - gy_s;
  lin_az = sensorData->lsmData.AccelZ - gz_s;
  return lin_az;
}


// =========================
// TIME
// =========================
unsigned long lastTime_us;
float dt;

// ======================================================
// YOU MUST REPLACE THIS WITH YOUR MPU9250 PROCESSING
// - must be gravity removed
// - must be in world frame (vertical axis)
// Returns: acceleration in m/s^2 × 100 (fixed-point)
// ======================================================
int32_t readVerticalAcceleration() {
  return (int32_t)(processSensorData(&sensorData, nullptr, 0) * 100);
}

// =========================
// READ ALTITUDE FROM BMP
// Returns: altitude in meters × 100 (fixed-point)
// =========================
int32_t readAltitude() {
  return (int32_t)(sensorData.bmpData.Altitude * 100);
}

// =========================
// READ ALTITUDE DERIVATIVE FROM BMP
// Returns: dz/dt in m/s × 100 (fixed-point)
// =========================
int32_t readAltitudeDerivative(int32_t current_z, float dt) {
  int32_t dz_dt_raw = kalman.computeAltitudeDerivative(current_z, dt);
  Serial.printf(">dz_dt_raw:%.2f\r\n", dz_dt_raw / 100.0f);
  return dz_dt_raw;
}





void setup() 
{
  Serial.begin(115200); // Start serial communication

  Serial1.setTX(OBC_RX_PIN);
  Serial1.setRX(OBC_TX_PIN);
  Serial1.begin(115200);
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
  Wire.setClock(400000); // Set I2C0 to 400kHz (Fast Mode)
  Serial.println("I2C0 initialized at 400kHz");

  Wire1.setSDA(SDA1_PIN);
  Wire1.setSCL(SCL1_PIN);
  Wire1.begin();
  Wire1.setClock(400000); // Set I2C1 to 400kHz (Fast Mode)
  Serial.println("I2C1 initialized at 400kHz");

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

  // Attach interrupt handlers for sensor ready pins
  pinMode(BAR2_RDY_PIN, INPUT);
  pinMode(BAR1_RDY_PIN, INPUT);
  pinMode(ST_IMU_INT_PIN, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(BAR2_RDY_PIN), ISR_BMP581_Ready, FALLING);
  attachInterrupt(digitalPinToInterrupt(BAR1_RDY_PIN), ISR_LPS22DF_Ready, FALLING);
  attachInterrupt(digitalPinToInterrupt(ST_IMU_INT_PIN), ISR_LSM6DSO_Ready, RISING);
  
  Serial.println("Interrupt handlers attached");

  // Initial sensor reads to populate data
  Serial.println("Performing initial sensor reads...");
  for (int i = 0; i < 10; i++) {
    if (ReadBMP581(sensorData.bmpData) == 0) break;
    delay(10);
  }
  for (int i = 0; i < 10; i++) {
    if (ReadLSM6DSO(sensorData.lsmData) == 0) break;
    delay(10);
  }
  for (int i = 0; i < 10; i++) {
    if (ReadLPS22DF(sensorData.lpsData) == 0) break;
    delay(10);
  }
  // Initialize timeouts to force reads on first loop iteration
  bmp_last_read = millis() - SENSOR_TIMEOUT_MS - 1;
  lsm_last_read = millis() - SENSOR_TIMEOUT_MS - 1;
  lps_last_read = millis() - SENSOR_TIMEOUT_MS - 1;
  Serial.println("Initial sensor reads complete.");

  //initKalmanMatrices();

  Serial.println("Setup complete.");
  //play_buzzer_success();
  filter.begin(7000);
  kalman.reset();
}

unsigned long t_prev = 0;
void loop() {
  unsigned long loop_start = millis();
  // ---- time ----
  unsigned long now_us = micros();
  dt = (now_us - lastTime_us) / 1e6;
  lastTime_us = now_us;

  if (dt <= 0 || dt > 0.05f) dt = 0.01f; // proteção

  unsigned long now_ms = millis();

  // ---- read sensor data (interrupt-driven with fallback polling) ----
  unsigned long sensor_start = millis();
  
  // BMP581: Read on interrupt or if timeout exceeded
  if (bmp_data_ready || (now_ms - bmp_last_read > SENSOR_TIMEOUT_MS)) {
    bmp_data_ready = false;
    if (ReadBMP581(sensorData.bmpData) == 0) {
      bmp_last_read = now_ms;
    }
  }
  
  // LSM6DSO: Read on interrupt or if timeout exceeded
  if (lsm_data_ready || (now_ms - lsm_last_read > SENSOR_TIMEOUT_MS)) {
    lsm_data_ready = false;
    if (ReadLSM6DSO(sensorData.lsmData) == 0) {
      lsm_last_read = now_ms;
    }
  }
  
  // LPS22DF: Read on interrupt or if timeout exceeded
  if (lps_data_ready || (now_ms - lps_last_read > SENSOR_TIMEOUT_MS)) {
    lps_data_ready = false;
    if (ReadLPS22DF(sensorData.lpsData) == 0) {
      lps_last_read = now_ms;
    }
  }
  
  unsigned long sensor_time = millis() - sensor_start;
    
  // ---- sensores ----
  int32_t acc = readVerticalAcceleration();

  // Filter acceleration through Kalman
  int32_t acc_filtered = kalman.filterAcceleration(acc);

  // Read altitude and compute derivative
  int32_t z = readAltitude();
  int32_t dz_dt_raw = readAltitudeDerivative(z, dt);

  // Filter barometer velocity
  int32_t vz_baro_filtered = kalman.filterBarometerVelocity(dz_dt_raw);
  
  // Integrate IMU velocity from filtered acceleration
  kalman.integrateIMUVelocity(acc_filtered, dt);
  
  // Get integrated IMU velocity and compute complementary velocity
  int32_t v_imu_int = kalman.getIntegratedIMUVelocity();
  int32_t dz_dt = kalman.getComplementaryVelocity(vz_baro_filtered, v_imu_int);
  Serial.printf(">v_imu_integrated:%.2f\r\n", v_imu_int / 100.0f);
  Serial.printf(">v_complementary:%.2f\r\n", dz_dt / 100.0f);
  
  // ---- Kalman ----
  kalman.predict(acc_filtered, dt);
  kalman.update(z);
  kalman.updateVelocity(dz_dt);

  // ---- Track and print Kalman frequency (once per second) ----
  kalman_iterations++;
  if (now_ms - last_freq_print_ms >= FREQ_PRINT_INTERVAL_MS) {
    unsigned long frequency = kalman_iterations;
    Serial.printf(">Kalman_Hz:%lu\r\n", frequency);
    kalman_iterations = 0;
    last_freq_print_ms = now_ms;
  }
  
  // ---- Print altitude and velocity (10 times per second) ----
  if (now_ms - last_state_print_ms >= STATE_PRINT_INTERVAL_MS) {
    Serial.printf(">altitude:%.2f,velocity:%.2f\r\n", kalman.getAltitude() / 100.0f, kalman.getVelocity() / 100.0f);
    last_state_print_ms = now_ms;
  }
  /*
  static unsigned long last_dt_print = 0;
  if (now_ms - last_dt_print > 1000) {
    Serial.printf(">dt_ms:%.6f\r\n", dt * 1000);  // Print dt in milliseconds
    last_dt_print = now_ms;
  }*/

  // SOLVE LATER
  if ((millis() - last_poll_time) >= POLL_INTERVAL_MS)
    {
      if (ReadSensors() != 0)
        Serial.println("Failed to read data.");
      else{
        DisplayData(&sensorData);
        if (SendData(&sensorData) != 0) 
        {
          Serial.println("Failed to send data over UART.");
        }
      }
      last_poll_time = millis();
    }
}
