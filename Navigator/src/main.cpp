#include <Arduino.h>
#include <ArduinoEigen.h>
#include "Comms.h"
#include "Sensors.h"
#include "Display.h"
#include "Sensors/buzzer.h"
//#include "runkalman.h"
//#include "func.h"
//#include "quaternion.h" 
#include <MadgwickAHRS.h>

Madgwick filter;

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

#define POLL_INTERVAL_MS 10

extern SensorDataResult sensorData;
unsigned long last_poll_time = 0;

<<<<<<< HEAD
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

// =========================
// STATE VECTOR (fixed-point: value × 100)
// =========================
int32_t h = 0;      // altitude
int32_t v = 0;      // velocity

// =========================
// COVARIANCE MATRIX (fixed-point: value × 10000)
// =========================
int32_t P[2][2] = {
  {10000, 0},
  {0, 10000}
};

// =========================
// NOISE PARAMETERS (fixed-point: value × 10000)
// =========================
int32_t Qh = 1000;    // process noise altitude
int32_t Qv = 2000;    // process noise velocity
int32_t R = 5000;    // barometer measurement noise
int32_t Rv = 100;   // Strong trust in velocity measurement
// lower values give more weight to the barometer velocity measurement

// =========================
// Barometer altitude derivative tracking
// =========================
int32_t prev_z = 0;   // previous altitude (fixed-point × 100)

// =========================
// IMU vertical acceleration (from MPU9250)
// =========================
float a_z = 0.0;

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
static unsigned long last_derivative_time = 0;
static int32_t last_vz = 0;

int32_t readAltitudeDerivative(int32_t current_z, float dt) {
  unsigned long now = micros();

  // update at just 100 Hz to prevent noise from exploding velocity
  if (now - last_derivative_time < 5000) {
    return last_vz;
  }

  int32_t dz = current_z - prev_z;  // already in fixed-point ×100
  uint32_t dt_us = now - last_derivative_time;  // microseconds

  last_derivative_time = now;
  prev_z = current_z;

  // dz_dt = dz / dt_us * 1e6 = dz * 1e6 / dt_us
  // Result should be in m/s ×100, so: (dz ×100 / 1e6 s) * 1e6 = dz
  // Actually: dz is already ×100 m, dt_us is microseconds
  // dz_dt [m/s ×100] = dz [m ×100] / dt_us [μs] * 1e6
  last_vz = (dz * 1000000) / (int32_t)dt_us;
  Serial.printf(">dz_dt_raw:%.2f\r\n", last_vz / 100.0f);
  return last_vz;
}


// =========================
// KALMAN PREDICT STEP (fixed-point: h,v × 100; P × 10000)
// =========================
void predict2(int32_t acc, float dt) {

  // State prediction
  // h = h + v * dt + 0.5 * acc * dt^2
  // v = v + acc * dt
  h = h + (int32_t)(v * dt) + (int32_t)(0.5f * acc * dt * dt);
  v = v + (int32_t)(acc * dt);

  // Covariance update
  int32_t P00 = P[0][0];
  int32_t P01 = P[0][1];
  int32_t P10 = P[1][0];
  int32_t P11 = P[1][1];

  P[0][0] = P00 + (int32_t)(dt * (P10 + P01)) + (int32_t)(dt * dt * P11) + Qh;
  P[0][1] = P01 + (int32_t)(dt * P11);
  P[1][0] = P10 + (int32_t)(dt * P11);
  P[1][1] = P11 + Qv;
}

// =========================
// KALMAN UPDATE STEP (BMP280) (fixed-point: all × 10000 for gains)
// =========================
void update2(int32_t z) {

  // Innovation
  int32_t y = z - h;

  // Innovation covariance (P[0][0] × 10000 + R × 10000 = S × 10000)
  int32_t S = P[0][0] + R;

  // Kalman gains (kept as fixed-point ratio, 1.0 = 10000)
  // K0 = P[0][0] / S, K1 = P[1][0] / S
  // Since both are × 10000, ratio is dimensionless but we keep 10000 scale for precision
  int32_t K0 = (P[0][0] * 10000) / S;
  int32_t K1 = (P[1][0] * 10000) / S;

  // Update state
  // h = h + K0 * y, where K0 is × 10000, y is × 100
  // Result: (K0 × 10000) * (y × 100) / 10000 = h + K0 * y / 100... 
  // Actually K0 should be treated as dimensionless ratio
  h = h + (K0 * y) / 10000;
  v = v + (K1 * y) / 10000;  // NEGATIVE: if altitude is high, velocity should decrease (was going too fast up)

  // Update covariance
  int32_t P00 = P[0][0];
  int32_t P01 = P[0][1];

  P[0][0] = P00 - (K0 * P00) / 10000;
  P[0][1] = P01 - (K0 * P01) / 10000;
  P[1][0] = P[1][0] - (K1 * P00) / 10000;
  P[1][1] = P[1][1] - (K1 * P01) / 10000;
}

// =========================
// KALMAN UPDATE STEP - VELOCITY (from barometer dz/dt) (fixed-point: all × 10000 for gains)
// =========================
void update2_velocity(int32_t dz_dt) {
  // Innovation: measured velocity - estimated velocity
  int32_t y = dz_dt - v;

  // CLAMP innovation to prevent velocity from exploding
  // Max correction: 200 m/s × 100 per update (20000)
  int32_t max_innovation = 5000;
  if (y > max_innovation) y = max_innovation;
  if (y < -max_innovation) y = -max_innovation;

  // Innovation covariance (P[1][1] × 10000 + Rv × 10000 = S × 10000)
  int32_t S = P[1][1] + Rv;

  // Kalman gains - reduced by factor of 5 for stability
  // K0 = P[0][1] / S, K1 = P[1][1] / S
  int32_t K0 = ((P[0][1] * 10000) / S) ;
  int32_t K1 = ((P[1][1] * 10000) / S) ;

  // Update state
  h = h + (K0 * y) / 10000;
  v = v + (K1 * y) / 10000;

  // Update covariance
  int32_t P01 = P[0][1];
  int32_t P11 = P[1][1];

  P[0][0] = P[0][0] - (K0 * P01) / 10000;
  P[0][1] = P01 - (K1 * P01) / 10000;
  P[1][0] = P[1][0] - (K0 * P11) / 10000;
  P[1][1] = P11 - (K1 * P11) / 10000;
}

// *** Acceleration filtering (fixed-point: × 10000) ***
int32_t alpha_acc_fp = 9000;  // 0.9 as fixed-point (9000/10000)
int32_t az_filtered_fp = 0;   // Fixed-point acceleration

// *** Velocity complementary filter (fixed-point) ***
int32_t alpha_v_fp = 9000;    // 0.90 as fixed-point → 90% barometer, 10% IMU
int32_t v_imu_int_fp = 0;     // Integrated velocity from acceleration (fixed-point)


void setup() {
=======
void setup()
{
>>>>>>> 82799b79a740b71aeff4e4c836784823be307a9d
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

<<<<<<< HEAD
  // Attach interrupt handlers for sensor ready pins
  pinMode(BMP581_RDY_PIN, INPUT);
  pinMode(LPS22DF_RDY_PIN, INPUT);
  pinMode(INT1_ST_IMU, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(BMP581_RDY_PIN), ISR_BMP581_Ready, FALLING);
  attachInterrupt(digitalPinToInterrupt(LPS22DF_RDY_PIN), ISR_LPS22DF_Ready, FALLING);
  attachInterrupt(digitalPinToInterrupt(INT1_ST_IMU), ISR_LSM6DSO_Ready, RISING);
  
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
=======
  play_buzzer_success();

  // while(1);
>>>>>>> 82799b79a740b71aeff4e4c836784823be307a9d
}

// *** Barometer velocity filtering (fixed-point: × 10000) ***
int32_t vz_baro_filtered_fp = 0;
int32_t alpha_baro_fp = 9500;  // 0.95 as fixed-point (9500/10000)

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

<<<<<<< HEAD
  // Acceleration filter (fixed-point, no float operations)
  // az_filtered = 0.9 * az_filtered + 0.1 * acc
  az_filtered_fp = (alpha_acc_fp * az_filtered_fp + (10000 - alpha_acc_fp) * acc) / 10000;
  int32_t acc_filtered = az_filtered_fp;

  // barómetro
  int32_t z = readAltitude();
  int32_t dz_dt_raw = readAltitudeDerivative(z, dt);

  // Barometer velocity filter (fixed-point, no float operations)
  // vz_baro_filtered = 0.95 * vz_baro_filtered + 0.05 * dz_dt_raw
  vz_baro_filtered_fp = (alpha_baro_fp * vz_baro_filtered_fp + (10000 - alpha_baro_fp) * dz_dt_raw) / 10000;
  
  // IMU velocity from acceleration integration (fixed-point)
  // v_imu_int = v_imu_int + acc_filtered * dt
  int32_t dt_scaled = (int32_t)(dt * 1000);  // Convert dt to fixed-point ms
  v_imu_int_fp = v_imu_int_fp + (acc_filtered * dt_scaled) / 1000;
  Serial.printf(">v_imu_integrated:%.2f\r\n", v_imu_int_fp / 100.0f);
  
  // Complementary filter: blend IMU velocity with barometer velocity (fixed-point)
  // vz_complementary = 0.7 * baro + 0.3 * imu
  int32_t dz_dt = (alpha_v_fp * vz_baro_filtered_fp + (10000 - alpha_v_fp) * v_imu_int_fp) / 10000;
  Serial.printf(">v_complementary:%.2f\r\n", dz_dt / 100.0f);
  
  // ---- Kalman ----
  predict2(acc_filtered, dt);
  update2(z);
  update2_velocity(dz_dt);

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
    Serial.printf(">altitude:%.2f,velocity:%.2f\r\n", h / 100.0f, v / 100.0f);
    last_state_print_ms = now_ms;
  }
  /*
  static unsigned long last_dt_print = 0;
  if (now_ms - last_dt_print > 1000) {
    Serial.printf(">dt_ms:%.6f\r\n", dt * 1000);  // Print dt in milliseconds
    last_dt_print = now_ms;
  }*/
=======
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
>>>>>>> 82799b79a740b71aeff4e4c836784823be307a9d
}
