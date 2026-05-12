#pragma once
#include <cstdint>

class KalmanFilter {
public:
  KalmanFilter();
  
  // Kalman prediction step
  void predict(int32_t acc, float dt);
  
  // Kalman update steps
  void update(int32_t altitude);
  void updateVelocity(int32_t velocity);
  
  // Filter raw sensor measurements
  int32_t filterAcceleration(int32_t acc);
  int32_t filterBarometerVelocity(int32_t dz_dt_raw);
  
  // Compute altitude derivative from barometer
  int32_t computeAltitudeDerivative(int32_t current_altitude, float dt);
  
  // Complementary filter: blend barometer and IMU velocity
  int32_t getComplementaryVelocity(int32_t baro_v_filtered, int32_t imu_v_integrated);
  
  // Integrate IMU velocity
  void integrateIMUVelocity(int32_t acc_filtered, float dt);
  
  // Getters
  int32_t getAltitude() const { return h; }
  int32_t getVelocity() const { return v; }
  int32_t getIntegratedIMUVelocity() const { return v_imu_int_fp; }
  int32_t getBarometerVelocity() const { return vz_baro_filtered_fp; }
  
  // Reset state
  void reset();

private:
  // State vector (fixed-point: value × 100)
  int32_t h;  // altitude
  int32_t v;  // velocity
  
  // Covariance matrix (fixed-point: value × 10000)
  int32_t P[2][2];
  
  // Noise parameters (fixed-point: value × 10000)
  int32_t Qh;   // process noise altitude
  int32_t Qv;   // process noise velocity
  int32_t R;    // barometer measurement noise
  int32_t Rv;   // velocity measurement noise
  
  // Filter coefficients (fixed-point)
  int32_t alpha_acc_fp;      // acceleration filter (0.9)
  int32_t alpha_baro_fp;     // barometer velocity filter (0.95)
  int32_t alpha_v_fp;        // velocity complementary filter (0.9)
  
  // Filtered states (fixed-point: × 10000 or × 100)
  int32_t az_filtered_fp;        // filtered acceleration
  int32_t vz_baro_filtered_fp;   // filtered barometer velocity
  int32_t v_imu_int_fp;         // integrated IMU velocity
  
  // Barometer derivative tracking
  int32_t prev_z;                // previous altitude
  unsigned long last_derivative_time;
  int32_t last_vz;
};
