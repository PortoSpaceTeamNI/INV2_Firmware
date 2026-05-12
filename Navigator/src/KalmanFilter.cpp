#include <Arduino.h>
#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() 
  : h(0), v(0),
    Qh(1000), Qv(2000), R(5000), Rv(100),
    alpha_acc_fp(9000), alpha_baro_fp(9500), alpha_v_fp(9000),
    az_filtered_fp(0), vz_baro_filtered_fp(0), v_imu_int_fp(0),
    prev_z(0), last_derivative_time(0), last_vz(0) {
  
  // Initialize covariance matrix
  P[0][0] = 10000;  // P_hh
  P[0][1] = 0;      // P_hv
  P[1][0] = 0;      // P_vh
  P[1][1] = 10000;  // P_vv
}

void KalmanFilter::reset() {
  h = 0;
  v = 0;
  P[0][0] = 10000;
  P[0][1] = 0;
  P[1][0] = 0;
  P[1][1] = 10000;
  
  az_filtered_fp = 0;
  vz_baro_filtered_fp = 0;
  v_imu_int_fp = 0;
  prev_z = 0;
  last_derivative_time = 0;
  last_vz = 0;
}

// =========================
// KALMAN PREDICT STEP (fixed-point: h,v × 100; P × 10000)
// =========================
void KalmanFilter::predict(int32_t acc, float dt) {
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
// KALMAN UPDATE STEP (Altitude) (fixed-point: all × 10000 for gains)
// =========================
void KalmanFilter::update(int32_t z) {
  // Innovation
  int32_t y = z - h;

  // Innovation covariance (P[0][0] × 10000 + R × 10000 = S × 10000)
  int32_t S = P[0][0] + R;

  // Kalman gains (kept as fixed-point ratio, 1.0 = 10000)
  int32_t K0 = (P[0][0] * 10000) / S;
  int32_t K1 = (P[1][0] * 10000) / S;

  // Update state
  h = h + (K0 * y) / 10000;
  v = v + (K1 * y) / 10000;

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
void KalmanFilter::updateVelocity(int32_t dz_dt) {
  // Innovation: measured velocity - estimated velocity
  int32_t y = dz_dt - v;

  // CLAMP innovation to prevent velocity from exploding
  int32_t max_innovation = 5000;
  if (y > max_innovation) y = max_innovation;
  if (y < -max_innovation) y = -max_innovation;

  // Innovation covariance (P[1][1] × 10000 + Rv × 10000 = S × 10000)
  int32_t S = P[1][1] + Rv;

  // Kalman gains
  int32_t K0 = ((P[0][1] * 10000) / S);
  int32_t K1 = ((P[1][1] * 10000) / S);

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

// =========================
// FILTER ACCELERATION (fixed-point: × 10000)
// =========================
int32_t KalmanFilter::filterAcceleration(int32_t acc) {
  // az_filtered = 0.9 * az_filtered + 0.1 * acc
  az_filtered_fp = (alpha_acc_fp * az_filtered_fp + (10000 - alpha_acc_fp) * acc) / 10000;
  return az_filtered_fp;
}

// =========================
// COMPUTE ALTITUDE DERIVATIVE FROM BAROMETER
// Returns: dz/dt in m/s × 100 (fixed-point)
// =========================
int32_t KalmanFilter::computeAltitudeDerivative(int32_t current_z, float dt) {
  unsigned long now = micros();

  // Update at just 100 Hz to prevent noise from exploding velocity
  if (now - last_derivative_time < 5000) {
    return last_vz;
  }

  int32_t dz = current_z - prev_z;  // already in fixed-point ×100
  uint32_t dt_us = now - last_derivative_time;  // microseconds

  last_derivative_time = now;
  prev_z = current_z;

  // dz_dt [m/s ×100] = dz [m ×100] / dt_us [μs] * 1e6
  last_vz = (dz * 1000000) / (int32_t)dt_us;
  
  return last_vz;
}

// =========================
// FILTER BAROMETER VELOCITY (fixed-point: × 10000)
// =========================
int32_t KalmanFilter::filterBarometerVelocity(int32_t dz_dt_raw) {
  // vz_baro_filtered = 0.95 * vz_baro_filtered + 0.05 * dz_dt_raw
  vz_baro_filtered_fp = (alpha_baro_fp * vz_baro_filtered_fp + (10000 - alpha_baro_fp) * dz_dt_raw) / 10000;
  return vz_baro_filtered_fp;
}

// =========================
// INTEGRATE IMU VELOCITY FROM ACCELERATION
// =========================
void KalmanFilter::integrateIMUVelocity(int32_t acc_filtered, float dt) {
  // v_imu_int = v_imu_int + acc_filtered * dt
  int32_t dt_scaled = (int32_t)(dt * 1000);  // Convert dt to fixed-point ms
  v_imu_int_fp = v_imu_int_fp + (acc_filtered * dt_scaled) / 1000;
}

// =========================
// COMPLEMENTARY FILTER: BLEND BAROMETER AND IMU VELOCITY
// =========================
int32_t KalmanFilter::getComplementaryVelocity(int32_t baro_v_filtered, int32_t imu_v_integrated) {
  // vz_complementary = 0.9 * baro + 0.1 * imu
  // (alpha_v_fp = 9000 means 90% barometer, 10% IMU)
  return (alpha_v_fp * baro_v_filtered + (10000 - alpha_v_fp) * imu_v_integrated) / 10000;
}
