#ifndef RUNKALMAN_H
#define RUNKALMAN_H
#include <ArduinoEigen.h>

extern Eigen::MatrixXf x;

void calibrateIMU(SensorDataResult *sensorData);
void average_acc_gyro(SensorDataResult *sensorData);
void initKalmanMatrices();
void runKalmanFilter(SensorDataResult *sensorData,  Eigen::MatrixXf *x, uint32_t Ts_us);

#endif // RUNKALMAN_H