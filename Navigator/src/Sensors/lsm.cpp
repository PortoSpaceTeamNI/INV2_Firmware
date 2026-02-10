#include "Sensors.h"
#include "Sensors/lsm.h"
#include "Pinout.h"
#include <Adafruit_LSM6DS.h>

Adafruit_LSM6DS lsm;

int InitializeLSM6DSO() {
    pinMode(INT1_ST_IMU, INPUT);

    int ret = lsm.begin_I2C(LSM6DSO_ADDR, &Wire1);
    return ret ? 0 : -1;
}

int ConfigureLSM6DSO() {
    lsm.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

    lsm.setGyroDataRate(LSM6DS_RATE_104_HZ);
    lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

    return 0;
}

bool IsLSM6DSOReady() {
    return lsm.accelerationAvailable() || lsm.gyroscopeAvailable();
}

int ReadLSM6DSO(LSM6DSODataResult& result) {
    sensors_event_t accel, gyro, temp;
    lsm.getEvent(&accel, &gyro, &temp);

    result.Temperature = temp.temperature;

    result.AccelX = accel.acceleration.x;
    result.AccelY = accel.acceleration.y;
    result.AccelZ = accel.acceleration.z;

    result.GyroX = gyro.gyro.x;
    result.GyroY = gyro.gyro.y;
    result.GyroZ = gyro.gyro.z;

    return 0;
}

/*
int ConfigureLSM6DSO();
int Read(LSM6DSODataResult& result);
bool IsLSM6DSOReady();
*/